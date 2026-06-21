#!/usr/bin/env python3
"""cuRobo Motion Planner Node for Doosan E0509

Pick sequence: pre-approach(CuRobo) → straight grasp(MoveLine) → close
               → straight reverse retreat(MoveLine) → pick-start scan pose → pick_complete
"""

import os
import time
import numpy as np
import yaml

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray, String, Empty
from dsr_msgs2.srv import MoveSplineJoint, MoveJoint, MoveLine, ChangeOperationSpeed
try:
    from dsr_gripper_tcp_interfaces.action import SafeGrasp as _SafeGraspAction
    from dsr_gripper_tcp_interfaces.srv import SetPosition as _SetPosition
    from dsr_gripper_tcp_interfaces.srv import GetState as _GetState
    _SAFE_GRASP_AVAILABLE = True
except ImportError:
    _SAFE_GRASP_AVAILABLE = False
    _SafeGraspAction = None
    _SetPosition = None
    _GetState = None

from curobo.geom.types import Cuboid
from curobo_kinematics_adapter import CuroboKinematicsAdapter
from approach_retreat_policy import (
    build_straight_retreat_steps,
    final_approach_fallback_depths,
    FinalApproachState,
    measured_tcp_approach_distance,
    tool_finish_base_direction,
)
from curobo_planning_adapter import CuroboPlanningAdapter
from doosan_motion_client import DoosanMotionClient
from grasp_candidate_policy import (
    GraspSearchResult,
    grasp_offsets_for_target,
    grasp_quat_variants_for_target,
    grasp_variant_pose,
    leftmost_depth_limited,
    leftmost_rejected_offsets,
    variant_label,
)
from grasp_search_executor import GraspSearchExecutor
from gripper_client import HarvestGripperClient
from harvest_grasp_orientation import published_roll_grasp_candidate
from harvest_math import (
    quat_normalize_wxyz,
    quat_rotate_vec,
)
from harvest_motion_params import *  # noqa: F403 - experiment constants
from harvest_result_policy import (
    allow_place_after_grasp,
    pick_sequence_result_code,
    place_gate_block_reason,
)
from open_stem_descent_policy import compute_open_stem_descent_m
from pick_target_policy import prepare_pick_target
from place_sequence_policy import classify_place_outcome
from planner_bootstrap import build_curobo_motion_gen, declare_and_load_params
from runtime_jsonl_logger import RuntimeJsonlLogger
from scene_obstacle_manager import SceneObstacleManager
from tray_place_executor import TrayPlaceExecutor
from tray_place_policy import TrayPlacePolicy
from trajectory_guards import TrajectoryGuards


def resolve_environment_yaml():
    candidates = [
        os.path.expanduser("~/doosan_ws/src/e0509_gripper_description/config/environment.yaml"),
        os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "config", "environment.yaml",
        ),
    ]
    for path in candidates:
        if os.path.exists(path):
            return path
    return candidates[0]


ENVIRONMENT_YAML = resolve_environment_yaml()


def load_environment_cuboids():
    if not os.path.exists(ENVIRONMENT_YAML):
        return [Cuboid(name="table", pose=[0.0, 0.0, -0.02, 1, 0, 0, 0], dims=[1.2, 1.2, 0.04])]
    with open(ENVIRONMENT_YAML, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    cuboids = []
    for obj in data.get("objects", []):
        if not obj.get("enabled", True):
            continue
        if obj.get("type", "cuboid") != "cuboid":
            continue
        try:
            cuboids.append(Cuboid(
                name=str(obj["name"]),
                pose=[float(v) for v in obj["pose"]],
                dims=[float(v) for v in obj["dims"]],
            ))
        except Exception as e:
            print(f"[WARN] environment object skipped: {obj.get('name', '?')} ({e})")
    if not cuboids:
        cuboids.append(Cuboid(name="table", pose=[0.0, 0.0, -0.02, 1, 0, 0, 0], dims=[1.2, 1.2, 0.04]))
    return cuboids


class CuroboPlanner(Node):

    JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

    JOINT_LIMITS = [
        (-6.273185, 6.273185),
        (-1.648063, 1.648063),
        (-2.6953,   2.6953  ),
        (-6.273185, 6.273185),
        (-2.346194, 2.346194),
        (-6.273185, 6.273185),
    ]

    def __init__(self):
        super().__init__("curobo_planner_node")

        self.runtime_log = RuntimeJsonlLogger(self.get_name())
        self.service_cb_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self.current_joints = None
        self._pick_busy = False
        self._sequence_hold_reason = None
        self._last_sequence_hold_warn_sec = 0.0
        self._marker_place_slot_idx = 0
        self.scene_manager = SceneObstacleManager(
            node=self,
            runtime_log=self.runtime_log,
            motion_gen=None,
            static_cuboids=load_environment_cuboids(),
        )

        # ── cuRobo 초기화 ──────────────────────────────────────────────────────
        self.declare_parameter("tool_model_profile", "measured_tcp_260mm")
        self._tool_model_profile = str(
            self.get_parameter("tool_model_profile").value).strip()
        if self._tool_model_profile not in {"measured_tcp_260mm", "legacy_160mm"}:
            raise ValueError(
                "tool_model_profile must be measured_tcp_260mm or legacy_160mm")
        self._measured_tcp_model = self._tool_model_profile == "measured_tcp_260mm"
        self._ee_to_tcp_offset_m = (
            0.0 if self._measured_tcp_model else LEGACY_EE_TO_TCP_OFFSET_M
        )
        self.declare_parameter("measured_tcp_plan_only", True)
        self._measured_tcp_plan_only = bool(
            self.get_parameter("measured_tcp_plan_only").value)
        self.motion_gen = build_curobo_motion_gen(
            self._measured_tcp_model, self.static_cuboids)
        self.scene_manager.set_motion_gen(self.motion_gen)
        self.kinematics_adapter = CuroboKinematicsAdapter(self.motion_gen)
        self.get_logger().info("cuRobo MotionGen warmed up!")

        declare_and_load_params(self, _SAFE_GRASP_AVAILABLE)

        # ── ROS2 인터페이스 ────────────────────────────────────────────────────
        self.create_subscription(
            JointState, "/dsr01/joint_states", self.joint_state_cb, 10,
            callback_group=self.service_cb_group)
        self.create_subscription(
            PoseStamped, "/dsr01/curobo/target_pose", self.target_pose_cb, 10,
            callback_group=self.service_cb_group)
        self.create_subscription(
            PoseStamped, "/dsr01/curobo/pick_pose", self.pick_pose_cb, 10,
            callback_group=self.service_cb_group)
        self.create_subscription(
            String, "/dsr01/curobo/obstacles", self.obstacles_cb, 10,
            callback_group=self.service_cb_group)
        self.create_subscription(
            Float64MultiArray, "/strawberry/detection/scene_positions", self._scene_cb, 10,
            callback_group=self.service_cb_group)
        self.create_subscription(
            String, "/strawberry/scan/status", self._scan_status_cb, 10,
            callback_group=self.service_cb_group)
        self.create_subscription(
            String, "/strawberry/exploration/set_cell_state", self._cell_state_cb, 10,
            callback_group=self.service_cb_group)

        self.pick_complete_pub = self.create_publisher(Empty, "/dsr01/curobo/pick_complete", 10)

        self.cli_spline = self.create_client(
            MoveSplineJoint, "/dsr01/motion/move_spline_joint",
            callback_group=self.service_cb_group)
        self.cli_movej = self.create_client(
            MoveJoint, "/dsr01/motion/move_joint",
            callback_group=self.service_cb_group)
        self.cli_movel = self.create_client(
            MoveLine, "/dsr01/motion/move_line",
            callback_group=self.service_cb_group)
        if _SAFE_GRASP_AVAILABLE:
            self.cli_set_position = self.create_client(
                _SetPosition, "/gripper_service/set_position",
                callback_group=self.service_cb_group)
            self.cli_get_state = self.create_client(
                _GetState, "/gripper_service/get_state",
                callback_group=self.service_cb_group)
        else:
            self.cli_set_position = None
            self.cli_get_state = None
        if _SAFE_GRASP_AVAILABLE:
            self._safe_grasp_cli = ActionClient(
                self, _SafeGraspAction, "/gripper_service/safe_grasp",
                callback_group=self.service_cb_group)
        else:
            self._safe_grasp_cli = None
        self.gripper_client = HarvestGripperClient(
            node=self,
            runtime_log=self.runtime_log,
            cli_set_position=self.cli_set_position,
            cli_get_state=self.cli_get_state,
            safe_grasp_cli=self._safe_grasp_cli,
            safe_grasp_action_type=_SafeGraspAction,
            use_safe_grasp_action=self._use_safe_grasp_action,
            safe_grasp_max_current=self._safe_grasp_max_current,
            safe_grasp_current_delta_threshold=self._safe_grasp_current_delta_threshold,
            safe_grasp_timeout_sec=self._safe_grasp_timeout_sec,
            grasp_current_contact_threshold_raw=self._grasp_current_contact_threshold_raw,
            set_position_type=_SetPosition,
            get_state_type=_GetState,
        )
        self.grasp_search_executor = GraspSearchExecutor(
            node=self,
            runtime_log=self.runtime_log,
            plan_fn=self.plan,
            measured_tcp_max_approach_m=self._measured_tcp_max_approach_m,
            ee_to_tcp_offset_m=self._ee_to_tcp_offset_m,
        )
        self.tray_place_policy = TrayPlacePolicy(
            node=self,
            runtime_log=self.runtime_log,
            tray_cells_json=self._tray_cells_json,
            marker_place_max_age_sec=self._marker_place_max_age_sec,
            marker_place_above_clearance_m=self._marker_place_above_clearance_m,
            measured_tcp_model=self._measured_tcp_model,
        )
        self.tray_place_executor = TrayPlaceExecutor(
            node=self,
            runtime_log=self.runtime_log,
            plan_fn=self.plan,
            execute_spline_fn=self.execute_spline,
            execute_base_z_relative_fn=self.execute_base_z_relative,
            plan_to_fixed_joints_pose_fn=self.plan_to_fixed_joints_pose,
            overview_joints_near_current_fn=self.overview_joints_near_current,
            nearest_equivalent_joints_fn=self._nearest_equivalent_joints,
            curobo_fk_ee_pose_fn=self._curobo_fk_ee_pose,
            trajectory_line_deviation_fn=self._trajectory_line_deviation_mm,
            ensure_operation_speed_fn=self._ensure_operation_speed,
            set_gripper_position_fn=self._set_gripper_position,
            current_joints_getter=lambda: self.current_joints,
            slot_idx_getter=lambda: self._marker_place_slot_idx,
            tray_place_policy=self.tray_place_policy,
            use_taught_slot0_place_reference=self._use_taught_slot0_place_reference,
            execute_marker_place_release=self._execute_marker_place_release,
            allow_generated_tray_slot_release=self._allow_generated_tray_slot_release,
            measured_tcp_model=self._measured_tcp_model,
            marker_place_above_clearance_m=self._marker_place_above_clearance_m,
            taught_slot_above_clearance_m=self._taught_slot_above_clearance_m,
            row2_place_pitch_tilt_deg=self._row2_place_pitch_tilt_deg,
            row2_release_correction_mm=self._row2_release_correction_mm,
            row2_max_line_deviation_mm=self._row2_max_line_deviation_mm,
        )
        self.trajectory_guards = TrajectoryGuards(
            logger=self.get_logger(),
            joint_names=self.JOINT_NAMES,
        )
        self.planning_adapter = CuroboPlanningAdapter(
            node=self,
            runtime_log=self.runtime_log,
            motion_gen=self.motion_gen,
            joint_names=self.JOINT_NAMES,
            joint_limits=self.JOINT_LIMITS,
            trajectory_guards=self.trajectory_guards,
            world_state_getter=lambda: (
                self.static_cuboids,
                self.dynamic_cuboids,
                self.neighbor_spheres,
            ),
            debug_dump_plan_calls=self._debug_dump_plan_calls,
        )
        self.cli_change_op_speed = self.create_client(
            ChangeOperationSpeed, "/dsr01/motion/change_operation_speed",
            callback_group=self.service_cb_group)
        self.motion_client = DoosanMotionClient(
            node=self,
            runtime_log=self.runtime_log,
            cli_spline=self.cli_spline,
            cli_movej=self.cli_movej,
            cli_movel=self.cli_movel,
            current_joints_getter=lambda: self.current_joints,
        )

        self.get_logger().info("cuRobo Planner Ready!")
        self.get_logger().info(f"Runtime JSONL: {self.runtime_log.path}")
        self.runtime_log.log(
            "node_start",
            wall_quat_wxyz=WALL_QUAT_WXYZ,
            grasp_retry_offsets_m=GRASP_RETRY_OFFSETS,
            leftmost_grasp_retry_offsets_m=LEFTMOST_GRASP_RETRY_OFFSETS,
            leftmost_grasp_x_correction_m=LEFTMOST_GRASP_X_CORR_M,
            leftmost_extra_advance_request_m=self._leftmost_extra_advance_request_m,
            leftmost_wall_safety_margin_m=self._leftmost_wall_safety_margin_m,
            leftmost_allow_wall_model_override=self._leftmost_allow_wall_model_override,
            pre_approach_offset_m=PRE_APPROACH_OFFSET,
            tool_model_profile=self._tool_model_profile,
            measured_tcp_plan_only=self._measured_tcp_plan_only,
            ee_to_tcp_offset_m=self._ee_to_tcp_offset_m,
            legacy_ee_to_tcp_offset_m=LEGACY_EE_TO_TCP_OFFSET_M,
            measured_flange_to_gripper_m=MEASURED_FLANGE_TO_GRIPPER_M,
            measured_flange_to_part_tip_m=MEASURED_FLANGE_TO_PART_TIP_M,
            measured_flange_to_grasp_center_m=MEASURED_FLANGE_TO_GRASP_CENTER_M,
            tcp_model_shortfall_m=TCP_MODEL_SHORTFALL_M,
            open_stem_descent_m=CRANE_Z_OFFSET_M,
            nw_high_target_base_y_nudge_m=self._nw_high_target_base_y_nudge_m,
            enable_marker_place=self._enable_marker_place,
            execute_marker_place_release=self._execute_marker_place_release,
            use_taught_slot0_place_reference=self._use_taught_slot0_place_reference,
            hold_after_taught_slot0_place=self._hold_after_taught_slot0_place,
            initial_place_slot_index=self._marker_place_slot_idx,
            allow_generated_tray_slot_release=self._allow_generated_tray_slot_release,
            allow_unverified_grasp_place=self._allow_unverified_grasp_place,
            grasp_current_contact_threshold_raw=self._grasp_current_contact_threshold_raw,
            use_published_grasp_orientation=self._use_published_grasp_orientation,
            published_grasp_roll_align_axis=self._published_grasp_roll_align_axis,
            published_grasp_roll_max_abs_deg=self._published_grasp_roll_max_abs_deg,
            marker_place_max_age_sec=self._marker_place_max_age_sec,
        )
        base_approach_dir = np.array(quat_rotate_vec(WALL_QUAT_WXYZ, [0.0, 0.0, 1.0]))
        base_elevation_deg = float(np.degrees(np.arcsin(np.clip(base_approach_dir[2], -1.0, 1.0))))
        self.get_logger().info(
            f"  ENV_CUBOIDS={len(self.static_cuboids)}  "
            f"SELF_COLLISION={USE_CUROBO_SELF_COLLISION}")
        self.get_logger().warn(
            "Leaf/stem geometry is not in the cuRobo world; visually occluded "
            "targets require reobserve/skip instead of forced approach")
        self.get_logger().info(
            f"  WALL_QUAT_WXYZ={WALL_QUAT_WXYZ} "
            f"approach_dir={np.round(base_approach_dir, 4).tolist()} "
            f"elevation={base_elevation_deg:+.1f}deg  "
            f"variants={len(self.grasp_quat_variants())}")
        self.get_logger().info(
            f"  LEFTMOST horizontal fallback x_corr="
            f"{LEFTMOST_GRASP_X_CORR_M*1000:+.0f}mm "
            f"offsets_mm={[round(v*1000) for v in LEFTMOST_GRASP_RETRY_OFFSETS]} "
            f"extra_request={self._leftmost_extra_advance_request_m*1000:.0f}mm "
            f"wall_margin={self._leftmost_wall_safety_margin_m*1000:.0f}mm "
            f"wall_override={self._leftmost_allow_wall_model_override} "
            f"pre_approach={PRE_APPROACH_OFFSET*1000:.0f}mm")
        if self._measured_tcp_model:
            self.get_logger().warn(
                "  TOOL_MODEL=measured_tcp_260mm: cuRobo ee_link is the measured "
                "grasp center; legacy length compensation and default extra advance "
                f"are disabled. plan_only={self._measured_tcp_plan_only}.")
            self.get_logger().info(
                "  MEASURED_TCP_FINAL_APPROACH "
                f"direct_curobo={self._direct_curobo_final_approach_for_measured_tcp} "
                f"max={self._measured_tcp_max_approach_m*1000:.0f}mm "
                f"tool_line_after_fallback="
                f"{self._measured_tcp_tool_line_after_curobo_fallback}")
            self.get_logger().info(
                "  PUBLISHED_GRASP_ORIENTATION "
                f"enabled={self._use_published_grasp_orientation} "
                f"roll_align_tool_{self._published_grasp_roll_align_axis} "
                f"max_abs_roll={self._published_grasp_roll_max_abs_deg:.0f}deg")
            self.get_logger().info(
                f"  OPEN_STEM_DESCENT={CRANE_Z_OFFSET_M*1000:.0f}mm: "
                "horizontal approach above KP1 -> open BASE -Z descent -> close at KP1")
            self.get_logger().warn(
                "  NW_HIGH_TARGET_CORRECTION "
                f"z>={self._nw_high_target_z_threshold_m*1000:.0f}mm: "
                f"final_extra={self._nw_high_target_final_extra_m*1000:.0f}mm "
                f"base_y_nudge={self._nw_high_target_base_y_nudge_m*1000:.0f}mm "
                f"y_plane_relax={NW_HIGH_TARGET_Y_PLANE_RELAX_M*1000:.0f}mm "
                f"crane_z_offset={self._nw_high_target_crane_z_offset_m*1000:.0f}mm "
                f"(SW/default={CRANE_Z_OFFSET_M*1000:.0f}mm)")
        else:
            self.get_logger().warn(
                "  TOOL_GEOMETRY_LEGACY: planner offset="
                f"{LEGACY_EE_TO_TCP_OFFSET_M*1000:.0f}mm, measured grasp center="
                f"{MEASURED_FLANGE_TO_GRASP_CENTER_M*1000:.0f}mm "
                f"(model shortfall={TCP_MODEL_SHORTFALL_M*1000:.0f}mm).")
        if os.path.exists(ENVIRONMENT_YAML):
            self.get_logger().info(f"  environment loaded: {ENVIRONMENT_YAML}")
        self.get_logger().info(
            f"  marker place: enabled={self._enable_marker_place} "
            f"release={self._execute_marker_place_release} "
            f"taught_slot0_reference={self._use_taught_slot0_place_reference} "
            f"hold_after_slot0={self._hold_after_taught_slot0_place} "
            f"allow_generated_slot_release={self._allow_generated_tray_slot_release} "
            f"allow_unverified_grasp={self._allow_unverified_grasp_place} "
            f"max_age={self._marker_place_max_age_sec:.0f}s")

        # 노드 시작 시 그리퍼를 approach 위치로 초기화 (2s 후 — gripper_service_node 연결 여유)
        self._gripper_init_done = False
        self.create_timer(2.0, self._init_gripper_once)

    @property
    def static_cuboids(self):
        return self.scene_manager.static_cuboids

    @property
    def dynamic_cuboids(self):
        return self.scene_manager.dynamic_cuboids

    @dynamic_cuboids.setter
    def dynamic_cuboids(self, value):
        self.scene_manager.dynamic_cuboids = value

    @property
    def neighbor_spheres(self):
        return self.scene_manager.neighbor_spheres

    @neighbor_spheres.setter
    def neighbor_spheres(self, value):
        self.scene_manager.neighbor_spheres = value

    def _init_gripper_once(self):
        if not self._gripper_init_done:
            self._gripper_init_done = True
            self._reset_gripper()

    def _reset_gripper(self):
        self.gripper_client.reset()

    def _set_gripper_position(self, position: int, timeout_sec: float = 5.0) -> bool:
        return self.gripper_client.set_position(position, timeout_sec)

    def _abort_pick_with_complete(self, reset_gripper: bool = True):
        self._clear_neighbor_obstacles()
        if reset_gripper:
            self._reset_gripper()
        self.pick_complete_pub.publish(Empty())

    # ── 콜백 ──────────────────────────────────────────────────────────────────

    def joint_state_cb(self, msg: JointState):
        jmap = {n: p for n, p in zip(msg.name, msg.position)}
        joints = [jmap.get(n) for n in self.JOINT_NAMES]
        if None not in joints:
            self.current_joints = joints

    def target_pose_cb(self, msg: PoseStamped):
        if self.current_joints is None:
            self.get_logger().warn("No joint state yet")
            return
        p, o = msg.pose.position, msg.pose.orientation
        ret = self.plan(self.current_joints, [p.x, p.y, p.z], [o.w, o.x, o.y, o.z])
        if ret is not None:
            self.execute_spline(*ret)

    def obstacles_cb(self, msg: String):
        try:
            self.scene_manager.obstacles_from_json(msg.data)
        except Exception as e:
            self.get_logger().error(f"obstacles_cb error: {e}")

    # ── World 관리 ─────────────────────────────────────────────────────────────

    def update_curobo_world(self, reason="manual"):
        self.scene_manager.update_curobo_world(reason)

    def _scene_cb(self, msg: Float64MultiArray) -> None:
        self.scene_manager.update_scene_positions_from_flat_array(msg.data)

    def _scan_status_cb(self, msg: String) -> None:
        self.runtime_log.log("scan_status", text=msg.data)

    def _cell_state_cb(self, msg: String) -> None:
        self.runtime_log.log("cell_state", text=msg.data)

    def _register_neighbor_obstacles(self, target_pos: np.ndarray) -> None:
        self.scene_manager.register_neighbor_obstacles(target_pos)

    def _clear_neighbor_obstacles(self) -> None:
        self.scene_manager.clear_neighbor_obstacles()

    # ── 충돌 진단 ──────────────────────────────────────────────────────────────

    def _check_state_feasible_with_world(self, joints, cuboids):
        return self.planning_adapter.check_state_feasible_with_world(
            joints, cuboids)

    def diagnose_start_world_collision(self, joints, label):
        return self.planning_adapter.diagnose_start_world_collision(joints, label)

    def diagnose_js_endpoint_collision(self, start_joints, target_joints, label):
        return self.planning_adapter.diagnose_js_endpoint_collision(
            start_joints, target_joints, label)

    # ── 유틸 ──────────────────────────────────────────────────────────────────

    def _clamp_joints(self, joints):
        return self.planning_adapter.clamp_joints(joints)

    def grasp_candidates_for_target(self, straw):
        return grasp_offsets_for_target(straw, self._measured_tcp_model)

    def grasp_quat_variants(self):
        return grasp_quat_variants_for_target(self._measured_tcp_model, False)

    def _published_roll_grasp_variant(self, input_quat_wxyz):
        """Return a wall-normal approach quaternion with roll aligned to published stem direction."""
        if not self._use_published_grasp_orientation:
            return None
        candidate = published_roll_grasp_candidate(
            input_quat_wxyz,
            WALL_QUAT_WXYZ,
            align_axis=self._published_grasp_roll_align_axis,
            max_abs_roll_deg=self._published_grasp_roll_max_abs_deg,
        )
        if not candidate.accepted:
            log_kwargs = {
                "reason": candidate.reason,
                "input_quat_wxyz": input_quat_wxyz,
            }
            if candidate.roll_deg is not None:
                log_kwargs["roll_deg"] = candidate.roll_deg
            if candidate.raw_roll_deg is not None:
                log_kwargs["raw_roll_deg"] = candidate.raw_roll_deg
            if candidate.stem_dir_base is not None:
                log_kwargs["stem_dir_base"] = candidate.stem_dir_base
            if candidate.wall_approach_dir is not None:
                log_kwargs["wall_approach_dir"] = candidate.wall_approach_dir
            if candidate.reason == "roll_exceeds_limit":
                log_kwargs["max_abs_roll_deg"] = self._published_grasp_roll_max_abs_deg
                self.get_logger().warn(
                    "PUBLISHED_GRASP_ORIENTATION rejected: "
                    f"roll={candidate.roll_deg:+.1f}deg "
                    f"(raw={candidate.raw_roll_deg:+.1f}deg) exceeds "
                    f"{self._published_grasp_roll_max_abs_deg:.1f}deg")
            self.runtime_log.log(
                "published_grasp_orientation_rejected",
                **log_kwargs,
            )
            return None

        self.get_logger().info(
            "PUBLISHED_GRASP_ORIENTATION candidate: "
            f"align_tool_{self._published_grasp_roll_align_axis} "
            f"roll={candidate.roll_deg:+.1f}deg raw={candidate.raw_roll_deg:+.1f}deg "
            f"approach_error={candidate.approach_error_deg:.2f}deg")
        self.runtime_log.log(
            "published_grasp_orientation_candidate",
            input_quat_wxyz=input_quat_wxyz,
            candidate_quat_wxyz=candidate.candidate_quat_wxyz,
            stem_dir_base=candidate.stem_dir_base,
            wall_approach_dir=candidate.wall_approach_dir,
            roll_align_axis=self._published_grasp_roll_align_axis,
            roll_deg=candidate.roll_deg,
            raw_roll_deg=candidate.raw_roll_deg,
            approach_error_deg=candidate.approach_error_deg,
        )
        return ("published_roll", candidate.candidate_quat_wxyz, candidate.roll_deg)

    def _verify_grasp(self):
        return self.gripper_client.verify_grasp()

    def _close_and_verify_grasp(self):
        return self.gripper_client.close_and_verify_grasp()

    def _close_via_safe_grasp_action(self):
        return self.gripper_client.close_via_safe_grasp_action()

    # ── 플래닝 ────────────────────────────────────────────────────────────────

    def trajectory_in_operational_limits(self, traj_rad, label):
        return self.trajectory_guards.in_operational_limits(traj_rad, label)

    def trajectory_has_reasonable_swing(
            self, traj_rad, start_joints, label,
            max_joint_delta_deg=None):
        return self.trajectory_guards.has_reasonable_swing(
            traj_rad, start_joints, label, max_joint_delta_deg)

    def normalize_trajectory_equivalents(self, traj_rad, label, robot_start_joints_rad=None):
        return self.trajectory_guards.normalize_equivalents(
            traj_rad, label, robot_start_joints_rad)

    def trajectory_has_no_spline_jumps(self, traj_rad, label, max_jump_deg=270.0):
        return self.trajectory_guards.has_no_spline_jumps(
            traj_rad, label, max_jump_deg)

    def plan(self, start_joints, target_pos, target_quat_wxyz, num_ik_seeds=32,
             max_attempts=None, timeout_sec=None, max_joint_delta_deg=None):
        return self.planning_adapter.plan(
            start_joints,
            target_pos,
            target_quat_wxyz,
            num_ik_seeds=num_ik_seeds,
            max_attempts=max_attempts,
            timeout_sec=timeout_sec,
            max_joint_delta_deg=max_joint_delta_deg,
        )

    def plan_js(self, start_joints, target_joints_rad, label, skip_swing_check=False,
                max_joint_delta_deg=None):
        return self.planning_adapter.plan_js(
            start_joints,
            target_joints_rad,
            label,
            skip_swing_check=skip_swing_check,
            max_joint_delta_deg=max_joint_delta_deg,
        )

    def _ensure_operation_speed(self, speed: int = 30) -> bool:
        """컨트롤러 operation speed를 강제 설정한다.
        Spline req.time이 이행되려면 speed=100 필수.
        서비스 불가 시 경고 후 계속 진행(비차단)."""
        if not self.cli_change_op_speed.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                f"change_operation_speed: service not available; "
                "speed not enforced — ensure teach pendant is at 100%")
            return False
        req = ChangeOperationSpeed.Request()
        req.speed = speed
        future = self.cli_change_op_speed.call_async(req)
        t0 = time.time()
        while not future.done() and (time.time() - t0) < 5.0:
            time.sleep(0.05)
        ok = future.done() and future.result() and future.result().success
        if ok:
            self.get_logger().info(f"Operation speed set to {speed}%")
        else:
            self.get_logger().warn(
                f"change_operation_speed({speed}) failed or timed out; continuing")
        self.runtime_log.log(
            "operation_speed_set",
            requested_speed=speed,
            success=bool(ok),
        )
        return ok

    def execute_spline(self, traj_rad, motion_time: float) -> bool:
        return self.motion_client.execute_spline(traj_rad, motion_time)

    def execute_tool_z_line(self, distance_m: float, motion_label="FINAL_APPROACH_STRAIGHT",
                            vel_mm_s: float = None, acc_mm_s2: float = None,
                            min_distance_m: float = 0.02) -> bool:
        return self.motion_client.execute_tool_z_line(
            distance_m,
            motion_label=motion_label,
            vel_mm_s=vel_mm_s,
            acc_mm_s2=acc_mm_s2,
            min_distance_m=min_distance_m,
        )

    def _execute_pitch_detach(self) -> bool:
        return self.motion_client.execute_pitch_detach()

    def _execute_retreat_step_via_curobo(self, delta_m, motion_label: str) -> bool:
        """Retreat one BASE-frame leg via cuRobo plan instead of a raw relative
        MoveLine. A raw relative line lets the Doosan controller pick whatever
        IK branch it wants for the destination, which can swing joints through
        the operational limit (observed: same 125mm leg only moved J2 ~4deg
        forward but ~15deg in reverse from a lower post-detach pose). cuRobo's
        planner checks operational joint limits before returning a trajectory,
        so an unreachable retreat fails cleanly (existing hold path) instead of
        physically exceeding a joint limit mid-motion."""
        if self.current_joints is None:
            self.get_logger().error(
                f"{motion_label}: no current_joints for cuRobo retreat plan")
            return False
        start_joints = list(self.current_joints)
        current_pos, current_quat = self._curobo_fk_ee_pose(start_joints)
        target_pos = (
            np.array(current_pos, dtype=float)
            + np.array(delta_m, dtype=float)
        ).tolist()
        plan_result = self.plan(start_joints, target_pos, current_quat, num_ik_seeds=32)
        if plan_result is None:
            self.get_logger().error(
                f"{motion_label}: cuRobo retreat plan failed (IK/limits/collision)")
            return False
        ok = self.execute_spline(*plan_result)
        if not ok:
            self.get_logger().error(f"{motion_label}: cuRobo retreat spline exec failed")
        return ok

    def _execute_retreat_steps(self, steps, vel_mm_s=None, acc_mm_s2=None) -> bool:
        for step in steps:
            if step["frame"] == "base":
                ok = self._execute_retreat_step_via_curobo(
                    step["delta_m"],
                    step["label"],
                )
            else:
                ok = self.execute_tool_z_line(
                    step["distance_m"],
                    motion_label=step["label"],
                    vel_mm_s=vel_mm_s or RETREAT_VEL_MM_S,
                    acc_mm_s2=acc_mm_s2 or RETREAT_ACC_MM_S2,
                )
            self.runtime_log.log(
                "retreat_step_complete",
                label=step["label"],
                ok=ok,
                current_joints_rad=self.current_joints,
            )
            if not ok:
                return False
        return True

    def execute_base_z_relative(self, distance_m: float, motion_label: str,
                                vel_mm_s: float = 30.0) -> bool:
        return self.motion_client.execute_base_z_relative(
            distance_m, motion_label, vel_mm_s)

    def execute_base_relative_line(self, delta_m, motion_label: str,
                                   vel_mm_s: float = 30.0,
                                   acc_mm_s2: float = 30.0) -> bool:
        return self.motion_client.execute_base_relative_line(
            delta_m, motion_label, vel_mm_s, acc_mm_s2)

    def execute_base_line(self, posx_mm_deg, motion_label, vel_mm_s=20.0) -> bool:
        return self.motion_client.execute_base_line(
            posx_mm_deg, motion_label, vel_mm_s)

    def _curobo_fk_ee_pose(self, joints_rad):
        """cuRobo robot model 기준 현재 ee_link pose를 반환한다."""
        return self.kinematics_adapter.ee_pose(joints_rad)

    def _trajectory_line_deviation_mm(self, traj_rad, start_pos_m, end_pos_m):
        """FK 궤적의 목표 Cartesian 선분 대비 최대 측방 편차를 계산한다."""
        return self.kinematics_adapter.trajectory_line_deviation_mm(
            traj_rad, start_pos_m, end_pos_m)

    def _nearest_equivalent_joints(self, base_joints_deg):
        """J4/J6를 현재 위치에서 가장 가까운 360° equivalent로 조정."""
        return self.trajectory_guards.nearest_equivalent_joints(
            base_joints_deg, self.current_joints)

    def home_joints_near_current(self):
        return self._nearest_equivalent_joints(HOME_JOINTS_DEG)

    def overview_joints_near_current(self):
        return self._nearest_equivalent_joints(OVERVIEW_JOINTS_DEG)

    def movej_direct(self, joints_deg, vel=40.0, acc=60.0):
        return self.motion_client.movej_direct(joints_deg, vel, acc)

    def plan_to_fixed_joints_pose(self, start_joints, target_joints_deg, label,
                                   skip_swing_check=False,
                                   max_joint_delta_deg=None):
        """고정 joint 자세 이동 — cuRobo joint-space plan."""
        target_joints_rad = np.deg2rad(target_joints_deg).tolist()
        ret = self.plan_js(start_joints, target_joints_rad, label,
                           skip_swing_check=skip_swing_check,
                           max_joint_delta_deg=max_joint_delta_deg)
        if ret is not None and self.execute_spline(*ret):
            return True, ret[0][-1].tolist()
        self.get_logger().warn(f"{label} CuRobo joint-space failed")
        return False, start_joints

    def _latest_tray_cells_json(self):
        return self.tray_place_policy.latest_tray_cells_json()

    def _marker_cell_with_taught_grid_pitch(self, cells, slot_index):
        return self.tray_place_policy.marker_cell_with_taught_grid_pitch(
            cells, slot_index)

    # ── Pick 시퀀스 ────────────────────────────────────────────────────────────

    def _hold_pick_sequence(self, reason: str):
        self._sequence_hold_reason = reason
        self.runtime_log.log(
            "pick_sequence_hold_latched",
            reason=reason,
            current_joints_rad=self.current_joints,
        )
        self.get_logger().warn(
            f"PICK_SEQUENCE_HOLD_LATCHED reason={reason}; "
            "new pick targets are blocked until planner restart")

    def pick_pose_cb(self, msg: PoseStamped):
        if self._sequence_hold_reason is not None:
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            if now_sec - self._last_sequence_hold_warn_sec >= 5.0:
                self._last_sequence_hold_warn_sec = now_sec
                self.get_logger().warn(
                    f"Pick target ignored: sequence hold "
                    f"({self._sequence_hold_reason})")
            return
        if self.current_joints is None:
            self.get_logger().warn("No joint state yet")
            return
        if self._pick_busy:
            self.get_logger().warn("Pick already in progress — ignored")
            return
        self._pick_busy = True
        try:
            self._pick(msg)
        finally:
            self._pick_busy = False

    def _execute_open_stem_descent_if_needed(self, crane_z_offset_m: float,
                                             straw_z_m: float,
                                             used_grasp_ee_pos,
                                             used_grasp_variant) -> bool:
        if not self._measured_tcp_model or crane_z_offset_m <= 0:
            return True

        open_stem_descent_m, descent_info = compute_open_stem_descent_m(
            crane_z_offset_m,
            float(straw_z_m),
            None if used_grasp_ee_pos is None else float(used_grasp_ee_pos[2]),
            self._nw_high_target_descent_extra_below_kp1_m,
        )
        if descent_info["mode"] == "dynamic":
            self.get_logger().warn(
                "OPEN_DESCENT_DYNAMIC: kp1_z="
                f"{descent_info['target_kp1_z_m']*1000:.0f}mm "
                f"reached_z={descent_info['reached_z_m']*1000:.0f}mm "
                f"overshoot={descent_info['overshoot_above_kp1_m']*1000:.0f}mm "
                f"extra_below_kp1={descent_info['extra_below_kp1_m']*1000:.0f}mm "
                f"-> descent={open_stem_descent_m*1000:.0f}mm")
            self.runtime_log.log(
                "nw_high_target_open_descent_dynamic",
                target_kp1_z_m=descent_info["target_kp1_z_m"],
                reached_z_m=descent_info["reached_z_m"],
                overshoot_above_kp1_m=descent_info["overshoot_above_kp1_m"],
                extra_below_kp1_m=descent_info["extra_below_kp1_m"],
                executed_descent_m=descent_info["executed_descent_m"],
                selected_variant=used_grasp_variant,
            )
        self.get_logger().info(
            f"OPEN_STEM_DESCENT — gripper={GRIPPER_APPROACH_POS}, "
            f"BASE -Z {open_stem_descent_m*1000:.0f}mm to KP1")
        if not self.execute_base_z_relative(
                -open_stem_descent_m, "OPEN_STEM_DESCENT", CRANE_DESCENT_VEL_MM_S):
            self.get_logger().error("ABORT: open stem descent 실패")
            self._abort_pick_with_complete()
            return False
        return True

    def _execute_nw_base_y_nudge_if_needed(self, is_nw_high_target: bool,
                                           raw_target_z_m: float) -> bool:
        if not is_nw_high_target or self._nw_high_target_base_y_nudge_m <= 0.0:
            return True
        self.get_logger().warn(
            "NW_HIGH_TARGET_BASE_Y_NUDGE: BASE +Y "
            f"{self._nw_high_target_base_y_nudge_m*1000:.0f}mm before close "
            "(pure depth correction after height alignment)")
        self.runtime_log.log(
            "nw_high_target_base_y_nudge",
            target_z_m=float(raw_target_z_m),
            base_y_nudge_m=self._nw_high_target_base_y_nudge_m,
        )
        if not self.execute_base_relative_line(
                [0.0, self._nw_high_target_base_y_nudge_m, 0.0],
                "NW_HIGH_TARGET_BASE_Y_NUDGE",
                CRANE_DESCENT_VEL_MM_S,
                FINAL_APPROACH_ACC_MM_S2):
            self.get_logger().error(
                "ABORT: NW high target BASE +Y nudge failed")
            self._abort_pick_with_complete()
            return False
        return True

    def _handle_gripper_close_failed(self, final_approach_distance: float,
                                     extra_advance_m: float,
                                     tool_finish_executed_m: float,
                                     tool_finish_executed_dir,
                                     used_approach_dir):
        self.get_logger().error(
            "ABORT: gripper close failed twice — skip detach and retreat straight")
        self.runtime_log.log(
            "pick_sequence_stopped",
            result_code="GRIPPER_CLOSE_FAILED",
            action="straight_retreat_without_detach",
        )
        retreat_distance_m = (
            final_approach_distance + extra_advance_m - tool_finish_executed_m)
        retreat_ok = self._execute_retreat_steps(
            build_straight_retreat_steps(
                self._measured_tcp_model,
                retreat_distance_m,
                used_approach_dir,
                tool_finish_executed_m,
                tool_finish_executed_dir,
                "CLOSE_FAIL_RETREAT_BASE",
                "CLOSE_FAIL_RETREAT",
            )
        )
        self._clear_neighbor_obstacles()
        if retreat_ok:
            self._reset_gripper()
            self.pick_complete_pub.publish(Empty())
        else:
            self._hold_pick_sequence("gripper_close_failed_retreat_failed")

    def _execute_detach_and_retreat(self, final_approach_distance: float,
                                    extra_advance_m: float,
                                    tool_finish_executed_m: float,
                                    tool_finish_executed_dir,
                                    used_approach_dir,
                                    grasp_joints):
        self.get_logger().info(
            f"4 detach pull — BASE -Z {DETACH_PULL_DOWN_MM:.0f}mm "
            f"at {DETACH_PULL_VEL_MM_S:.0f}mm/s")
        self._execute_pitch_detach()  # 실패해도 retreat은 항상 실행

        # 실측 TCP 모델은 cuRobo가 pre-approach까지만 계획하므로 최종 MoveLine
        # 전체를 역진한다. Legacy 모델은 기존 검증 baseline대로 extra advance만
        # 역진하고 이후 joint-space 복귀를 사용한다.
        reverse_distance_m = extra_advance_m
        if self._measured_tcp_model:
            reverse_distance_m += final_approach_distance - tool_finish_executed_m
        reverse_ok = self._execute_retreat_steps(
            build_straight_retreat_steps(
                self._measured_tcp_model,
                reverse_distance_m,
                used_approach_dir,
                tool_finish_executed_m,
                tool_finish_executed_dir,
                "RETREAT_BASE",
                "RETREAT",
            )
        )
        if not reverse_ok:
            self.get_logger().error(
                "ABORT: straight reverse retreat failed — holding current pose")
            self._clear_neighbor_obstacles()
            self._hold_pick_sequence("straight_reverse_retreat_failed")
            return None

        time.sleep(STRAIGHT_RETREAT_SETTLE_SEC)
        return (
            list(self.current_joints)
            if self.current_joints is not None
            else grasp_joints
        )

    def _maybe_execute_place_after_retreat(self, grasp_result: str,
                                           retreat_joints):
        # Place 게이트 기본값은 fail-closed다. 센서 판독이 불가능한 실험에서만
        # allow_unverified_grasp_place를 명시적으로 켜고 사람 관찰 라벨을 남긴다.
        allow_place = allow_place_after_grasp(
            grasp_result,
            self._allow_unverified_grasp_place,
        )
        if not allow_place:
            place_block_reason = place_gate_block_reason(grasp_result)
            self.get_logger().warn(
                f"PLACE_GATE_BLOCKED ({grasp_result}): {place_block_reason}")
            self.runtime_log.log(
                "place_gate_blocked",
                grasp_result=grasp_result,
                reason=place_block_reason,
            )

        return_start_joints = retreat_joints
        if not self._enable_marker_place or not allow_place:
            return False, return_start_joints

        place_status, place_joints = self.tray_place_executor.execute_marker_place_after_retreat(
            retreat_joints)
        if place_status == "success":
            self._marker_place_slot_idx += 1
        outcome = classify_place_outcome(
            place_status,
            self._use_taught_slot0_place_reference,
            self._hold_after_taught_slot0_place,
            self._marker_place_slot_idx,
        )
        if outcome["action"] == "continue":
            return False, place_joints
        if (
            outcome["action"] == "hold"
            and outcome["result_code"] == "TAUGHT_TRAY_PLACE_COMPLETE_HOLD"
        ):
            completed_slot_index = outcome["completed_slot_index"]
            self._clear_neighbor_obstacles()
            self.runtime_log.log(
                "pick_sequence_stopped",
                result_code=outcome["result_code"],
                slot_index=completed_slot_index,
                current_joints_rad=self.current_joints,
            )
            self.get_logger().warn(
                f"TAUGHT_TRAY_SLOT{completed_slot_index}_PLACE_COMPLETE_HOLD: "
                "release complete; "
                "automatic next pick blocked until planner restart")
            self._hold_pick_sequence(outcome["hold_reason"])
            return True, return_start_joints
        if (
            outcome["action"] == "hold"
            and outcome["result_code"] == "TAUGHT_TRAY_FULL"
        ):
            self._clear_neighbor_obstacles()
            self.runtime_log.log(
                "pick_sequence_stopped",
                result_code=outcome["result_code"],
                current_joints_rad=self.current_joints,
            )
            self.get_logger().warn(
                "TAUGHT_TRAY_FULL: all 15 slots consumed; "
                "automatic next pick blocked until tray reset")
            self._hold_pick_sequence(outcome["hold_reason"])
            return True, return_start_joints
        if outcome["action"] == "skip":
            # tray 없음/stale — place 생략, scan 복귀
            self.get_logger().warn("PLACE_SKIPPED: tray unavailable; returning to scan")
            self.runtime_log.log("place_skipped", reason=outcome["reason"],
                                 grasp_result=grasp_result)
            return False, return_start_joints

        # 로봇이 이미 움직인 뒤 실패 or preview hold → latch
        self._clear_neighbor_obstacles()
        self.runtime_log.log(
            "pick_sequence_stopped",
            result_code=outcome["result_code"],
            place_status=place_status,
            current_joints_rad=self.current_joints,
        )
        self.get_logger().warn(
            f"PICK_SEQUENCE_HOLD place_status={place_status}; "
            "pick_complete not published, automatic scan paused")
        self._hold_pick_sequence(outcome["hold_reason"])
        return True, return_start_joints

    def _return_to_pick_start_and_complete(self, return_start_joints,
                                           pick_start_joints,
                                           grasp_result: str,
                                           detach_result: str) -> bool:
        self.get_logger().info("7 return to pick-start scan pose")
        # 직선 retreat 또는 marker place 완료 후 이번 pick이 시작된 scan pose로
        # 복귀한다. scan_executor는 같은 SW 셀의 다음 target을 이어서 전달한다.
        pick_start_joints_deg = np.rad2deg(pick_start_joints).tolist()
        pick_start_joints_deg = self._nearest_equivalent_joints(pick_start_joints_deg)
        ok, _ = self.plan_to_fixed_joints_pose(
            return_start_joints,
            pick_start_joints_deg,
            "pick-start scan pose after pick/place",
            skip_swing_check=True,
        )
        if not ok:
            self.get_logger().warn(
                "pick-start scan pose after pick/place failed; holding current pose")
            self._clear_neighbor_obstacles()
            self.runtime_log.log(
                "pick_sequence_stopped",
                result_code="RETURN_TO_SCAN_FAILED",
                current_joints_rad=self.current_joints,
            )
            self._hold_pick_sequence("return_to_scan_failed")
            return False

        self._clear_neighbor_obstacles()
        self._reset_gripper()  # 다음 파지를 위해 approach 위치(600)로 복귀
        self.pick_complete_pub.publish(Empty())
        sequence_result_code = pick_sequence_result_code(grasp_result)
        self.runtime_log.log(
            "pick_sequence_complete",
            result_code=sequence_result_code,
            grasp_result=grasp_result,
            detach_result=detach_result,
            return_pose="pick_start_scan_pose",
            marker_place_enabled=self._enable_marker_place,
            marker_place_release_executed=(
                self._enable_marker_place and self._execute_marker_place_release),
            current_joints_rad=self.current_joints,
        )
        self.get_logger().info(f"=== PICK COMPLETE ({sequence_result_code}) ===")
        return True

    def _execute_leftmost_extra_advance_if_needed(self, raw_x_m: float,
                                                  used_grasp_offset: float,
                                                  used_approach_dir,
                                                  used_grasp_ee_pos):
        extra_advance_m = 0.0
        if raw_x_m > 0.25 or self._leftmost_extra_advance_request_m <= 0.0:
            return True, extra_advance_m, used_grasp_ee_pos

        available_extra_m = max(
            0.0, used_grasp_offset - self._leftmost_wall_safety_margin_m)
        extra_advance_m = (
            self._leftmost_extra_advance_request_m
            if self._leftmost_allow_wall_model_override
            else min(self._leftmost_extra_advance_request_m, available_extra_m)
        )
        if self._leftmost_allow_wall_model_override:
            modeled_overtravel_m = max(0.0, extra_advance_m - available_extra_m)
            self.get_logger().error(
                f"LEFTMOST_WALL_MODEL_OVERRIDE: executing "
                f"{extra_advance_m*1000:.0f}mm extra advance; "
                f"modeled wall overtravel={modeled_overtravel_m*1000:.0f}mm. "
                f"Physical clearance and E-stop must be verified.")
            self.runtime_log.log(
                "leftmost_wall_model_override",
                requested_m=self._leftmost_extra_advance_request_m,
                executed_m=extra_advance_m,
                safe_available_m=available_extra_m,
                modeled_wall_overtravel_m=modeled_overtravel_m,
                reason="explicit_ros_parameter_override",
            )
        if extra_advance_m < 0.020:
            self.get_logger().warn(
                f"LEFTMOST_EXTRA_ADVANCE_BLOCKED: request="
                f"{self._leftmost_extra_advance_request_m*1000:.0f}mm, "
                f"safe_available={available_extra_m*1000:.0f}mm, "
                f"wall_margin={self._leftmost_wall_safety_margin_m*1000:.0f}mm")
            self.runtime_log.log(
                "leftmost_extra_advance_blocked",
                requested_m=self._leftmost_extra_advance_request_m,
                safe_available_m=available_extra_m,
                wall_safety_margin_m=self._leftmost_wall_safety_margin_m,
                selected_grasp_offset_m=used_grasp_offset,
            )
            return True, 0.0, used_grasp_ee_pos

        if extra_advance_m < self._leftmost_extra_advance_request_m:
            self.get_logger().warn(
                f"LEFTMOST_EXTRA_ADVANCE_CAPPED: request="
                f"{self._leftmost_extra_advance_request_m*1000:.0f}mm -> "
                f"execute={extra_advance_m*1000:.0f}mm "
                f"(wall margin {self._leftmost_wall_safety_margin_m*1000:.0f}mm)")
        self.runtime_log.log(
            "leftmost_extra_advance",
            requested_m=self._leftmost_extra_advance_request_m,
            executed_m=extra_advance_m,
            wall_safety_margin_m=self._leftmost_wall_safety_margin_m,
            selected_grasp_offset_m=used_grasp_offset,
            validation="wall_distance_gate_only_not_curobo_endpoint",
        )
        if not self.execute_tool_z_line(
            extra_advance_m,
            motion_label="LEFTMOST_EXTRA_ADVANCE",
            vel_mm_s=LEFTMOST_EXTRA_ADVANCE_VEL_MM_S,
            acc_mm_s2=FINAL_APPROACH_ACC_MM_S2,
        ):
            self.get_logger().error(
                "ABORT: leftmost extra advance failed after dispatch — "
                "holding current pose")
            self._clear_neighbor_obstacles()
            self._hold_pick_sequence("leftmost_extra_advance_failed")
            return False, extra_advance_m, used_grasp_ee_pos
        used_grasp_ee_pos = (
            used_grasp_ee_pos + extra_advance_m * used_approach_dir)
        return True, extra_advance_m, used_grasp_ee_pos

    def _execute_final_approach_tool_finish(self, remaining_tool_line_m: float,
                                            curobo_depth_m: float,
                                            requested_total_m: float,
                                            used_grasp_variant,
                                            used_approach_dir,
                                            failure_context: str):
        self.get_logger().warn(
            "FINAL_APPROACH_TOOL_FINISH: cuRobo reached "
            f"{curobo_depth_m*1000:.0f}mm only; executing "
            f"remaining {remaining_tool_line_m*1000:.0f}mm with "
            "TOOL +Z MoveLine like the proven SW baseline")
        self.runtime_log.log(
            "final_approach_tool_finish_requested",
            reason="curobo_deep_final_approach_ik_fail",
            curobo_depth_m=curobo_depth_m,
            tool_finish_m=remaining_tool_line_m,
            requested_total_m=requested_total_m,
        )
        finish_direction = tool_finish_base_direction(
            used_grasp_variant,
            used_approach_dir,
        )
        if finish_direction.use_base_line:
            if finish_direction.is_published_roll:
                self.get_logger().warn(
                    "FINAL_APPROACH_TOOL_FINISH_BASE_FOR_PUBLISHED_ROLL: "
                    "using BASE relative line; TOOL +Z returned "
                    "success/no-motion in this branch")
            tool_finish_ok = self.execute_base_relative_line(
                remaining_tool_line_m * finish_direction.direction,
                "FINAL_APPROACH_TOOL_FINISH",
                vel_mm_s=FINAL_APPROACH_VEL_MM_S,
                acc_mm_s2=FINAL_APPROACH_ACC_MM_S2,
            )
            tool_finish_delta = remaining_tool_line_m * finish_direction.direction
            tool_finish_dir = finish_direction.direction if tool_finish_ok else None
        else:
            tool_finish_ok = self.execute_tool_z_line(
                remaining_tool_line_m,
                motion_label="FINAL_APPROACH_TOOL_FINISH",
                vel_mm_s=FINAL_APPROACH_VEL_MM_S,
                acc_mm_s2=FINAL_APPROACH_ACC_MM_S2,
                min_distance_m=0.005,
            )
            tool_finish_delta = remaining_tool_line_m * np.array(
                used_approach_dir, dtype=float)
            tool_finish_dir = None

        if not tool_finish_ok:
            self.get_logger().error(
                "FINAL_APPROACH_TOOL_FINISH failed after "
                f"{failure_context}")
        return tool_finish_ok, tool_finish_delta, remaining_tool_line_m, tool_finish_dir

    def _compute_final_approach_distance(self, raw_straw, straw,
                                         used_pre_ee_pos,
                                         used_approach_dir,
                                         used_grasp_offset: float):
        if not self._measured_tcp_model:
            return PRE_APPROACH_OFFSET - used_grasp_offset

        approach_distance = measured_tcp_approach_distance(
            raw_y_m=raw_straw[1],
            straw=straw,
            used_pre_ee_pos=used_pre_ee_pos,
            used_approach_dir=used_approach_dir,
            max_approach_m=self._measured_tcp_max_approach_m,
            y_detection_bias_m=Y_DETECTION_BIAS_M,
            pre_approach_offset_m=PRE_APPROACH_OFFSET,
            final_standoff_m=MEASURED_TCP_FINAL_STANDOFF_M,
            wall_surface_y_m=WALL_SURFACE_Y_M,
        )
        target_plane_dist = approach_distance.target_plane_dist_m
        uncapped_distance = approach_distance.uncapped_distance_m
        final_approach_distance = approach_distance.final_distance_m

        if (
            abs(float(used_approach_dir[2])) > 1e-3
            and self._nw_high_target_final_extra_m > 0.0
        ):
            before_extra_m = final_approach_distance
            final_approach_distance = min(
                MEASURED_TCP_MAX_APPROACH_CEILING_M,
                final_approach_distance + self._nw_high_target_final_extra_m,
            )
            self.get_logger().warn(
                "NW_HIGH_TARGET_FINAL_EXTRA: "
                f"{before_extra_m*1000:.0f}mm -> "
                f"{final_approach_distance*1000:.0f}mm "
                f"(target_z={raw_straw[2]*1000:.0f}mm, "
                "observed shallow by ~30mm)")
            self.runtime_log.log(
                "nw_high_target_final_extra",
                target_z_m=float(raw_straw[2]),
                before_m=before_extra_m,
                after_m=final_approach_distance,
                requested_extra_m=self._nw_high_target_final_extra_m,
                ceiling_m=MEASURED_TCP_MAX_APPROACH_CEILING_M,
            )
        if final_approach_distance + 1e-6 < uncapped_distance:
            self.get_logger().warn(
                "MEASURED_TCP_APPROACH_CAPPED: requested "
                f"{uncapped_distance*1000:.0f}mm -> "
                f"{final_approach_distance*1000:.0f}mm "
                f"(target_plane={target_plane_dist*1000:.0f}mm, "
                "NW measured-TCP experimental depth cap)")
            self.runtime_log.log(
                "measured_tcp_approach_capped",
                requested_distance_m=uncapped_distance,
                capped_distance_m=final_approach_distance,
                target_plane_distance_m=target_plane_dist,
                max_approach_m=self._measured_tcp_max_approach_m,
                reason="nw_measured_tcp_experimental_depth_cap",
            )
        return final_approach_distance

    def _try_precomputed_final_approach(self, final_state: FinalApproachState,
                                        ret_grasp,
                                        measured_best_depth_m: float,
                                        requested_distance_m: float,
                                        used_pre_ee_pos,
                                        used_grasp_variant,
                                        used_approach_dir):
        if (
            not self._measured_tcp_model
            or not self._direct_curobo_final_approach_for_measured_tcp
        ):
            return False, False

        selected_curobo_depth_m = measured_best_depth_m
        approach_ok = (
            ret_grasp is not None
            and selected_curobo_depth_m > 0.0
            and self.execute_spline(*ret_grasp)
        )
        self.runtime_log.log(
            "final_approach_precomputed_curobo",
            controller="curobo_plus_doosan_move_spline_joint",
            requested_distance_m=requested_distance_m,
            executed_depth_m=selected_curobo_depth_m,
            success=approach_ok,
            approach_dir=used_approach_dir,
        )
        if approach_ok:
            self.get_logger().info(
                "FINAL_APPROACH_PRECOMPUTED_CUROBO "
                f"depth={selected_curobo_depth_m*1000:.0f}mm "
                "(reusing probe plan; no extra IK fallback search)")
            final_state.distance_m = selected_curobo_depth_m
            final_state.grasp_ee_pos = (
                used_pre_ee_pos + final_state.distance_m * used_approach_dir)
            remaining_tool_line_m = requested_distance_m - selected_curobo_depth_m
            if (
                self._measured_tcp_tool_line_after_curobo_fallback
                and remaining_tool_line_m >= 0.020
            ):
                (
                    tool_finish_ok,
                    tool_finish_delta,
                    _finish_m,
                    tool_finish_dir,
                ) = self._execute_final_approach_tool_finish(
                    remaining_tool_line_m,
                    selected_curobo_depth_m,
                    requested_distance_m,
                    used_grasp_variant,
                    used_approach_dir,
                    "precomputed cuRobo final approach",
                )
                if not tool_finish_ok:
                    approach_ok = False
                else:
                    final_state.apply_tool_finish(
                        selected_curobo_depth_m,
                        remaining_tool_line_m,
                        tool_finish_delta,
                        tool_finish_dir,
                    )
                    self.runtime_log.log(
                        "final_approach_tool_finish_success",
                        executed_total_m=final_state.distance_m,
                        horizontal_only=final_state.tool_finish_executed_dir is not None,
                    )
        else:
            self.get_logger().warn(
                "FINAL_APPROACH_PRECOMPUTED_CUROBO failed; "
                "falling back to depth search")
        return True, approach_ok

    def _try_final_approach_fallback(self, final_state: FinalApproachState,
                                     requested_final_approach_distance: float,
                                     used_pre_ee_pos,
                                     used_grasp_quat,
                                     used_grasp_variant,
                                     used_approach_dir,
                                     precomputed_final_attempted: bool) -> bool:
        fallback_ok = False
        if not (
            self._measured_tcp_model
            and ENABLE_CUROBO_FINAL_APPROACH_FALLBACK
            and not precomputed_final_attempted
            and used_pre_ee_pos is not None
            and used_grasp_quat is not None
            and self.current_joints is not None
        ):
            return fallback_ok

        depth_candidates = final_approach_fallback_depths(
            requested_final_approach_distance,
            self._direct_curobo_final_approach_for_measured_tcp,
        )
        for depth_m in depth_candidates:
            fallback_target = (
                used_pre_ee_pos
                + depth_m * used_approach_dir
            )
            self.get_logger().warn(
                "FINAL_APPROACH_STRAIGHT_BASE failed; trying cuRobo "
                f"fallback depth={depth_m*1000:.0f}mm "
                f"xyz={[round(v*1000, 1) for v in fallback_target]}mm")
            self.runtime_log.log(
                "final_approach_fallback_requested",
                reason="doosan_moveline_failed_or_no_motion",
                target_pos_m=fallback_target.tolist(),
                approach_dir=used_approach_dir,
                depth_m=depth_m,
            )
            fallback_plan = self.plan(
                self.current_joints,
                fallback_target.tolist(),
                used_grasp_quat,
                num_ik_seeds=24,
                max_attempts=2,
                timeout_sec=1.5,
                max_joint_delta_deg=90.0,
            )
            if fallback_plan is None:
                continue
            fallback_ok = self.execute_spline(*fallback_plan)
            if fallback_ok:
                final_state.distance_m = depth_m
                final_state.grasp_ee_pos = fallback_target.copy()
                self.runtime_log.log(
                    "final_approach_fallback_success",
                    controller="curobo_plus_doosan_move_spline_joint",
                    executed_depth_m=depth_m,
                )
                remaining_tool_line_m = (
                    requested_final_approach_distance - depth_m)
                if (
                    self._direct_curobo_final_approach_for_measured_tcp
                    and self._measured_tcp_tool_line_after_curobo_fallback
                    and remaining_tool_line_m >= 0.020
                ):
                    (
                        tool_finish_ok,
                        tool_finish_delta,
                        tool_finish_executed_m,
                        tool_finish_executed_dir,
                    ) = self._execute_final_approach_tool_finish(
                        remaining_tool_line_m,
                        depth_m,
                        requested_final_approach_distance,
                        used_grasp_variant,
                        used_approach_dir,
                        "cuRobo shallow fallback",
                    )
                    if not tool_finish_ok:
                        fallback_ok = False
                        break
                    final_state.apply_tool_finish(
                        depth_m,
                        remaining_tool_line_m,
                        tool_finish_delta,
                        tool_finish_executed_dir,
                    )
                    self.runtime_log.log(
                        "final_approach_tool_finish_success",
                        executed_total_m=final_state.distance_m,
                        horizontal_only=final_state.tool_finish_executed_dir is not None,
                    )
                break
        return fallback_ok

    def _execute_final_approach(self, final_state: FinalApproachState,
                                final_approach_distance: float,
                                ret_grasp,
                                measured_best_depth_m: float,
                                used_pre_ee_pos,
                                used_grasp_quat,
                                used_grasp_variant,
                                used_approach_dir) -> bool:
        if final_approach_distance <= 0.001:
            return True

        requested_final_approach_distance = final_approach_distance
        precomputed_final_attempted, approach_ok = (
            self._try_precomputed_final_approach(
                final_state,
                ret_grasp,
                measured_best_depth_m,
                requested_final_approach_distance,
                used_pre_ee_pos,
                used_grasp_variant,
                used_approach_dir,
            )
        )
        if not precomputed_final_attempted and self._measured_tcp_model:
            approach_ok = self.execute_base_relative_line(
                final_approach_distance * used_approach_dir,
                "FINAL_APPROACH_STRAIGHT_BASE",
                vel_mm_s=FINAL_APPROACH_VEL_MM_S,
                acc_mm_s2=FINAL_APPROACH_ACC_MM_S2,
            )
        elif not precomputed_final_attempted:
            approach_ok = self.execute_tool_z_line(
                final_approach_distance,
                min_distance_m=0.005)
        if not approach_ok:
            fallback_ok = self._try_final_approach_fallback(
                final_state,
                requested_final_approach_distance,
                used_pre_ee_pos,
                used_grasp_quat,
                used_grasp_variant,
                used_approach_dir,
                precomputed_final_attempted,
            )
            if not fallback_ok:
                self.get_logger().error("ABORT: 직선 진입 실패")
                self._abort_pick_with_complete()
                return False
        return True

    def _prepare_pick_target_or_abort(self, p):
        target_info = prepare_pick_target(
            p,
            self._measured_tcp_model,
            self._pick_target_x_bias_m,
            self._pick_target_z_bias_m,
            self._nw_high_target_z_threshold_m,
            self._nw_high_target_crane_z_offset_m,
        )
        detection_raw_y = target_info["detection_raw_y"]
        raw_y = target_info["raw_y"]
        wall_y_clamped = target_info["wall_y_clamped"]
        raw_straw = target_info["raw_straw"]
        straw = target_info["straw"]
        is_nw_high_target = target_info["is_nw_high_target"]
        crane_z_offset_m = target_info["crane_z_offset_m"]

        if wall_y_clamped:
            self.get_logger().warn(
                f"Detection Y={detection_raw_y*1000:.0f}mm > wall surface "
                f"{WALL_SURFACE_Y_M*1000:.0f}mm "
                f"(FK calibration drift) — clamped to {WALL_SURFACE_Y_M*1000:.0f}mm")
        if target_info["y_relax_applied"]:
            before_y = target_info["y_relax_before_m"]
            self.get_logger().warn(
                "NW_HIGH_TARGET_Y_PLANE_RELAX: clamped target Y "
                f"{before_y*1000:.0f}mm -> {straw[1]*1000:.0f}mm "
                "(limited stem-depth correction before planning)")
            self.runtime_log.log(
                "nw_high_target_y_plane_relax",
                raw_detection_y_m=detection_raw_y,
                clamped_wall_y_m=WALL_SURFACE_Y_M,
                before_target_y_m=before_y,
                after_target_y_m=float(straw[1]),
                relax_m=NW_HIGH_TARGET_Y_PLANE_RELAX_M,
            )

        x_min, x_max = target_info["x_range_m"]
        if not target_info["x_guard_ok"]:
            self.get_logger().warn(
                f"ABORT: pick target x={raw_straw[0]*1000:.0f}mm outside "
                f"[{x_min*1000:.0f}, {x_max*1000:.0f}]mm")
            self.pick_complete_pub.publish(Empty())
            return None
        if not target_info["z_guard_ok"]:
            self.get_logger().warn(
                f"SKIP: pick target z={raw_straw[2]*1000:.0f}mm > "
                f"{target_info['z_max_m']*1000:.0f}mm "
                "(NW high/leaf candidate guard)")
            self.runtime_log.log(
                "pick_target_skipped",
                reason="target_z_above_measured_tcp_guard",
                raw_target_m=raw_straw,
                z_max_m=target_info["z_max_m"],
                wall_y_clamped=wall_y_clamped,
            )
            self.pick_complete_pub.publish(Empty())
            return None
        return target_info

    def _search_grasp(self, straw, is_nw_high_target, input_quat_wxyz,
                       grasp_retry_offsets, crane_z_offset_m):
        # 2. Grasp (cuRobo 2-step): 6cm pre-approach → 직선 진입
        # 직전 측방 편차가 줄기 형상/검출점 영향인지 분리하기 위해 6cm를 재검증한다.
        grasp_quat_variants = grasp_quat_variants_for_target(
            self._measured_tcp_model,
            is_nw_high_target,
        )
        published_roll_variant = self._published_roll_grasp_variant(input_quat_wxyz)
        if published_roll_variant is not None:
            grasp_quat_variants = [published_roll_variant] + grasp_quat_variants

        n_offsets = len(grasp_retry_offsets)
        n_quats   = len(grasp_quat_variants)
        self.get_logger().info(
            f"2 grasp (CuRobo 2-step {PRE_APPROACH_OFFSET*100:.0f}cm pre) — "
            f"trying {n_offsets} offsets × {n_quats} quats "
            f"| target=({straw[0]*1000:.0f},{straw[1]*1000:.0f},{straw[2]*1000:.0f})mm "
            f"| start_J1={np.rad2deg(self.current_joints[0]):.1f}°")
        if is_nw_high_target:
            self.get_logger().warn(
                "NW_HIGH_TARGET_VARIANT_ORDER: "
                + ", ".join(variant_label(v) for v in grasp_quat_variants)
                + " (prefer flatter branch over +15deg side-drift)")
        grasp_search = GraspSearchResult()
        # 2026-06-20: FINAL_APPROACH_TOOL_FINISH가 틸트 variant에서 horiz_dir로
        # 꺾여 들어가면 전진 경로가 더 이상 단일 직선(used_approach_dir)이 아니다.
        # retreat 쪽이 그 꺾인 다리를 모르면 전체 거리를 한 방향으로만 되돌리려
        # 하면서 실제로 온 길을 못 따라가 관절을 무리하게 꺾는다(J2 한도 초과
        # 실기로 확인). 이 두 변수로 꺾인 다리만 따로 기록해서 retreat에서
        # 분리해 되돌린다. 틸트가 없으면(horiz_dir==used_approach_dir) 0/None
        # 그대로라 기존 단일 직선 retreat과 동일하게 동작한다(SW no-op).
        tool_finish_executed_m = 0.0
        tool_finish_executed_dir = None
        measured_best = None
        # 2026-06-18: depth probing picks the deepest reachable standoff, but
        # multiple grasp_quat_variants often reach the IDENTICAL depth with
        # wildly different elbow health (J3 from ~0deg/near-singular up to
        # ~60deg/healthy) — verified by replaying a real failing run offline
        # bit-for-bit (replay_plan_call_dump.py). The old `depth_m >
        # measured_best_depth_m` strict-greater comparison always kept the
        # FIRST variant tried on a tie, which was consistently the worst one.
        # Track J3 health as a tiebreaker so an equally-deep but healthier
        # elbow from a later variant can replace it.
        for quat_frame, axis, quat_deg in grasp_quat_variants:
            q_retry, approach_dir, ee_pre = grasp_variant_pose(
                (quat_frame, axis, quat_deg),
                straw,
                self._ee_to_tcp_offset_m,
                self._measured_tcp_model,
                crane_z_offset_m,
            )
            r_pre_for_variant = self.plan(
                self.current_joints, ee_pre.tolist(), q_retry, num_ik_seeds=24
            )
            if r_pre_for_variant is None:
                grasp_search.attempt_count += len(grasp_retry_offsets)
                continue
            pre_joints = r_pre_for_variant[0][-1].tolist()

            if self._measured_tcp_model:
                # Measured TCP에서 pre-approach만 보고 첫 자세를 확정하면,
                # NW처럼 자세가 빡빡한 영역에서 final 접근 IK가 계속 막힌다.
                # 실행 전 각 orientation의 final depth reachability를 probing해
                # 가장 깊게 들어갈 수 있는 자세를 고른다.
                measured_best, should_break_outer = (
                    self.grasp_search_executor.run_measured_tcp_depth_probe(
                        ee_pre,
                        approach_dir,
                        q_retry,
                        pre_joints,
                        r_pre_for_variant,
                        (quat_frame, axis, quat_deg),
                        grasp_search,
                        measured_best,
                        is_nw_high_target,
                    )
                )
                if should_break_outer:
                    break
                continue

            self.grasp_search_executor.try_legacy_grasp_offsets(
                grasp_retry_offsets,
                straw,
                approach_dir,
                q_retry,
                pre_joints,
                r_pre_for_variant,
                (quat_frame, axis, quat_deg),
                ee_pre,
                grasp_search,
            )
            if grasp_search.found:
                break

        if self._measured_tcp_model and measured_best is not None:
            grasp_search.apply_measured_best(measured_best)
            self.runtime_log.log(
                "measured_tcp_final_probe_selected",
                selected_depth_m=grasp_search.measured_best_depth_m,
                grasp_variant=grasp_search.grasp_variant,
                pre_ee_pos_m=grasp_search.pre_ee_pos.tolist(),
                final_ee_pos_m=grasp_search.grasp_ee_pos.tolist(),
            )
        return (
            grasp_search, n_offsets, n_quats,
            tool_finish_executed_m, tool_finish_executed_dir,
        )

    def _pick(self, msg: PoseStamped):
        p = msg.pose.position
        input_quat_wxyz = quat_normalize_wxyz([
            msg.pose.orientation.w,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
        ])
        # 같은 셀의 다음 target을 계속 처리할 수 있도록 이번 pick이 시작된
        # taught scan pose를 저장한다. overview 복귀는 scan_executor가 담당한다.
        pick_start_joints = list(self.current_joints)
        self.runtime_log.log(
            "pick_sequence_start",
            input_frame=msg.header.frame_id,
            input_target_m=[p.x, p.y, p.z],
            input_quat_xyzw=[
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ],
            input_quat_wxyz=input_quat_wxyz,
            start_joints_rad=pick_start_joints,
        )

        target_info = self._prepare_pick_target_or_abort(p)
        if target_info is None:
            return
        wall_y_clamped = target_info["wall_y_clamped"]
        raw_straw = target_info["raw_straw"]
        straw = target_info["straw"]
        is_nw_high_target = target_info["is_nw_high_target"]
        crane_z_offset_m = target_info["crane_z_offset_m"]

        grasp_retry_offsets = self.grasp_candidates_for_target(straw)

        self.get_logger().info(
            f"=== PICK 딸기 raw=({raw_straw[0]*1000:.0f},{raw_straw[1]*1000:.0f},{raw_straw[2]*1000:.0f})mm "
            f"grasp=({straw[0]*1000:.0f},{straw[1]*1000:.0f},{straw[2]*1000:.0f})mm "
            f"x_bias={self._pick_target_x_bias_m*1000:+.0f}mm "
            f"z_bias={self._pick_target_z_bias_m*1000:+.0f}mm ===")
        self.runtime_log.log(
            "pick_target_prepared",
            raw_target_m=raw_straw,
            grasp_target_m=straw,
            grasp_x_bias_m=self._pick_target_x_bias_m,
            grasp_z_bias_m=self._pick_target_z_bias_m,
            wall_y_clamped=wall_y_clamped,
            nw_high_target=is_nw_high_target,
        )

        # 접근 중 잎/과실을 집게로 미는 것을 줄이기 위해 수평 진입 전에
        # 파지 파츠를 600으로 명시적으로 열어 둔다.
        self.gripper_client.open_for_stem_descent()

        self._register_neighbor_obstacles(straw)
        self.motion_gen.detach_object_from_robot()

        if raw_straw[0] < -0.30 and not self._measured_tcp_model:
            straw[0] += LEFTMOST_GRASP_X_CORR_M

        grasp_search, n_offsets, n_quats, tool_finish_executed_m, tool_finish_executed_dir = (
            self._search_grasp(
                straw, is_nw_high_target, input_quat_wxyz,
                grasp_retry_offsets, crane_z_offset_m,
            )
        )

        ret_pre = grasp_search.ret_pre
        ret_grasp = grasp_search.ret_grasp
        used_grasp_offset = grasp_search.grasp_offset_m
        used_grasp_variant = grasp_search.grasp_variant
        used_approach_dir = grasp_search.approach_dir
        used_grasp_quat = grasp_search.grasp_quat
        used_pre_ee_pos = grasp_search.pre_ee_pos
        used_grasp_ee_pos = grasp_search.grasp_ee_pos
        measured_best_depth_m = grasp_search.measured_best_depth_m

        if ret_pre is not None and leftmost_depth_limited(raw_straw[0], used_grasp_offset):
            self.get_logger().warn(
                f"LEFTMOST_DEPTH_LIMITED: deeper 30/35/40/45mm endpoints rejected; "
                f"using {used_grasp_offset*1000:.0f}mm stand-off")
            self.runtime_log.log(
                "leftmost_depth_limited",
                selected_grasp_offset_m=used_grasp_offset,
                attempted_offsets_m=leftmost_rejected_offsets(used_grasp_offset),
                reason="deeper_endpoints_rejected",
            )

        if ret_pre is None:
            self.get_logger().error(
                f"ABORT: grasp 전체 실패 — {grasp_search.attempt_count}개 후보 모두 reject "
                f"(target=({straw[0]*1000:.0f},{straw[1]*1000:.0f},{straw[2]*1000:.0f})mm "
                f"start_J=[{', '.join(f'{np.rad2deg(v):.0f}' for v in self.current_joints)}]°)")
            self._abort_pick_with_complete()
            return

        if self._measured_tcp_model and self._measured_tcp_plan_only:
            self.get_logger().warn(
                "MEASURED_TCP_PLAN_ONLY: valid pre-approach found and guarded "
                f"{(PRE_APPROACH_OFFSET - used_grasp_offset)*1000:.0f}mm final "
                "MoveLine prepared; "
                "no robot motion dispatched. Set measured_tcp_plan_only:=false only "
                "after reviewing the target and keeping E-stop ready.")
            self.runtime_log.log(
                "measured_tcp_plan_only_hold",
                grasp_offset_m=used_grasp_offset,
                grasp_variant=used_grasp_variant,
                approach_dir=used_approach_dir,
                planned_pre_endpoint_rad=ret_pre[0][-1].tolist(),
                planned_grasp_endpoint_rad=ret_grasp[0][-1].tolist(),
                final_standoff_m=used_grasp_offset,
                guarded_final_move_line_m=PRE_APPROACH_OFFSET - used_grasp_offset,
                pick_complete_published=False,
            )
            self._clear_neighbor_obstacles()
            self.get_logger().warn(
                "MEASURED_TCP_PLAN_ONLY_HOLD: /pick_complete was not published, "
                "so the scan executor must not return home or advance automatically.")
            return

        # pre-approach 실행 후 직선 진입
        final_approach_distance = self._compute_final_approach_distance(
            raw_straw,
            straw,
            used_pre_ee_pos,
            used_approach_dir,
            used_grasp_offset,
        )
        final_state = FinalApproachState(
            final_approach_distance,
            used_grasp_ee_pos,
            tool_finish_executed_m,
            tool_finish_executed_dir,
        )
        if not self.execute_spline(*ret_pre):
            self.get_logger().error("ABORT: pre-approach spline 실패")
            self._abort_pick_with_complete()
            return
        self.get_logger().info(
            f"PRE_APPROACH_REACHED — settling {PRE_APPROACH_SETTLE_SEC:.1f}s "
            f"before {final_approach_distance*1000:.0f}mm straight approach")
        time.sleep(PRE_APPROACH_SETTLE_SEC)

        if not self._execute_final_approach(
            final_state,
            final_approach_distance,
            ret_grasp,
            measured_best_depth_m,
            used_pre_ee_pos,
            used_grasp_quat,
            used_grasp_variant,
            used_approach_dir,
        ):
            return

        # 실기 확인: 모든 벽면 딸기 줄기는 모델 벽 앞면보다 ~30mm 안쪽에 위치.
        # wall_margin=-30mm이면 available = offset+30mm → 80mm extra 자동 실행.
        # rightmost(x>250mm)는 offsets[-0.03, 0.0]으로 이미 깊게 진입하므로 제외.
        extra_ok, extra_advance_m, used_grasp_ee_pos = (
            self._execute_leftmost_extra_advance_if_needed(
                raw_straw[0],
                used_grasp_offset,
                used_approach_dir,
                final_state.grasp_ee_pos,
            )
        )
        final_approach_distance = final_state.distance_m
        tool_finish_executed_m = final_state.tool_finish_executed_m
        tool_finish_executed_dir = final_state.tool_finish_executed_dir
        if not extra_ok:
            return

        grasp_joints = (
            list(self.current_joints)
            if self.current_joints is not None
            else ret_grasp[0][-1].tolist()
        )
        self.get_logger().info(
            f"GRASP_POSE_REACHED — offset={used_grasp_offset:+.3f}m "
            f"pre={PRE_APPROACH_OFFSET*100:.0f}cm+{final_approach_distance*1000:.0f}mm+{extra_advance_m*1000:.0f}mm "
            f"variant={used_grasp_variant} elevation={np.degrees(np.arcsin(np.clip(used_approach_dir[2], -1.0, 1.0))):+.1f}deg "
            f"(attempt {grasp_search.attempt_count}/{n_offsets * n_quats})")
        self.runtime_log.log(
            "grasp_pose_reached",
            grasp_offset_m=used_grasp_offset,
            grasp_variant=used_grasp_variant,
            approach_dir=used_approach_dir,
            extra_advance_m=extra_advance_m,
            current_joints_rad=self.current_joints,
        )

        # 수평 진입 완료 후 열린 그리퍼로 줄기를 따라 KP1까지 하강한다.
        if not self._execute_open_stem_descent_if_needed(
                crane_z_offset_m, float(straw[2]), used_grasp_ee_pos, used_grasp_variant):
            return

        if not self._execute_nw_base_y_nudge_if_needed(
                is_nw_high_target, float(raw_straw[2])):
            return

        # 3. 그리퍼 닫기 + 파지 확인
        grasp_result, present_pos, present_current_raw, grasp_reason = (
            self._close_and_verify_grasp())
        if grasp_result == "GRIPPER_CLOSE_FAILED":
            self._handle_gripper_close_failed(
                final_approach_distance,
                extra_advance_m,
                tool_finish_executed_m,
                tool_finish_executed_dir,
                used_approach_dir,
            )
            return
        # 4. BASE -Z 당기기로 줄기 분리 후 직선 역진 retreat
        retreat_joints = self._execute_detach_and_retreat(
            final_approach_distance,
            extra_advance_m,
            tool_finish_executed_m,
            tool_finish_executed_dir,
            used_approach_dir,
            grasp_joints,
        )
        if retreat_joints is None:
            return

        # 4b. VERIFY_DETACH
        detach_result = "DETACH_UNVERIFIED"
        self.runtime_log.log(
            "verify_detach",
            result_code=detach_result,
            grasp_result=grasp_result,
            retreat_policy="pitch_detach_then_straight_reverse",
            reason="no sensor; pitch detach executed",
        )

        place_handled, return_start_joints = self._maybe_execute_place_after_retreat(
            grasp_result,
            retreat_joints,
        )
        if place_handled:
            return

        self._return_to_pick_start_and_complete(
            return_start_joints,
            pick_start_joints,
            grasp_result,
            detach_result,
        )


def main():
    rclpy.init()
    node = CuroboPlanner()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
