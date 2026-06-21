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
from approach_retreat_policy import FinalApproachState
from curobo_planning_adapter import CuroboPlanningAdapter
from doosan_motion_client import DoosanMotionClient
from final_approach_executor import FinalApproachExecutor
from grasp_candidate_policy import (
    grasp_offsets_for_target,
    grasp_quat_variants_for_target,
)
from grasp_search_executor import GraspSearchExecutor
from gripper_client import HarvestGripperClient
from harvest_grasp_orientation import published_roll_grasp_candidate
from harvest_math import quat_rotate_vec
from harvest_motion_params import *  # noqa: F403 - experiment constants
from pick_sequence_executor import PickSequenceExecutor
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

    def _log_startup_banner(self) -> None:
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
        self.final_approach_executor = FinalApproachExecutor(
            node=self,
            runtime_log=self.runtime_log,
            plan_fn=self.plan,
            execute_spline_fn=self.execute_spline,
            execute_base_relative_line_fn=self.execute_base_relative_line,
            execute_tool_z_line_fn=self.execute_tool_z_line,
            current_joints_getter=lambda: self.current_joints,
            measured_tcp_model=self._measured_tcp_model,
            flat_grasp_only=self._flat_grasp_only,
            flat_grasp_target_plane_margin_m=self._flat_grasp_target_plane_margin_m,
            direct_curobo_final_approach_for_measured_tcp=(
                self._direct_curobo_final_approach_for_measured_tcp),
            measured_tcp_tool_line_after_curobo_fallback=(
                self._measured_tcp_tool_line_after_curobo_fallback),
            nw_high_target_final_extra_m=self._nw_high_target_final_extra_m,
            measured_tcp_max_approach_m=self._measured_tcp_max_approach_m,
        )
        self.pick_sequence_executor = PickSequenceExecutor(
            node=self,
            runtime_log=self.runtime_log,
            gripper_client=self.gripper_client,
            motion_gen=self.motion_gen,
            grasp_search_executor=self.grasp_search_executor,
            tray_place_executor=self.tray_place_executor,
            plan_fn=self.plan,
            execute_spline_fn=self.execute_spline,
            execute_base_z_relative_fn=self.execute_base_z_relative,
            execute_base_relative_line_fn=self.execute_base_relative_line,
            execute_tool_z_line_fn=self.execute_tool_z_line,
            execute_pitch_detach_fn=self._execute_pitch_detach,
            execute_retreat_steps_fn=self._execute_retreat_steps,
            plan_to_fixed_joints_pose_fn=self.plan_to_fixed_joints_pose,
            nearest_equivalent_joints_fn=self._nearest_equivalent_joints,
            grasp_candidates_for_target_fn=self.grasp_candidates_for_target,
            published_roll_grasp_variant_fn=self._published_roll_grasp_variant,
            close_and_verify_grasp_fn=self._close_and_verify_grasp,
            compute_final_approach_distance_fn=self._compute_final_approach_distance,
            execute_final_approach_fn=self._execute_final_approach,
            register_neighbor_obstacles_fn=self._register_neighbor_obstacles,
            clear_neighbor_obstacles_fn=self._clear_neighbor_obstacles,
            reset_gripper_fn=self._reset_gripper,
            abort_pick_with_complete_fn=self._abort_pick_with_complete,
            publish_pick_complete_fn=lambda: self.pick_complete_pub.publish(Empty()),
            hold_pick_sequence_fn=self._hold_pick_sequence,
            current_joints_getter=lambda: self.current_joints,
            marker_place_slot_idx_getter=lambda: self._marker_place_slot_idx,
            increment_marker_place_slot_idx_fn=self._increment_marker_place_slot_idx,
            measured_tcp_model=self._measured_tcp_model,
            measured_tcp_plan_only=self._measured_tcp_plan_only,
            flat_grasp_only=self._flat_grasp_only,
            ee_to_tcp_offset_m=self._ee_to_tcp_offset_m,
            pick_target_x_bias_m=self._pick_target_x_bias_m,
            pick_target_z_bias_m=self._pick_target_z_bias_m,
            nw_high_target_z_threshold_m=self._nw_high_target_z_threshold_m,
            nw_high_target_crane_z_offset_m=self._nw_high_target_crane_z_offset_m,
            nw_high_target_descent_extra_below_kp1_m=(
                self._nw_high_target_descent_extra_below_kp1_m),
            nw_high_target_base_y_nudge_m=self._nw_high_target_base_y_nudge_m,
            leftmost_extra_advance_request_m=self._leftmost_extra_advance_request_m,
            leftmost_wall_safety_margin_m=self._leftmost_wall_safety_margin_m,
            leftmost_allow_wall_model_override=self._leftmost_allow_wall_model_override,
            allow_unverified_grasp_place=self._allow_unverified_grasp_place,
            enable_marker_place=self._enable_marker_place,
            execute_marker_place_release=self._execute_marker_place_release,
            use_taught_slot0_place_reference=self._use_taught_slot0_place_reference,
            hold_after_taught_slot0_place=self._hold_after_taught_slot0_place,
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

        self._log_startup_banner()

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

    def _increment_marker_place_slot_idx(self):
        self._marker_place_slot_idx += 1

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
            self.pick_sequence_executor.run(msg)
        finally:
            self._pick_busy = False

    def _compute_final_approach_distance(self, raw_straw, straw,
                                         used_pre_ee_pos,
                                         used_approach_dir,
                                         used_grasp_offset: float,
                                         is_nw_high_target: bool,
                                         wall_y_clamped: bool = False):
        return self.final_approach_executor.compute_distance(
            raw_straw, straw, used_pre_ee_pos, used_approach_dir,
            used_grasp_offset, is_nw_high_target, wall_y_clamped,
        )

    def _execute_final_approach(self, final_state: FinalApproachState,
                                final_approach_distance: float,
                                ret_grasp,
                                measured_best_depth_m: float,
                                used_pre_ee_pos,
                                used_grasp_quat,
                                used_grasp_variant,
                                used_approach_dir) -> bool:
        ok = self.final_approach_executor.execute(
            final_state, final_approach_distance, ret_grasp,
            measured_best_depth_m, used_pre_ee_pos, used_grasp_quat,
            used_grasp_variant, used_approach_dir,
        )
        if not ok:
            self.get_logger().error("ABORT: 직선 진입 실패")
            self._abort_pick_with_complete()
        return ok


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
