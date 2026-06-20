#!/usr/bin/env python3
"""cuRobo Motion Planner Node for Doosan E0509

Pick sequence: pre-approach(CuRobo) → straight grasp(MoveLine) → close
               → straight reverse retreat(MoveLine) → pick-start scan pose → pick_complete
"""

import os
import time
import torch
import numpy as np
import json
import yaml
import glob
from scipy.spatial.transform import Rotation as SciR

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

from curobo.types.base import TensorDeviceType
from curobo.types.robot import JointState as CuroboJointState, RobotConfig
from curobo.types.math import Pose
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig
from curobo.geom.types import WorldConfig, Cuboid, Sphere
from harvest_grasp_orientation import published_roll_grasp_candidate
from harvest_math import (
    quat_from_axis_angle,
    quat_multiply_wxyz,
    quat_normalize_wxyz,
    quat_rotate_vec,
)
from harvest_motion_params import *  # noqa: F403 - experiment constants
from runtime_jsonl_logger import RuntimeJsonlLogger


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
        self.static_cuboids = load_environment_cuboids()
        self.dynamic_cuboids = []
        self.neighbor_spheres: list = []
        self._registered_neighbor_positions: list[np.ndarray] = []
        self._scene_positions: list = []

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
        config_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "config", "curobo"
        )
        if not os.path.exists(config_dir):
            from ament_index_python.packages import get_package_share_directory
            config_dir = os.path.join(
                get_package_share_directory("e0509_gripper_description"),
                "config", "curobo"
            )

        tensor_args = TensorDeviceType(device=torch.device("cuda:0"))
        robot_config_name = (
            "e0509_gripper_measured_tcp.yml"
            if self._measured_tcp_model
            else "e0509_gripper.yml"
        )
        with open(os.path.join(config_dir, robot_config_name), "r", encoding="utf-8") as f:
            robot_cfg_data = yaml.safe_load(f)
        robot_kin = robot_cfg_data["robot_cfg"]["kinematics"]
        robot_kin["urdf_path"] = os.path.join(config_dir, "e0509_gripper.urdf")
        robot_kin["collision_spheres"] = os.path.join(config_dir, "e0509_spheres.yml")
        robot_cfg = RobotConfig.from_dict(robot_cfg_data, tensor_args=tensor_args)
        world_cfg = WorldConfig(cuboid=self.static_cuboids)
        motion_gen_cfg = MotionGenConfig.load_from_robot_config(
            robot_cfg, world_cfg, tensor_args=tensor_args,
            num_trajopt_seeds=16, num_graph_seeds=16,
            collision_cache={"obb": 30, "mesh": 10, "sphere": 30},
            use_cuda_graph=False,
            self_collision_check=USE_CUROBO_SELF_COLLISION,
            self_collision_opt=USE_CUROBO_SELF_COLLISION,
        )
        self.motion_gen = MotionGen(motion_gen_cfg)
        self.motion_gen.warmup(warmup_js_trajopt=False)
        self.motion_gen.detach_object_from_robot()
        self.get_logger().info("cuRobo MotionGen warmed up!")

        self.declare_parameter("enable_marker_place_sequence", False)
        self.declare_parameter("execute_marker_place_release", False)
        self.declare_parameter("use_taught_slot0_place_reference", False)
        self.declare_parameter("hold_after_taught_slot0_place", True)
        self.declare_parameter("initial_place_slot_index", 0)
        self.declare_parameter("allow_generated_tray_slot_release", False)
        self.declare_parameter("allow_unverified_grasp_place", False)
        self.declare_parameter("grasp_current_contact_threshold_raw", -1)
        self.declare_parameter("tray_cells_json", "")
        self.declare_parameter("marker_place_max_age_sec", 3600.0)
        self.declare_parameter("marker_place_above_clearance_m", 0.100)
        self.declare_parameter(
            "taught_slot_above_clearance_m", TAUGHT_SLOT0_ABOVE_CLEARANCE_M)
        self.declare_parameter("row2_place_pitch_tilt_deg", 15.0)
        self.declare_parameter("row2_release_correction_mm", [0.0, 0.0, 0.0])
        self.declare_parameter("row2_max_line_deviation_mm", 20.0)
        self.declare_parameter("use_safe_grasp_action", True)
        self.declare_parameter("safe_grasp_max_current", 400)
        self.declare_parameter("safe_grasp_current_delta_threshold", 120)
        self.declare_parameter("safe_grasp_timeout_sec", 8.0)
        self.declare_parameter(
            "direct_curobo_final_approach_for_measured_tcp",
            DIRECT_CUROBO_FINAL_APPROACH_FOR_MEASURED_TCP)
        self.declare_parameter(
            "measured_tcp_max_approach_m",
            MEASURED_TCP_MAX_APPROACH_M)
        self.declare_parameter(
            "measured_tcp_tool_line_after_curobo_fallback",
            True)
        self.declare_parameter("use_published_grasp_orientation", False)
        self.declare_parameter("published_grasp_roll_align_axis", "x")
        self.declare_parameter("published_grasp_roll_max_abs_deg", 75.0)
        self.declare_parameter("pick_target_x_bias_m", 0.0)
        self.declare_parameter("pick_target_z_bias_m", GRASP_Z_BIAS)
        self.declare_parameter(
            "nw_high_target_z_threshold_m", NW_HIGH_TARGET_Z_THRESHOLD_M)
        self.declare_parameter(
            "nw_high_target_final_extra_m", NW_HIGH_TARGET_FINAL_EXTRA_M)
        self.declare_parameter(
            "nw_high_target_base_y_nudge_m", NW_HIGH_TARGET_BASE_Y_NUDGE_M)
        self.declare_parameter(
            "nw_high_target_crane_z_offset_m", NW_HIGH_TARGET_CRANE_Z_OFFSET_M)
        self.declare_parameter(
            "nw_high_target_descent_extra_below_kp1_m",
            NW_HIGH_TARGET_DESCENT_EXTRA_BELOW_KP1_M)
        self.declare_parameter("debug_dump_plan_calls", False)
        self._debug_dump_plan_calls = bool(
            self.get_parameter("debug_dump_plan_calls").value)
        self.declare_parameter(
            "leftmost_extra_advance_request_m",
            0.0 if self._measured_tcp_model else LEFTMOST_EXTRA_ADVANCE_REQUEST_M)
        self.declare_parameter(
            "leftmost_wall_safety_margin_m", LEFTMOST_WALL_SAFETY_MARGIN_M)
        self.declare_parameter("leftmost_allow_wall_model_override", False)
        self._enable_marker_place = bool(
            self.get_parameter("enable_marker_place_sequence").value)
        self._execute_marker_place_release = bool(
            self.get_parameter("execute_marker_place_release").value)
        self._use_taught_slot0_place_reference = bool(
            self.get_parameter("use_taught_slot0_place_reference").value)
        self._hold_after_taught_slot0_place = bool(
            self.get_parameter("hold_after_taught_slot0_place").value)
        self._marker_place_slot_idx = int(
            self.get_parameter("initial_place_slot_index").value)
        self._allow_generated_tray_slot_release = bool(
            self.get_parameter("allow_generated_tray_slot_release").value)
        if not 0 <= self._marker_place_slot_idx < TAUGHT_TRAY_SLOT_COUNT:
            raise ValueError(
                f"initial_place_slot_index must be 0..{TAUGHT_TRAY_SLOT_COUNT - 1}")
        self._allow_unverified_grasp_place = bool(
            self.get_parameter("allow_unverified_grasp_place").value)
        self._grasp_current_contact_threshold_raw = int(
            self.get_parameter("grasp_current_contact_threshold_raw").value)
        self._tray_cells_json = os.path.expanduser(
            str(self.get_parameter("tray_cells_json").value))
        self._marker_place_max_age_sec = float(
            self.get_parameter("marker_place_max_age_sec").value)
        self._marker_place_above_clearance_m = float(
            self.get_parameter("marker_place_above_clearance_m").value)
        self._taught_slot_above_clearance_m = float(
            self.get_parameter("taught_slot_above_clearance_m").value)
        self._row2_place_pitch_tilt_deg = float(
            self.get_parameter("row2_place_pitch_tilt_deg").value)
        self._row2_release_correction_mm = list(
            self.get_parameter("row2_release_correction_mm").value)
        self._row2_max_line_deviation_mm = float(
            self.get_parameter("row2_max_line_deviation_mm").value)
        self._use_safe_grasp_action = bool(
            self.get_parameter("use_safe_grasp_action").value) and _SAFE_GRASP_AVAILABLE
        self._safe_grasp_max_current = int(
            self.get_parameter("safe_grasp_max_current").value)
        self._safe_grasp_current_delta_threshold = int(
            self.get_parameter("safe_grasp_current_delta_threshold").value)
        self._safe_grasp_timeout_sec = float(
            self.get_parameter("safe_grasp_timeout_sec").value)
        self._direct_curobo_final_approach_for_measured_tcp = bool(
            self.get_parameter(
                "direct_curobo_final_approach_for_measured_tcp").value)
        self._measured_tcp_max_approach_m = float(
            self.get_parameter("measured_tcp_max_approach_m").value)
        self._measured_tcp_tool_line_after_curobo_fallback = bool(
            self.get_parameter(
                "measured_tcp_tool_line_after_curobo_fallback").value)
        self._use_published_grasp_orientation = bool(
            self.get_parameter("use_published_grasp_orientation").value)
        self._published_grasp_roll_align_axis = str(
            self.get_parameter("published_grasp_roll_align_axis").value
        ).strip().lower()
        if self._published_grasp_roll_align_axis not in {"x", "y"}:
            raise ValueError("published_grasp_roll_align_axis must be 'x' or 'y'")
        self._published_grasp_roll_max_abs_deg = max(
            0.0, float(
                self.get_parameter("published_grasp_roll_max_abs_deg").value))
        self._pick_target_x_bias_m = float(
            self.get_parameter("pick_target_x_bias_m").value)
        self._pick_target_z_bias_m = float(
            self.get_parameter("pick_target_z_bias_m").value)
        self._nw_high_target_z_threshold_m = float(
            self.get_parameter("nw_high_target_z_threshold_m").value)
        self._nw_high_target_final_extra_m = max(
            0.0, float(self.get_parameter("nw_high_target_final_extra_m").value))
        self._nw_high_target_base_y_nudge_m = max(
            0.0, float(
                self.get_parameter("nw_high_target_base_y_nudge_m").value))
        self._nw_high_target_crane_z_offset_m = max(
            0.0, float(
                self.get_parameter("nw_high_target_crane_z_offset_m").value))
        self._nw_high_target_descent_extra_below_kp1_m = max(
            0.0, float(self.get_parameter(
                "nw_high_target_descent_extra_below_kp1_m").value))
        self._leftmost_extra_advance_request_m = max(
            0.0, float(self.get_parameter("leftmost_extra_advance_request_m").value))
        self._leftmost_wall_safety_margin_m = float(
            self.get_parameter("leftmost_wall_safety_margin_m").value)
        self._leftmost_allow_wall_model_override = bool(
            self.get_parameter("leftmost_allow_wall_model_override").value)

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
        self.cli_change_op_speed = self.create_client(
            ChangeOperationSpeed, "/dsr01/motion/change_operation_speed",
            callback_group=self.service_cb_group)

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

    def _init_gripper_once(self):
        if not self._gripper_init_done:
            self._gripper_init_done = True
            self._reset_gripper()

    def _reset_gripper(self):
        """파지 완료/실패 후 그리퍼를 approach 위치(GRIPPER_APPROACH_POS)로 복귀."""
        self._set_gripper_position(GRIPPER_APPROACH_POS, timeout_sec=5.0)

    def _set_gripper_position(self, position: int, timeout_sec: float = 5.0) -> bool:
        """SetPosition 서비스 호출 (blocking). 서비스 미연결 시 warn 후 False 반환."""
        if self.cli_set_position is None:
            self.get_logger().warn("GRIPPER: cli_set_position unavailable (virtual?)")
            return False
        if not self.cli_set_position.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("GRIPPER: /gripper_service/set_position not available")
            return False
        req = _SetPosition.Request()
        req.position = int(position)
        req.timeout_sec = float(timeout_sec)
        future = self.cli_set_position.call_async(req)
        t0 = time.time()
        while not future.done() and (time.time() - t0) < timeout_sec + 2.0:
            time.sleep(0.05)
        if not future.done():
            self.get_logger().warn(f"GRIPPER: set_position({position}) timed out")
            return False
        res = future.result()
        if res is None or not res.success:
            self.get_logger().warn(
                f"GRIPPER: set_position({position}) failed: {getattr(res, 'message', 'None')}")
            return False
        return True

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
            data = json.loads(msg.data)
            cuboids = []
            for obj in data:
                cuboids.append(Cuboid(
                    name=obj["name"],
                    pose=[*obj["pos"], 1, 0, 0, 0],
                    dims=obj.get("dims", [0.05, 0.05, 0.05])
                ))
            self.dynamic_cuboids = cuboids
            self.update_curobo_world("dynamic obstacles")
        except Exception as e:
            self.get_logger().error(f"obstacles_cb error: {e}")

    # ── World 관리 ─────────────────────────────────────────────────────────────

    def update_curobo_world(self, reason="manual"):
        cuboids = self.static_cuboids + self.dynamic_cuboids
        self.motion_gen.update_world(WorldConfig(cuboid=cuboids, sphere=self.neighbor_spheres))
        self.get_logger().info(
            f"World updated ({reason}): static={len(self.static_cuboids)} "
            f"dynamic={len(self.dynamic_cuboids)} "
            f"neighbor_spheres={len(self.neighbor_spheres)}")
        self.runtime_log.log(
            "collision_world_update",
            reason=reason,
            cuboids=[{"name": c.name, "pose": c.pose, "dims": c.dims} for c in cuboids],
            neighbor_spheres=[
                {"name": s.name, "pose": s.pose, "radius": s.radius}
                for s in self.neighbor_spheres
            ],
        )

    def _scene_cb(self, msg: Float64MultiArray) -> None:
        data = msg.data
        self._scene_positions = [
            np.array([data[i], data[i+1], data[i+2]])
            for i in range(0, len(data) - 2, 3)
        ]
        self.runtime_log.log("scene_positions_received", positions_m=self._scene_positions)

    def _scan_status_cb(self, msg: String) -> None:
        self.runtime_log.log("scan_status", text=msg.data)

    def _cell_state_cb(self, msg: String) -> None:
        self.runtime_log.log("cell_state", text=msg.data)

    def _register_neighbor_obstacles(self, target_pos: np.ndarray) -> None:
        spheres = []
        registered_positions = []
        for i, pos in enumerate(self._scene_positions):
            if np.linalg.norm(pos - target_pos) < 0.035:
                continue
            spheres.append(Sphere(
                name=f"neighbor_{i}",
                pose=[float(pos[0]), float(pos[1]), float(pos[2]), 1.0, 0.0, 0.0, 0.0],
                radius=NEIGHBOR_SPHERE_RADIUS_M,
            ))
            registered_positions.append(np.array(pos, dtype=float))
        self.neighbor_spheres = spheres
        self._registered_neighbor_positions = registered_positions
        self.update_curobo_world("neighbor obstacles registered")
        self.get_logger().info(f"Registered {len(spheres)} neighbor sphere obstacle(s)")

    def _clear_neighbor_obstacles(self) -> None:
        had_neighbors = bool(self.neighbor_spheres or self._registered_neighbor_positions)
        self.neighbor_spheres = []
        self._registered_neighbor_positions = []
        if had_neighbors:
            self.update_curobo_world("neighbor obstacles cleared")

    # ── 충돌 진단 ──────────────────────────────────────────────────────────────

    def _check_state_feasible_with_world(self, joints, cuboids):
        try:
            self.motion_gen.update_world(WorldConfig(cuboid=cuboids))
            state = CuroboJointState.from_position(
                position=torch.tensor([self._clamp_joints(joints)], device="cuda:0", dtype=torch.float32),
                joint_names=self.JOINT_NAMES,
            )
            valid, status = self.motion_gen.check_start_state(state)
            return bool(valid), status
        finally:
            try:
                self.motion_gen.rollout_fn.primitive_collision_constraint.enable_cost()
                self.motion_gen.rollout_fn.robot_self_collision_constraint.enable_cost()
            except Exception:
                pass

    def diagnose_start_world_collision(self, joints, label):
        if not DEBUG_START_COLLISION:
            return
        full_world = self.static_cuboids + self.dynamic_cuboids
        far_dummy = Cuboid(
            name="debug_far_dummy",
            pose=[10.0, 10.0, 10.0, 1.0, 0.0, 0.0, 0.0],
            dims=[0.01, 0.01, 0.01],
        )
        tests = [("empty_world", [far_dummy])]
        tests += [(f"static:{c.name}", [c]) for c in self.static_cuboids]
        tests += [(f"dynamic:{c.name}", [c]) for c in self.dynamic_cuboids]
        bad = []
        try:
            for name, cuboids in tests:
                feasible, status = self._check_state_feasible_with_world(joints, cuboids)
                self.get_logger().warn(
                    f"{label} collision diag {name}: "
                    f"{'OK' if feasible else 'COLLISION'} status={status}")
                if not feasible:
                    bad.append(f"{name}:{status}")
        except Exception as e:
            self.get_logger().warn(f"{label} collision diag failed: {e}")
        finally:
            self.motion_gen.update_world(WorldConfig(cuboid=full_world))
        if bad:
            self.get_logger().error(f"{label} start collision suspects: {bad}")
        else:
            self.get_logger().warn(f"{label} no single obstacle reproduced the collision")

    def diagnose_js_endpoint_collision(self, start_joints, target_joints, label):
        if not DEBUG_START_COLLISION:
            return
        self.get_logger().warn(f"{label} endpoint collision diagnostic")
        self.diagnose_start_world_collision(start_joints, f"{label} start")
        self.diagnose_start_world_collision(target_joints, f"{label} goal")

    # ── 유틸 ──────────────────────────────────────────────────────────────────

    def _clamp_joints(self, joints):
        return [float(np.clip(j, lo, hi)) for j, (lo, hi) in zip(joints, self.JOINT_LIMITS)]

    def grasp_candidates_for_target(self, straw):
        if self._measured_tcp_model:
            return [MEASURED_TCP_FINAL_STANDOFF_M]
        if straw[0] > 0.25:
            return [-0.03, 0.0]
        if straw[0] < -0.30:
            # 더 깊은 30/35mm부터 검사하되 cuRobo가 검증한 endpoint만 실행한다.
            return LEFTMOST_GRASP_RETRY_OFFSETS
        return GRASP_RETRY_OFFSETS

    def grasp_quat_variants(self):
        if self._measured_tcp_model:
            return MEASURED_TCP_GRASP_QUAT_RETRY_VARIANTS
        return GRASP_QUAT_RETRY_VARIANTS

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
        """GetState 서비스로 접촉 판정. SafeGrasp fallback 경로에서만 호출됨."""
        if self.cli_get_state is None:
            return "GRASP_UNVERIFIED", -1, -1, "get_state unavailable (virtual?)"
        if not self.cli_get_state.wait_for_service(timeout_sec=0.5):
            return "GRASP_UNVERIFIED", -1, -1, "get_state service unavailable"
        req = _GetState.Request()
        req.force_read = True
        future = self.cli_get_state.call_async(req)
        t0 = time.time()
        while not future.done() and (time.time() - t0) < GRASP_VERIFY_TIMEOUT_SEC:
            time.sleep(0.05)
        if not future.done():
            return "GRASP_UNVERIFIED", -1, -1, "get_state timeout"
        res = future.result()
        if not res or not res.success:
            return "GRASP_UNVERIFIED", -1, -1, "get_state service error"
        position = res.state.present_position
        current_raw = res.state.present_current
        if position < 0 or current_raw < 0:
            return (
                "GRASP_UNVERIFIED", position, current_raw,
                "hardware state read failed (virtual mode or serial error)")
        if position >= GRASP_EMPTY_POSITION_THRESHOLD:
            return "GRASP_EMPTY", position, current_raw, (
                f"fully closed (pos={position} >= threshold={GRASP_EMPTY_POSITION_THRESHOLD})")
        if (
            self._grasp_current_contact_threshold_raw >= 0
            and current_raw < self._grasp_current_contact_threshold_raw
        ):
            return "GRASP_UNVERIFIED", position, current_raw, (
                f"position indicates contact but current={current_raw} below calibrated "
                f"threshold={self._grasp_current_contact_threshold_raw}")
        return "GRASP_CONTACT_DETECTED", position, current_raw, (
            f"jaw stopped at pos={position} and current_raw={current_raw}; "
            f"current threshold={'disabled' if self._grasp_current_contact_threshold_raw < 0 else self._grasp_current_contact_threshold_raw}")

    def _close_and_verify_grasp(self):
        """SafeGrasp action 시도 → 서버 없으면 SetPosition+GetState fallback.

        Returns (grasp_result, present_pos, present_current_raw, grasp_reason).
        grasp_result은 GRASP_CONTACT_DETECTED | GRASP_EMPTY | GRASP_UNVERIFIED |
        GRIPPER_CLOSE_FAILED 중 하나.
        """
        if self._use_safe_grasp_action and self._safe_grasp_cli is not None:
            if self._safe_grasp_cli.server_is_ready():
                return self._close_via_safe_grasp_action()
            self.get_logger().warn(
                "SAFE_GRASP: action server not ready — fallback to set_position+get_state")
        close_ok = self._set_gripper_position(700, timeout_sec=10.0)
        if not close_ok:
            return "GRIPPER_CLOSE_FAILED", -1, -1, "set_position(700) failed"
        time.sleep(GRIPPER_CLOSE_SETTLE_SEC)
        return self._verify_grasp()

    def _close_via_safe_grasp_action(self):
        """SafeGrasp action으로 close + current 감지를 단일 원자 동작으로 수행."""
        goal = _SafeGraspAction.Goal()
        goal.target_position = 700
        goal.max_current = self._safe_grasp_max_current
        goal.current_delta_threshold = self._safe_grasp_current_delta_threshold
        goal.timeout_sec = self._safe_grasp_timeout_sec

        feedback_samples = []

        def _on_feedback(fb_msg):
            fb = fb_msg.feedback
            feedback_samples.append({
                "present_position": fb.present_position,
                "present_current": fb.present_current,
                "current_delta": fb.current_delta,
                "grasp_detected": fb.grasp_detected,
            })

        send_future = self._safe_grasp_cli.send_goal_async(
            goal, feedback_callback=_on_feedback)
        deadline = time.time() + self._safe_grasp_timeout_sec + 5.0
        while not send_future.done() and time.time() < deadline:
            time.sleep(0.05)
        if not send_future.done():
            return "GRASP_UNVERIFIED", -1, -1, "SafeGrasp: goal send timeout"

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return "GRASP_UNVERIFIED", -1, -1, "SafeGrasp: goal rejected"

        result_future = goal_handle.get_result_async()
        while not result_future.done() and time.time() < deadline:
            time.sleep(0.05)
        if not result_future.done():
            return "GRASP_UNVERIFIED", -1, -1, "SafeGrasp: result timeout"

        wrapped = result_future.result()
        if wrapped is None:
            return "GRASP_UNVERIFIED", -1, -1, "SafeGrasp: result None"

        res = wrapped.result
        final_pos = res.final_position
        final_cur = res.final_current

        self.runtime_log.log(
            "safe_grasp_action",
            grasp_detected=res.grasp_detected,
            object_lost=res.object_lost,
            final_position=final_pos,
            final_current=final_cur,
            message=res.message,
            max_current=self._safe_grasp_max_current,
            current_delta_threshold=self._safe_grasp_current_delta_threshold,
            feedback_samples=feedback_samples,
        )

        if final_pos < 0 or final_cur < 0:
            return (
                "GRASP_UNVERIFIED", final_pos, final_cur,
                f"SafeGrasp: invalid state (virtual/serial error) — {res.message}")

        if res.grasp_detected:
            if final_pos >= GRASP_EMPTY_POSITION_THRESHOLD:
                # current spike but jaw fully closed — treat as empty
                return "GRASP_EMPTY", final_pos, final_cur, (
                    f"SafeGrasp: current spike but jaw fully closed "
                    f"pos={final_pos} >= {GRASP_EMPTY_POSITION_THRESHOLD}")
            return "GRASP_CONTACT_DETECTED", final_pos, final_cur, (
                f"SafeGrasp detected: pos={final_pos} current={final_cur} — {res.message}")

        if final_pos >= GRASP_EMPTY_POSITION_THRESHOLD:
            return "GRASP_EMPTY", final_pos, final_cur, (
                f"SafeGrasp: jaw fully closed pos={final_pos} >= "
                f"{GRASP_EMPTY_POSITION_THRESHOLD} — {res.message}")

        return "GRASP_UNVERIFIED", final_pos, final_cur, (
            f"SafeGrasp: no detection, pos={final_pos} below threshold — {res.message}")

    # ── 플래닝 ────────────────────────────────────────────────────────────────

    def trajectory_in_operational_limits(self, traj_rad, label):
        traj_deg = np.rad2deg(traj_rad)
        for joint_idx, (lo, hi) in enumerate(OPERATIONAL_JOINT_LIMITS_DEG):
            vals = traj_deg[:, joint_idx]
            if np.any(vals < lo) or np.any(vals > hi):
                self.get_logger().warn(
                    f"{label} rejected: J{joint_idx+1} out of [{lo:.0f}°, {hi:.0f}°] "
                    f"(range {vals.min():.1f}°..{vals.max():.1f}°)")
                return False
        return True

    def trajectory_has_reasonable_swing(
            self, traj_rad, start_joints, label,
            max_joint_delta_deg=None):
        traj_deg = np.rad2deg(traj_rad)
        start_deg = np.rad2deg(start_joints)
        limits = max_joint_delta_deg or MAX_HARVEST_JOINT_DELTA_DEG
        if isinstance(limits, (int, float)):
            limits = [float(limits)] * len(self.JOINT_NAMES)
        for joint_idx, max_delta in enumerate(limits):
            vals = traj_deg[:, joint_idx]
            if joint_idx in WRAP_EQUIVALENT_JOINT_IDX:
                # endpoint 등가 거리가 아닌 trajectory 실제 range 검사:
                # normalize가 "돌아가는 방향"을 따라붙어도 310° 스윙을 탐지
                delta_vals = np.abs(vals - float(vals[0]))
            else:
                delta_vals = np.abs(vals - start_deg[joint_idx])
            delta = float(np.max(delta_vals))
            if delta > max_delta:
                end_deg = traj_deg[-1, joint_idx]
                self.get_logger().warn(
                    f"{label} rejected: J{joint_idx+1} swing {delta:.1f}° > {max_delta:.1f}° "
                    f"(start={start_deg[joint_idx]:.1f}° → end={end_deg:.1f}°)")
                return False
        return True

    def normalize_trajectory_equivalents(self, traj_rad, label, robot_start_joints_rad=None):
        traj_deg = np.rad2deg(traj_rad).astype(float)
        robot_start_deg = (
            np.rad2deg(robot_start_joints_rad).tolist()
            if robot_start_joints_rad is not None
            else None
        )
        rewritten = []
        for joint_idx in WRAP_EQUIVALENT_JOINT_IDX:
            lo, hi = OPERATIONAL_JOINT_LIMITS_DEG[joint_idx]
            original = traj_deg[:, joint_idx].copy()
            # Seed from the actual robot state so the entire trajectory is anchored
            # to the same ±360° representation the controller is tracking.
            # Without this, when CuRobo's first IK waypoint uses a different
            # equivalent (e.g. -54.5° vs 305.5°), the Doosan spline executor
            # physically rotates J4/J6 a full turn to reach the plan's start point.
            prev = float(robot_start_deg[joint_idx]) if robot_start_deg is not None else None
            for row_idx, value in enumerate(original):
                candidates = [value + 360.0 * k for k in range(-2, 3)]
                valid = [c for c in candidates if lo <= c <= hi]
                if not valid:
                    continue
                reference = prev if prev is not None else value
                best = min(valid, key=lambda c: abs(c - reference))
                traj_deg[row_idx, joint_idx] = best
                prev = best
            if np.max(np.abs(traj_deg[:, joint_idx] - original)) > 1e-6:
                rewritten.append(
                    f"J{joint_idx+1} {float(np.min(original)):.1f}~{float(np.max(original)):.1f}"
                    f" -> {float(np.min(traj_deg[:, joint_idx])):.1f}~{float(np.max(traj_deg[:, joint_idx])):.1f}"
                )
        if rewritten:
            self.get_logger().info(
                f"{label} joint equivalent rewrite: " + "; ".join(rewritten))
        return np.deg2rad(traj_deg)

    def trajectory_has_no_spline_jumps(self, traj_rad, label, max_jump_deg=270.0):
        """normalize 후 연속 waypoint 간 대형 각도 점프 검사.

        J4/J6가 ±한계 경계를 넘으면 normalize가 강제로 반대 부호로 바꾸면서
        직전 waypoint와 357° 차이가 생기고 Doosan 스플라인이 360° 스핀함.
        이를 실행 전에 탐지해서 plan 자체를 reject.
        """
        traj_deg = np.rad2deg(traj_rad)
        for joint_idx in WRAP_EQUIVALENT_JOINT_IDX:
            diffs = np.abs(np.diff(traj_deg[:, joint_idx]))
            if len(diffs) == 0:
                continue
            max_diff = float(np.max(diffs))
            if max_diff > max_jump_deg:
                bad_idx = int(np.argmax(diffs))
                self.get_logger().warn(
                    f"{label} rejected: J{joint_idx+1} spline jump {max_diff:.1f}° "
                    f"> {max_jump_deg:.1f}° at waypoint {bad_idx} "
                    f"(limit boundary crossing — normalize 불연속)")
                return False
        return True

    def plan(self, start_joints, target_pos, target_quat_wxyz, num_ik_seeds=32,
             max_attempts=None, timeout_sec=None, max_joint_delta_deg=None):
        t0 = time.time()
        start_joints = self._clamp_joints(start_joints)
        start_state = CuroboJointState.from_position(
            position=torch.tensor([start_joints], device="cuda:0", dtype=torch.float32),
            joint_names=self.JOINT_NAMES,
        )
        target_pose = Pose(
            position=torch.tensor([target_pos], device="cuda:0", dtype=torch.float32),
            quaternion=torch.tensor([target_quat_wxyz], device="cuda:0", dtype=torch.float32),
        )
        if self._debug_dump_plan_calls:
            # 2026-06-18: offline replay of (start_joints, target, world) with
            # identical inputs consistently found a healthy-J3 branch while the
            # live node converged to a near-singular one for the same logged
            # values — root cause still unknown. Dump the exact live inputs
            # (including world state) so a replay script can find the real
            # divergence instead of guessing. Off by default; zero cost to SW.
            self.runtime_log.log(
                "plan_call_debug_dump",
                start_joints_rad=start_joints,
                target_pos_m=target_pos,
                target_quat_wxyz=target_quat_wxyz,
                num_ik_seeds=num_ik_seeds,
                max_attempts=(
                    max_attempts if max_attempts is not None else CARTESIAN_PLAN_MAX_ATTEMPTS
                ),
                timeout_sec=(
                    timeout_sec if timeout_sec is not None else CARTESIAN_PLAN_TIMEOUT_SEC
                ),
                max_joint_delta_deg=max_joint_delta_deg,
                static_cuboids=[
                    {"name": c.name, "pose": c.pose, "dims": c.dims} for c in self.static_cuboids
                ],
                dynamic_cuboids=[
                    {"name": c.name, "pose": c.pose, "dims": c.dims} for c in self.dynamic_cuboids
                ],
                neighbor_spheres=[
                    {"name": s.name, "pose": s.pose, "radius": s.radius}
                    for s in self.neighbor_spheres
                ],
            )
        result = self.motion_gen.plan_single(
            start_state, target_pose,
            MotionGenPlanConfig(
                num_ik_seeds=num_ik_seeds,
                max_attempts=(
                    max_attempts
                    if max_attempts is not None
                    else CARTESIAN_PLAN_MAX_ATTEMPTS
                ),
                timeout=(
                    timeout_sec
                    if timeout_sec is not None
                    else CARTESIAN_PLAN_TIMEOUT_SEC
                ),
                enable_graph_attempt=None,
            ),
        )
        dt = (time.time() - t0) * 1000

        if result.success.item():
            traj = result.get_interpolated_plan().position.cpu().numpy()
            traj = self.normalize_trajectory_equivalents(traj, "Cartesian plan")
            if not self.trajectory_in_operational_limits(traj, "Cartesian plan"):
                self.runtime_log.log(
                    "curobo_plan_rejected",
                    planner="cartesian",
                    reason="operational_joint_limits",
                    start_joints_rad=start_joints,
                    target_pos_m=target_pos,
                    target_quat_wxyz=target_quat_wxyz,
                    trajectory_rad=traj,
                )
                return None
            if not self.trajectory_has_no_spline_jumps(traj, "Cartesian plan"):
                self.runtime_log.log(
                    "curobo_plan_rejected",
                    planner="cartesian",
                    reason="spline_jump",
                    start_joints_rad=start_joints,
                    target_pos_m=target_pos,
                    target_quat_wxyz=target_quat_wxyz,
                    trajectory_rad=traj,
                )
                return None
            if not self.trajectory_has_reasonable_swing(
                    traj, start_joints, "Cartesian plan",
                    max_joint_delta_deg=max_joint_delta_deg):
                self.runtime_log.log(
                    "curobo_plan_rejected",
                    planner="cartesian",
                    reason="joint_swing",
                    start_joints_rad=start_joints,
                    target_pos_m=target_pos,
                    target_quat_wxyz=target_quat_wxyz,
                    trajectory_rad=traj,
                )
                return None
            motion_time = float(result.motion_time.item())
            end_deg = [f"{np.rad2deg(v):.1f}" for v in traj[-1]]
            self.get_logger().info(
                f"Plan OK {dt:.0f}ms {traj.shape[0]}pts {motion_time:.2f}s | "
                f"goal={[f'{v*1000:.0f}' for v in target_pos]}mm | "
                f"end_J=[{', '.join(end_deg)}]°")
            self.runtime_log.log(
                "curobo_plan_success",
                planner="cartesian",
                planning_latency_ms=dt,
                motion_time_sec=motion_time,
                start_joints_rad=start_joints,
                target_pos_m=target_pos,
                target_quat_wxyz=target_quat_wxyz,
                trajectory_rad=traj,
            )
            return traj, motion_time
        else:
            status = str(getattr(result, "status", "UNKNOWN"))
            start_deg = [f"{np.rad2deg(v):.1f}" for v in start_joints]
            self.get_logger().error(
                f"Plan FAIL {dt:.0f}ms | status={status} | "
                f"goal={[f'{v*1000:.0f}' for v in target_pos]}mm | "
                f"start_J=[{', '.join(start_deg)}]°")
            if "INVALID_START_STATE_WORLD_COLLISION" in status:
                self.diagnose_start_world_collision(start_joints, "Cartesian plan")
            self.runtime_log.log(
                "curobo_plan_fail",
                planner="cartesian",
                status=status,
                planning_latency_ms=dt,
                start_joints_rad=start_joints,
                target_pos_m=target_pos,
                target_quat_wxyz=target_quat_wxyz,
            )
            return None

    def plan_js(self, start_joints, target_joints_rad, label, skip_swing_check=False,
                max_joint_delta_deg=None):
        t0 = time.time()
        start_joints = self._clamp_joints(start_joints)
        target_joints_rad = self._clamp_joints(target_joints_rad)
        start_state = CuroboJointState.from_position(
            position=torch.tensor([start_joints], device="cuda:0", dtype=torch.float32),
            joint_names=self.JOINT_NAMES,
        )
        goal_state = CuroboJointState.from_position(
            position=torch.tensor([target_joints_rad], device="cuda:0", dtype=torch.float32),
            joint_names=self.JOINT_NAMES,
        )
        result = self.motion_gen.plan_single_js(
            start_state, goal_state, MotionGenPlanConfig(enable_graph=True)
        )
        dt = (time.time() - t0) * 1000

        if result.success.item():
            traj = result.get_interpolated_plan().position.cpu().numpy()
            traj = self.normalize_trajectory_equivalents(traj, label)
            if not self.trajectory_in_operational_limits(traj, label):
                self.runtime_log.log(
                    "curobo_plan_rejected", planner="joint_space", label=label,
                    reason="operational_joint_limits", trajectory_rad=traj)
                return None
            if not self.trajectory_has_no_spline_jumps(traj, label):
                self.runtime_log.log(
                    "curobo_plan_rejected", planner="joint_space", label=label,
                    reason="spline_jump", trajectory_rad=traj)
                return None
            if not skip_swing_check and not self.trajectory_has_reasonable_swing(
                    traj, start_joints, label, max_joint_delta_deg):
                self.runtime_log.log(
                    "curobo_plan_rejected", planner="joint_space", label=label,
                    reason="joint_swing", trajectory_rad=traj)
                return None
            motion_time = float(result.motion_time.item())
            self.get_logger().info(
                f"{label} JS Plan OK {dt:.0f}ms {traj.shape[0]}pts {motion_time:.2f}s | "
                f"goal={[f'{v:.1f}' for v in np.rad2deg(target_joints_rad)]}°")
            self.runtime_log.log(
                "curobo_plan_success",
                planner="joint_space",
                label=label,
                planning_latency_ms=dt,
                motion_time_sec=motion_time,
                start_joints_rad=start_joints,
                target_joints_rad=target_joints_rad,
                trajectory_rad=traj,
            )
            return traj, motion_time

        status = getattr(result, "status", "?")
        self.get_logger().error(
            f"{label} JS Plan FAIL {dt:.0f}ms | status={status} | "
            f"goal={[f'{v:.1f}' for v in np.rad2deg(target_joints_rad)]}°")
        if "INVALID_START_STATE_WORLD_COLLISION" in str(status) or "GRAPH_FAIL" in str(status):
            self.diagnose_js_endpoint_collision(start_joints, target_joints_rad, label)
        self.runtime_log.log(
            "curobo_plan_fail",
            planner="joint_space",
            label=label,
            status=str(status),
            planning_latency_ms=dt,
            start_joints_rad=start_joints,
            target_joints_rad=target_joints_rad,
        )
        return None

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
        if not self.cli_spline.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("MoveSplineJoint not available")
            return False
        traj_deg = np.rad2deg(traj_rad)
        n = traj_deg.shape[0]
        if n > MAX_SPLINE_POINTS:
            idx = np.linspace(0, n - 1, MAX_SPLINE_POINTS, dtype=int)
            traj_deg = traj_deg[idx]
            n = MAX_SPLINE_POINTS

        from std_msgs.msg import Float64MultiArray as F64MA
        req = MoveSplineJoint.Request()
        req.pos_cnt = n
        for row in traj_deg:
            pt = F64MA()
            pt.data = row.tolist()
            req.pos.append(pt)
        req.vel = [SPLINE_VEL_DEG_S] * 6
        req.acc = [SPLINE_ACC_DEG_S2] * 6
        req.time = max(float(motion_time) * SPLINE_TIME_SCALE, SPLINE_MIN_TIME)
        req.mode = 0
        req.sync_type = 0

        self.get_logger().info(
            f"Spline {n}pts plan={motion_time:.2f}s exec={req.time:.2f}s "
            f"→ end={[f'{v:.1f}' for v in traj_deg[-1]]}°")
        self.runtime_log.log(
            "motion_command",
            controller="doosan_move_spline_joint",
            service="/dsr01/motion/move_spline_joint",
            trajectory_deg=traj_deg,
            planned_motion_time_sec=motion_time,
            requested_time_sec=req.time,
            velocity_deg_s=req.vel,
            acceleration_deg_s2=req.acc,
        )
        future = self.cli_spline.call_async(req)
        t0 = time.time()
        while not future.done() and (time.time() - t0) < 60.0:
            time.sleep(0.05)

        ok = future.done() and future.result() and future.result().success
        if not ok:
            self.get_logger().error("Spline failed/timeout")
        self.runtime_log.log(
            "motion_result",
            controller="doosan_move_spline_joint",
            success=bool(ok),
            current_joints_rad=self.current_joints,
        )
        return ok

    def execute_tool_z_line(self, distance_m: float, motion_label="FINAL_APPROACH_STRAIGHT",
                            vel_mm_s: float = None, acc_mm_s2: float = None,
                            min_distance_m: float = 0.02) -> bool:
        """현재 TCP 자세를 유지하고 TOOL Z축 방향으로 직선 이동."""
        if not min_distance_m <= abs(distance_m) <= 0.25:
            self.get_logger().error(
                f"MoveLine rejected: {motion_label} distance={distance_m*1000:.1f}mm "
                f"allowed={min_distance_m*1000:.1f}..250.0mm")
            return False
        if not self.cli_movel.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("MoveLine not available")
            return False

        vel = vel_mm_s if vel_mm_s is not None else FINAL_APPROACH_VEL_MM_S
        acc = acc_mm_s2 if acc_mm_s2 is not None else FINAL_APPROACH_ACC_MM_S2

        req = MoveLine.Request()
        req.pos = [0.0, 0.0, float(distance_m * 1000.0), 0.0, 0.0, 0.0]
        req.vel = [vel, 10.0]
        req.acc = [acc, 20.0]
        req.time = 0.0
        req.radius = 0.0
        req.ref = 1         # DR_TOOL
        req.mode = 1        # DR_MV_MOD_REL
        req.blend_type = 0
        req.sync_type = 0   # SYNC: 완전히 도착한 뒤 응답

        self.get_logger().info(
            f"{motion_label} TOOL {'+Z' if distance_m > 0 else '-Z'} "
            f"{abs(distance_m)*1000:.1f}mm "
            f"vel={vel:.1f}mm/s")
        self.runtime_log.log(
            "motion_command",
            controller="doosan_move_line",
            label=motion_label,
            service="/dsr01/motion/move_line",
            reference_frame="tool",
            relative_pose_mm_deg=req.pos,
            velocity=req.vel,
            acceleration=req.acc,
        )
        start_joints = (
            np.array(self.current_joints, dtype=float)
            if self.current_joints is not None
            else None
        )
        future = self.cli_movel.call_async(req)
        t0 = time.time()
        # The Doosan operation-speed slider scales the commanded velocity.
        # At 10%, a nominal 180 mm / 50 mm/s move takes about 36 s, so a fixed
        # 30 s timeout aborts just before arrival and prevents gripper close.
        nominal_motion_sec = abs(distance_m * 1000.0) / max(float(vel), 1.0)
        timeout_sec = max(30.0, nominal_motion_sec * 10.0 + 10.0)
        while not future.done() and (time.time() - t0) < timeout_sec:
            time.sleep(0.05)
        ok = future.done() and future.result() and future.result().success
        elapsed_sec = time.time() - t0
        min_expected_sec = max(0.5, nominal_motion_sec * 0.6)
        if ok and elapsed_sec < min_expected_sec:
            remaining_sec = min_expected_sec - elapsed_sec
            self.get_logger().warn(
                f"{motion_label} MoveLine returned early "
                f"({elapsed_sec:.2f}s < expected {min_expected_sec:.2f}s); "
                f"waiting {remaining_sec:.2f}s before continuing")
            time.sleep(remaining_sec)
        if ok and start_joints is not None and abs(distance_m) > 0.05:
            end_joints = (
                np.array(self.current_joints, dtype=float)
                if self.current_joints is not None
                else start_joints
            )
            joint_delta_deg = np.degrees(np.abs(end_joints - start_joints))
            max_delta_deg = float(np.max(joint_delta_deg))
            if max_delta_deg < 0.5:
                self.get_logger().error(
                    f"{motion_label} MoveLine reported success but joints barely moved "
                    f"(max_delta={max_delta_deg:.2f}deg); treating as failed")
                ok = False
        if not ok:
            self.get_logger().error(
                f"{motion_label} MoveLine failed/timeout>{timeout_sec:.1f}s")
        self.runtime_log.log(
            "motion_result",
            controller="doosan_move_line",
            label=motion_label,
            success=bool(ok),
            timeout_sec=timeout_sec,
            elapsed_sec=elapsed_sec,
            current_joints_rad=self.current_joints,
        )
        return ok

    def _execute_pitch_detach(self) -> bool:
        """파지 후 TCP를 BASE -Z 방향으로 당겨 줄기 분리. 회전 없음."""
        if not self.cli_movel.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn("DETACH_PULL: MoveLine service unavailable")
            return False
        req = MoveLine.Request()
        req.pos = [0.0, 0.0, -float(DETACH_PULL_DOWN_MM), 0.0, 0.0, 0.0]
        req.vel = [float(DETACH_PULL_VEL_MM_S), 10.0]
        req.acc = [30.0, 20.0]
        req.time = 0.0
        req.radius = 0.0
        req.ref = 0   # BASE frame
        req.mode = 1  # RELATIVE
        req.blend_type = 0
        req.sync_type = 0
        future = self.cli_movel.call_async(req)
        t0 = time.time()
        while not future.done() and (time.time() - t0) < 30.0:
            time.sleep(0.05)
        ok = future.done() and bool(future.result() and future.result().success)
        self.get_logger().info(
            f"DETACH_PULL_DOWN: BASE -Z {DETACH_PULL_DOWN_MM:.0f}mm "
            f"→ {'OK' if ok else 'FAIL'}")
        self.runtime_log.log("detach_pull_down",
                             pull_mm=DETACH_PULL_DOWN_MM,
                             vel_mm_s=DETACH_PULL_VEL_MM_S, success=ok)
        return ok

    def execute_base_z_relative(self, distance_m: float, motion_label: str,
                                vel_mm_s: float = 30.0) -> bool:
        """BASE 기준 Z축 상대 직선 이동. 크레인 접근/이탈 하강·상승에 사용."""
        return self.execute_base_relative_line(
            [0.0, 0.0, float(distance_m)], motion_label, vel_mm_s)

    def execute_base_relative_line(self, delta_m, motion_label: str,
                                   vel_mm_s: float = 30.0,
                                   acc_mm_s2: float = 30.0) -> bool:
        """BASE 기준 XYZ 상대 직선 이동."""
        if not self.cli_movel.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(f"{motion_label}: MoveLine service unavailable")
            return False
        delta_m = np.array(delta_m, dtype=float)
        distance_m = float(np.linalg.norm(delta_m))
        if not 0.005 <= distance_m <= 0.30:
            self.get_logger().error(
                f"{motion_label}: BASE relative distance "
                f"{distance_m*1000:.1f}mm outside 5..300mm")
            return False
        req = MoveLine.Request()
        req.pos = [
            float(delta_m[0] * 1000.0),
            float(delta_m[1] * 1000.0),
            float(delta_m[2] * 1000.0),
            0.0, 0.0, 0.0,
        ]
        req.vel = [float(vel_mm_s), 10.0]
        req.acc = [float(acc_mm_s2), 20.0]
        req.time = 0.0
        req.radius = 0.0
        req.ref = 0   # DR_BASE
        req.mode = 1  # DR_MV_MOD_REL
        req.blend_type = 0
        req.sync_type = 0
        self.get_logger().info(
            f"{motion_label} BASE REL "
            f"xyz={[round(v * 1000.0, 1) for v in delta_m]}mm "
            f"dist={distance_m*1000:.1f}mm vel={vel_mm_s:.1f}mm/s")
        self.runtime_log.log(
            "motion_command",
            controller="doosan_move_line",
            label=motion_label,
            service="/dsr01/motion/move_line",
            reference_frame="base",
            relative_pose_mm_deg=req.pos,
            velocity=req.vel,
        )
        start_joints = (
            np.array(self.current_joints, dtype=float)
            if self.current_joints is not None
            else None
        )
        future = self.cli_movel.call_async(req)
        t0 = time.time()
        nominal_motion_sec = distance_m * 1000.0 / max(float(vel_mm_s), 1.0)
        timeout_sec = max(10.0, nominal_motion_sec * 3.0 + 5.0)
        while not future.done() and (time.time() - t0) < timeout_sec:
            time.sleep(0.05)
        ok = future.done() and bool(future.result() and future.result().success)
        elapsed_sec = time.time() - t0
        min_expected_sec = max(0.5, nominal_motion_sec * 0.6)
        if ok and elapsed_sec < min_expected_sec:
            remaining_sec = min_expected_sec - elapsed_sec
            self.get_logger().warn(
                f"{motion_label} BASE MoveLine returned early "
                f"({elapsed_sec:.2f}s < expected {min_expected_sec:.2f}s); "
                f"waiting {remaining_sec:.2f}s before continuing")
            time.sleep(remaining_sec)
        if ok and start_joints is not None and distance_m > 0.05:
            end_joints = (
                np.array(self.current_joints, dtype=float)
                if self.current_joints is not None
                else start_joints
            )
            max_delta_deg = float(np.max(np.degrees(np.abs(end_joints - start_joints))))
            if max_delta_deg < 0.5:
                self.get_logger().error(
                    f"{motion_label} BASE MoveLine reported success but joints barely moved "
                    f"(max_delta={max_delta_deg:.2f}deg); treating as failed")
                ok = False
        self.runtime_log.log(
            "motion_result",
            controller="doosan_move_line",
            label=motion_label,
            success=bool(ok),
            timeout_sec=timeout_sec,
            elapsed_sec=elapsed_sec,
        )
        if not ok:
            self.get_logger().error(f"{motion_label} BASE relative failed/timeout")
        return ok

    def execute_base_line(self, posx_mm_deg, motion_label, vel_mm_s=20.0) -> bool:
        """베이스 기준 절대 TCP 직선 이동. Marker place의 수직 above/release에만 사용."""
        if len(posx_mm_deg) != 6:
            self.get_logger().error(f"{motion_label}: expected 6D posx")
            return False
        if not self.cli_movel.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("MoveLine not available")
            return False

        req = MoveLine.Request()
        req.pos = [float(v) for v in posx_mm_deg]
        req.vel = [float(vel_mm_s), 10.0]
        req.acc = [30.0, 20.0]
        req.time = 0.0
        req.radius = 0.0
        req.ref = 0         # DR_BASE
        req.mode = 0        # DR_MV_MOD_ABS
        req.blend_type = 0
        req.sync_type = 0

        self.get_logger().info(
            f"{motion_label} BASE ABS "
            f"xyz={[round(v, 1) for v in req.pos[:3]]}mm "
            f"abc={[round(v, 1) for v in req.pos[3:]]}deg")
        self.runtime_log.log(
            "motion_command",
            controller="doosan_move_line",
            label=motion_label,
            service="/dsr01/motion/move_line",
            reference_frame="base",
            absolute_pose_mm_deg=req.pos,
            velocity=req.vel,
            acceleration=req.acc,
        )
        future = self.cli_movel.call_async(req)
        t0 = time.time()
        while not future.done() and (time.time() - t0) < 60.0:
            time.sleep(0.05)
        ok = future.done() and future.result() and future.result().success
        if not ok:
            self.get_logger().error(f"{motion_label} MoveLine failed/timeout")
        self.runtime_log.log(
            "motion_result",
            controller="doosan_move_line",
            label=motion_label,
            success=bool(ok),
            current_joints_rad=self.current_joints,
        )
        return ok

    def _doosan_zyz_to_wxyz(self, rx_deg: float, ry_deg: float, rz_deg: float):
        """Doosan ZYZ Euler (deg) → quaternion [w, x, y, z] for cuRobo."""
        r = SciR.from_euler("ZYZ", [rx_deg, ry_deg, rz_deg], degrees=True)
        xyzw = r.as_quat()
        return [float(xyzw[3]), float(xyzw[0]), float(xyzw[1]), float(xyzw[2])]

    def _curobo_fk_ee_pose(self, joints_rad):
        """cuRobo robot model 기준 현재 ee_link pose를 반환한다."""
        q = torch.tensor(
            [joints_rad], device="cuda:0", dtype=torch.float32)
        state = self.motion_gen.kinematics.get_state(q)
        return (
            state.ee_position[0].detach().cpu().numpy().tolist(),
            state.ee_quaternion[0].detach().cpu().numpy().tolist(),
        )

    def _trajectory_line_deviation_mm(self, traj_rad, start_pos_m, end_pos_m):
        """FK 궤적의 목표 Cartesian 선분 대비 최대 측방 편차를 계산한다."""
        q = torch.tensor(traj_rad, device="cuda:0", dtype=torch.float32)
        state = self.motion_gen.kinematics.get_state(q)
        points = state.ee_position.detach().cpu().numpy()
        start = np.array(start_pos_m, dtype=float)
        end = np.array(end_pos_m, dtype=float)
        line = end - start
        line_norm_sq = float(np.dot(line, line))
        if line_norm_sq < 1e-12:
            return float("inf"), -1
        fractions = np.clip(((points - start) @ line) / line_norm_sq, 0.0, 1.0)
        projected = start + fractions[:, None] * line
        deviations_mm = np.linalg.norm(points - projected, axis=1) * 1000.0
        max_index = int(np.argmax(deviations_mm))
        return float(deviations_mm[max_index]), max_index

    def _quat_from_tool_z(self, tool_z, roll_deg=0.0):
        """원하는 world TOOL +Z 방향과 roll로 quaternion [w,x,y,z] 생성."""
        z_axis = np.array(tool_z, dtype=float)
        z_axis /= np.linalg.norm(z_axis)
        reference = np.array([0.0, 0.0, 1.0])
        if abs(float(np.dot(reference, z_axis))) > 0.95:
            reference = np.array([1.0, 0.0, 0.0])
        x_axis = np.cross(reference, z_axis)
        x_axis /= np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)
        rotation = SciR.from_matrix(np.column_stack((x_axis, y_axis, z_axis)))
        rotation = rotation * SciR.from_euler("Z", roll_deg, degrees=True)
        xyzw = rotation.as_quat()
        return [float(xyzw[3]), float(xyzw[0]), float(xyzw[1]), float(xyzw[2])]

    def _marker_place_orientation_candidates(self, tray_view_joints, target_pos_m):
        """Tray place용 orientation 후보.

        tray-view FK와 tray JSON 자세는 현재 slot ABOVE에서 모두 IK_FAIL이었다.
        완전 top-down은 260mm tool 때문에 손목을 작업반경 바깥으로 밀 수 있다.
        따라서 계란판 방향으로 기울어진 사선 하향 TOOL +Z 후보를 먼저 탐색한다.
        """
        _, tray_view_quat = self._curobo_fk_ee_pose(tray_view_joints)
        candidates = [("tray_view_fk", tray_view_quat)]

        target = np.array(target_pos_m, dtype=float)
        radial_xy = np.array([target[0], target[1], 0.0])
        radial_xy /= np.linalg.norm(radial_xy)
        for down_component in (-0.25, -0.50, -0.75):
            tool_z = radial_xy + np.array([0.0, 0.0, down_component])
            tool_z /= np.linalg.norm(tool_z)
            for roll_deg in (0.0, 90.0, -90.0, 180.0):
                candidates.append((
                    f"inclined_down_{abs(down_component):.2f}_roll_{roll_deg:+.0f}",
                    self._quat_from_tool_z(tool_z, roll_deg),
                ))

        for yaw_deg in (0.0, 45.0, -45.0, 90.0, -90.0, 180.0):
            rotation = (
                SciR.from_euler("Z", yaw_deg, degrees=True)
                * SciR.from_euler("X", 180.0, degrees=True)
            )
            xyzw = rotation.as_quat()
            candidates.append((
                f"top_down_yaw_{yaw_deg:+.0f}",
                [float(xyzw[3]), float(xyzw[0]), float(xyzw[1]), float(xyzw[2])],
            ))
        return candidates

    def _nearest_equivalent_joints(self, base_joints_deg):
        """J4/J6를 현재 위치에서 가장 가까운 360° equivalent로 조정."""
        if self.current_joints is None:
            return base_joints_deg
        current_deg = np.rad2deg(self.current_joints)
        joints = list(base_joints_deg)
        for i in WRAP_EQUIVALENT_JOINT_IDX:
            lo, hi = OPERATIONAL_JOINT_LIMITS_DEG[i]
            candidates = [joints[i] + 360.0 * k for k in range(-2, 3)]
            valid = [c for c in candidates if lo <= c <= hi]
            if valid:
                joints[i] = min(valid, key=lambda c: abs(c - current_deg[i]))
        return joints

    def home_joints_near_current(self):
        return self._nearest_equivalent_joints(HOME_JOINTS_DEG)

    def overview_joints_near_current(self):
        return self._nearest_equivalent_joints(OVERVIEW_JOINTS_DEG)

    def movej_direct(self, joints_deg, vel=40.0, acc=60.0):
        """cuRobo 우회 — Doosan MoveJoint 직접 호출. 최후 수단용."""
        if not self.cli_movej.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("MoveJoint service not available")
            return False
        req = MoveJoint.Request()
        req.pos = [float(v) for v in joints_deg]
        req.vel = float(vel)
        req.acc = float(acc)
        req.time = 0.0
        req.radius = 0.0
        req.mode = 0
        req.blend_type = 0
        req.sync_type = 0
        self.get_logger().warn(
            f"MoveJoint direct → {[round(v, 1) for v in joints_deg]}° vel={vel}")
        future = self.cli_movej.call_async(req)
        t0 = time.time()
        while not future.done() and (time.time() - t0) < 90.0:
            time.sleep(0.05)
        ok = future.done() and future.result() and future.result().success
        if not ok:
            self.get_logger().error("MoveJoint direct failed/timeout")
        return ok

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
        if self._tray_cells_json:
            return self._tray_cells_json
        files = sorted(
            glob.glob(DEFAULT_TRAY_CELLS_GLOB),
            key=os.path.getmtime,
            reverse=True,
        )
        return files[0] if files else None

    def _marker_cell_with_taught_grid_pitch(self, cells, slot_index):
        """마커의 위치/회전 방향에 Slot0·1·3 실측 tray pitch를 적용한다."""
        by_index = {int(cell.get("index", idx)): cell for idx, cell in enumerate(cells)}
        if not all(index in by_index for index in (0, 1, 3, slot_index)):
            raise ValueError("marker JSON missing required slot0/1/3/target cells")

        cell0 = by_index[0]
        selected = dict(by_index[slot_index])
        contact0 = np.array([
            float(cell0["position_contact_mm"][axis]) for axis in ("x", "y", "z")
        ])
        contact1 = np.array([
            float(by_index[1]["position_contact_mm"][axis]) for axis in ("x", "y", "z")
        ])
        contact3 = np.array([
            float(by_index[3]["position_contact_mm"][axis]) for axis in ("x", "y", "z")
        ])
        marker_vertical = contact1 - contact0
        marker_horizontal = contact3 - contact0
        marker_vertical_norm = float(np.linalg.norm(marker_vertical))
        marker_horizontal_norm = float(np.linalg.norm(marker_horizontal))
        if marker_vertical_norm < 1e-6 or marker_horizontal_norm < 1e-6:
            raise ValueError("invalid marker tray grid axis")

        taught_slot0 = np.array(TAUGHT_SLOT0_PLACE_REFERENCE_POSX_MM_DEG[:3])
        taught_vertical_pitch = float(np.linalg.norm(
            np.array(TAUGHT_SLOT1_PLACE_REFERENCE_POSX_MM_DEG[:3]) - taught_slot0))
        taught_horizontal_pitch = float(np.linalg.norm(
            np.array(TAUGHT_SLOT3_PLACE_REFERENCE_POSX_MM_DEG[:3]) - taught_slot0))
        horizontal_idx, vertical_idx = divmod(slot_index, 3)
        calibrated_contact = (
            contact0
            + vertical_idx * taught_vertical_pitch * marker_vertical / marker_vertical_norm
            + horizontal_idx * taught_horizontal_pitch * marker_horizontal / marker_horizontal_norm
        )

        original_contact = np.array([
            float(selected["position_contact_mm"][axis]) for axis in ("x", "y", "z")
        ])
        correction = calibrated_contact - original_contact
        selected["position_contact_mm"] = {
            axis: float(calibrated_contact[idx])
            for idx, axis in enumerate(("x", "y", "z"))
        }
        if "position_tcp_mm" in selected:
            selected["position_tcp_mm"] = {
                axis: float(selected["position_tcp_mm"][axis]) + float(correction[idx])
                for idx, axis in enumerate(("x", "y", "z"))
            }
        return selected, {
            "marker_vertical_pitch_mm": marker_vertical_norm,
            "marker_horizontal_pitch_mm": marker_horizontal_norm,
            "taught_vertical_pitch_mm": taught_vertical_pitch,
            "taught_horizontal_pitch_mm": taught_horizontal_pitch,
            "position_correction_mm": correction.tolist(),
        }

    def _load_marker_place_target(self):
        path = self._latest_tray_cells_json()
        if not path or not os.path.isfile(path):
            self.get_logger().error("MARKER_PLACE_BLOCKED: tray cells JSON not found")
            return None
        age_sec = time.time() - os.path.getmtime(path)
        if age_sec > self._marker_place_max_age_sec:
            self.get_logger().error(
                f"MARKER_PLACE_BLOCKED: tray localization stale "
                f"age={age_sec:.0f}s > {self._marker_place_max_age_sec:.0f}s")
            return None
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            cells = data.get("cells", [])
            if not cells:
                raise ValueError("no cells")
            slot_index = self._marker_place_slot_idx % len(cells)
            cell, grid_calibration = self._marker_cell_with_taught_grid_pitch(
                cells, slot_index)
            orient = cell["task_orientation_deg"]
            if self._measured_tcp_model and "position_contact_mm" in cell:
                # share_tray의 position_tcp_mm는 기존 Robotis TCP에서 연장 파츠
                # 120mm를 보정한 좌표다. measured grasp_tcp_link는 이미 물리 파지
                # 중심(파츠 끝보다 약 10mm 뒤)을 ee_link로 사용하므로 중복 보정을
                # 피하고 contact point에서 TOOL +Z 반대 방향 10mm만 이동한다.
                contact = cell["position_contact_mm"]
                rotation = SciR.from_euler(
                    "ZYZ",
                    [
                        float(orient["rx"]),
                        float(orient["ry"]),
                        float(orient["rz"]),
                    ],
                    degrees=True,
                )
                tool_z = rotation.apply([0.0, 0.0, 1.0])
                tcp = {
                    axis: float(contact[axis]) - 10.0 * float(tool_z[idx])
                    for idx, axis in enumerate(("x", "y", "z"))
                }
                target_source = "measured_grasp_center_from_contact_minus_10mm"
            else:
                tcp = cell["position_tcp_mm"]
                target_source = "legacy_position_tcp_mm"
            release = [
                float(tcp["x"]), float(tcp["y"]), float(tcp["z"]),
                float(orient["rx"]), float(orient["ry"]), float(orient["rz"]),
            ]
            if not (
                -800.0 <= release[0] <= 800.0
                and -800.0 <= release[1] <= 800.0
                and 250.0 <= release[2] <= 1200.0
            ):
                raise ValueError(f"target outside guarded workspace: {release[:3]}")
        except Exception as exc:
            self.get_logger().error(f"MARKER_PLACE_BLOCKED: invalid tray JSON ({exc})")
            return None

        above = list(release)
        above[2] += self._marker_place_above_clearance_m * 1000.0
        gripper_offset = data.get("gripper_offset") or {}
        self.runtime_log.log(
            "marker_place_target_loaded",
            path=path,
            age_sec=age_sec,
            slot_index=cell.get("index"),
            row=cell.get("row"),
            col=cell.get("col"),
            release_posx_mm_deg=release,
            above_posx_mm_deg=above,
            source_standoff_mm=gripper_offset.get("fingertip_standoff_mm"),
            target_source=target_source,
            source_contact_mm=cell.get("position_contact_mm"),
            source_legacy_tcp_mm=cell.get("position_tcp_mm"),
            grid_calibration=grid_calibration,
        )
        self.get_logger().info(
            f"MARKER_TRAY_GRID slot={cell.get('index')} "
            f"marker_pitch(vertical/horizontal)="
            f"{grid_calibration['marker_vertical_pitch_mm']:.1f}/"
            f"{grid_calibration['marker_horizontal_pitch_mm']:.1f}mm -> "
            f"taught_pitch="
            f"{grid_calibration['taught_vertical_pitch_mm']:.1f}/"
            f"{grid_calibration['taught_horizontal_pitch_mm']:.1f}mm "
            f"correction={np.round(grid_calibration['position_correction_mm'], 1).tolist()}mm")
        return {
            "path": path,
            "slot_index": int(cell.get("index", self._marker_place_slot_idx)),
            "release": release,
            "above": above,
            "target_source": target_source,
            "contact_mm": cell.get("position_contact_mm"),
        }

    def _taught_grid_slot_offset_m(self, slot_index: int):
        """Slot0/1/3 실측 위치로부터 지정 슬롯의 BASE 위치 오프셋을 계산한다."""
        slot0 = np.array(TAUGHT_SLOT0_PLACE_REFERENCE_POSX_MM_DEG[:3], dtype=float)
        slot1 = np.array(TAUGHT_SLOT1_PLACE_REFERENCE_POSX_MM_DEG[:3], dtype=float)
        slot3 = np.array(TAUGHT_SLOT3_PLACE_REFERENCE_POSX_MM_DEG[:3], dtype=float)
        horizontal_idx, vertical_idx = divmod(slot_index, 3)
        offset_mm = horizontal_idx * (slot3 - slot0) + vertical_idx * (slot1 - slot0)
        return (offset_mm / 1000.0).tolist()

    def _execute_taught_slot0_place_reference_after_retreat(self, retreat_joints):
        """Slot0 FK와 실측 격자 벡터로 생성한 슬롯에 수직 Place한다."""
        slot_index = self._marker_place_slot_idx
        if not 0 <= slot_index < TAUGHT_TRAY_SLOT_COUNT:
            self.get_logger().error(
                f"TAUGHT_TRAY_PLACE_COMPLETE: slot index {slot_index} out of range")
            return "tray_complete", retreat_joints
        self.get_logger().warn(
            f"TAUGHT_TRAY_GRID_PLACE active: slot={slot_index}; fixed tray pose only; "
            "marker localization is bypassed")

        reference_deg = self._nearest_equivalent_joints(
            TAUGHT_SLOT0_PLACE_REFERENCE_JOINTS_DEG)
        reference_rad = np.deg2rad(reference_deg).tolist()
        release_fk_pos_m, release_fk_quat = self._curobo_fk_ee_pose(reference_rad)
        is_row2 = (slot_index % 3 == 2)
        if is_row2 and self._row2_place_pitch_tilt_deg != 0.0:
            w, x, y, z = release_fk_quat
            base_rot = SciR.from_quat([x, y, z, w])
            tilt_rot = SciR.from_euler('y', self._row2_place_pitch_tilt_deg, degrees=True)
            tilted = tilt_rot * base_rot
            q = tilted.as_quat()
            release_fk_quat = [float(q[3]), float(q[0]), float(q[1]), float(q[2])]
            self.get_logger().info(
                f"ROW2_PLACE_TILT: {self._row2_place_pitch_tilt_deg:.1f}deg pitch "
                f"quat_wxyz={[round(v, 4) for v in release_fk_quat]}")
        slot_offset_m = self._taught_grid_slot_offset_m(slot_index)
        release_pos_m = (
            np.array(release_fk_pos_m, dtype=float)
            + np.array(slot_offset_m, dtype=float)
        ).tolist()
        if is_row2 and any(v != 0.0 for v in self._row2_release_correction_mm):
            corr_m = np.array(self._row2_release_correction_mm, dtype=float) / 1000.0
            release_pos_m = (np.array(release_pos_m, dtype=float) + corr_m).tolist()
            self.get_logger().info(
                f"ROW2_RELEASE_CORRECTION: {self._row2_release_correction_mm}mm "
                f"→ release_pos={[round(v*1000,1) for v in release_pos_m]}mm")
        above_pos_m = list(release_pos_m)
        clearance_m = self._taught_slot_above_clearance_m
        above_pos_m[2] += clearance_m
        self.get_logger().info(
            f"TAUGHT_TRAY_SLOT{slot_index}_ABOVE generated from Slot0 FK + grid offset: "
            f"clearance={clearance_m*1000:.0f}mm "
            f"goal_mm={[round(v * 1000, 1) for v in above_pos_m]}")
        self._ensure_operation_speed(30)
        above_plan = self.plan(
            retreat_joints, above_pos_m, release_fk_quat,
            num_ik_seeds=64, max_attempts=3, timeout_sec=2.0,
            max_joint_delta_deg=MAX_TAUGHT_PLACE_TRANSFER_JOINT_DELTA_DEG)
        if above_plan is None or not self.execute_spline(*above_plan):
            self.get_logger().error(
                f"TAUGHT_TRAY_SLOT{slot_index}_PLACE_BLOCKED: "
                "above plan failed; holding fruit")
            return "failed", retreat_joints
        above_joints = list(above_plan[0][-1].tolist())

        row2_descent_plan = None
        if is_row2:
            # Preview에서도 release 경로와 측방 편차를 계산해 실제 하강 전에
            # 안전성을 확인할 수 있게 한다.
            self.get_logger().info(
                "TAUGHT_SLOT0_RELEASE_DESCEND cuRobo continuous (row2): "
                "plan once and validate Cartesian line deviation")
            row2_descent_plan = self.plan(
                above_joints, release_pos_m, release_fk_quat,
                num_ik_seeds=64, max_attempts=3, timeout_sec=2.0,
                max_joint_delta_deg=MAX_TAUGHT_PLACE_TRANSFER_JOINT_DELTA_DEG)
            if row2_descent_plan is None:
                self.get_logger().error(
                    f"TAUGHT_TRAY_SLOT{slot_index}_PLACE_BLOCKED: "
                    "release descent plan failed; holding fruit")
                return "failed", list(self.current_joints or above_joints)
            row2_traj, _ = row2_descent_plan
            line_deviation_mm, deviation_index = self._trajectory_line_deviation_mm(
                row2_traj, above_pos_m, release_pos_m)
            self.get_logger().info(
                f"ROW2_DESCENT_LINE_CHECK max_deviation={line_deviation_mm:.1f}mm "
                f"limit={self._row2_max_line_deviation_mm:.1f}mm "
                f"waypoint={deviation_index}")
            self.runtime_log.log(
                "row2_cartesian_line_check",
                phase="descent",
                slot_index=slot_index,
                max_deviation_mm=line_deviation_mm,
                limit_mm=self._row2_max_line_deviation_mm,
                max_deviation_waypoint=deviation_index,
            )
            if line_deviation_mm > self._row2_max_line_deviation_mm:
                self.get_logger().error(
                    f"TAUGHT_TRAY_SLOT{slot_index}_PLACE_BLOCKED: row2 descent "
                    f"deviates {line_deviation_mm:.1f}mm from Cartesian line "
                    f"(limit {self._row2_max_line_deviation_mm:.1f}mm)")
                return "failed", list(self.current_joints or above_joints)

        self.runtime_log.log(
            "taught_slot0_place_above_reached",
            slot_index=slot_index,
            release_enabled=self._execute_marker_place_release,
            reference_joints_deg=reference_deg,
            reference_posx_mm_deg=TAUGHT_SLOT0_PLACE_REFERENCE_POSX_MM_DEG,
            slot_offset_m=slot_offset_m,
            generated_release_pos_m=release_pos_m,
            above_pos_m=above_pos_m,
            above_clearance_m=clearance_m,
        )
        if not self._execute_marker_place_release:
            self.get_logger().warn(
                f"TAUGHT_TRAY_SLOT{slot_index}_PLACE_PREVIEW_HOLD: "
                "above reached; release disabled")
            return "preview_hold", list(self.current_joints or above_joints)

        if slot_index != 0 and not self._allow_generated_tray_slot_release:
            self.get_logger().error(
                f"TAUGHT_TRAY_SLOT{slot_index}_RELEASE_BLOCKED: slot pose is generated "
                "from the Slot0/1/3 grid and has not been physically taught/verified. "
                "Holding at Above; teach the actual slot release pose before enabling.")
            self.runtime_log.log(
                "generated_tray_slot_release_blocked",
                slot_index=slot_index,
                reason="generated_slot_not_physically_verified",
                above_pos_m=above_pos_m,
                release_pos_m=release_pos_m,
            )
            return "preview_hold", list(self.current_joints or above_joints)

        if is_row2:
            # Preview에서 검증한 동일 궤적을 정지 없이 단일 spline으로 실행한다.
            # 독립 hop/분할 실행은 J4 branch 전환과 구간별 정지를 유발하므로
            # 사용하지 않는다.
            self.get_logger().info(
                "TAUGHT_SLOT0_RELEASE_DESCEND cuRobo continuous (row2): "
                "executing validated one-spline trajectory")
            full_traj, full_time = row2_descent_plan
            if not self.execute_spline(full_traj, full_time):
                self.get_logger().error(
                    f"TAUGHT_TRAY_SLOT{slot_index}_PLACE_BLOCKED: "
                    "continuous descent spline failed; holding fruit")
                return "failed", list(self.current_joints or above_joints)
            release_joints = list(full_traj[-1].tolist())
        else:
            self.get_logger().info(
                f"TAUGHT_SLOT0_RELEASE_DESCEND BASE -Z {round(clearance_m*1000)}mm")
            if not self.execute_base_z_relative(
                    -clearance_m,
                    "TAUGHT_SLOT0_RELEASE_DESCEND",
                    TAUGHT_SLOT0_VERTICAL_VEL_MM_S):
                self.get_logger().error(
                    f"TAUGHT_TRAY_SLOT{slot_index}_PLACE_BLOCKED: "
                    "vertical release descend failed; holding fruit")
                return "failed", list(self.current_joints or above_joints)
            release_joints = list(self.current_joints or above_joints)

        self.get_logger().warn(
            f"TAUGHT_TRAY_SLOT{slot_index}_PLACE_RELEASE: "
            f"position_cmd={GRIPPER_PLACE_RELEASE_POS}")
        self.runtime_log.log(
            "gripper_command", command="set_position",
            position=GRIPPER_PLACE_RELEASE_POS, slot_index=slot_index,
            source="taught_slot0_grid_reference")
        self._set_gripper_position(GRIPPER_PLACE_RELEASE_POS, timeout_sec=3.0)

        if is_row2:
            self.get_logger().info(
                "TAUGHT_SLOT0_RELEASE_ASCEND cuRobo continuous (row2): "
                "plan once, validate Cartesian line deviation, execute one spline")
            ascent_plan = self.plan(
                release_joints, above_pos_m, release_fk_quat,
                num_ik_seeds=64, max_attempts=3, timeout_sec=2.0,
                max_joint_delta_deg=MAX_TAUGHT_PLACE_TRANSFER_JOINT_DELTA_DEG)
            if ascent_plan is None:
                self.get_logger().error(
                    f"TAUGHT_TRAY_SLOT{slot_index}_RELEASED_BUT_ASCEND_FAILED: "
                    "ascent plan failed; holding position")
                return "failed_after_release", list(self.current_joints or release_joints)
            asc_traj, asc_time = ascent_plan
            line_deviation_mm, deviation_index = self._trajectory_line_deviation_mm(
                asc_traj, release_pos_m, above_pos_m)
            self.get_logger().info(
                f"ROW2_ASCENT_LINE_CHECK max_deviation={line_deviation_mm:.1f}mm "
                f"limit={self._row2_max_line_deviation_mm:.1f}mm "
                f"waypoint={deviation_index}")
            self.runtime_log.log(
                "row2_cartesian_line_check",
                phase="ascent",
                slot_index=slot_index,
                max_deviation_mm=line_deviation_mm,
                limit_mm=self._row2_max_line_deviation_mm,
                max_deviation_waypoint=deviation_index,
            )
            if line_deviation_mm > self._row2_max_line_deviation_mm:
                self.get_logger().error(
                    f"TAUGHT_TRAY_SLOT{slot_index}_RELEASED_BUT_ASCEND_FAILED: "
                    f"row2 ascent deviates {line_deviation_mm:.1f}mm from Cartesian "
                    f"line (limit {self._row2_max_line_deviation_mm:.1f}mm)")
                return "failed_after_release", list(self.current_joints or release_joints)
            if not self.execute_spline(asc_traj, asc_time):
                self.get_logger().error(
                    f"TAUGHT_TRAY_SLOT{slot_index}_RELEASED_BUT_ASCEND_FAILED: "
                    "continuous ascent spline failed; holding position")
                return "failed_after_release", list(self.current_joints or release_joints)
        else:
            if not self.execute_base_z_relative(
                    clearance_m,
                    "TAUGHT_SLOT0_RELEASE_ASCEND",
                    TAUGHT_SLOT0_VERTICAL_VEL_MM_S):
                self.get_logger().error(
                    f"TAUGHT_TRAY_SLOT{slot_index}_RELEASED_BUT_ASCEND_FAILED: "
                    "holding position")
                return "failed_after_release", list(self.current_joints or above_joints)

        self._marker_place_slot_idx += 1
        self.runtime_log.log(
            "marker_place_complete",
            result_code="PLACE_SEQUENCE_COMPLETE_UNVERIFIED",
            slot_index=slot_index,
            source="taught_slot0_grid_reference",
        )
        return "success", list(self.current_joints or above_joints)

    def _execute_marker_place_after_retreat(self, retreat_joints):
        """Marker-derived place. Release 승인 전에는 above에서 정지한다."""
        if self._use_taught_slot0_place_reference:
            return self._execute_taught_slot0_place_reference_after_retreat(
                retreat_joints)

        target = self._load_marker_place_target()
        if target is None:
            return "skip", retreat_joints   # tray 없음/stale → soft skip, hold 없음

        self.get_logger().info(
            f"5 marker place slot={target['slot_index']} via overview/tray-view "
            f"source={target['target_source']}")
        overview_deg = self.overview_joints_near_current()
        ok, overview_joints = self.plan_to_fixed_joints_pose(
            retreat_joints, overview_deg, "marker place transfer overview",
            skip_swing_check=True)
        if not ok:
            self.get_logger().error(
                "MARKER_PLACE_BLOCKED: transfer overview plan failed; holding fruit")
            return "failed", retreat_joints

        tray_view_deg = self._nearest_equivalent_joints(TRAY_VIEW_JOINTS_DEG)
        ok, tray_view_joints = self.plan_to_fixed_joints_pose(
            overview_joints, tray_view_deg, "marker place tray view",
            skip_swing_check=True)
        if not ok:
            self.get_logger().error(
                "MARKER_PLACE_BLOCKED: tray-view plan failed; holding fruit")
            return "failed", overview_joints

        # Tray localization은 Doosan controller TCP orientation을 저장하지만 cuRobo
        # measured grasp_tcp_link와 convention/model 차이로 그대로는 IK_FAIL이 난다.
        # 도달 가능한 place orientation을 preview ABOVE에서 먼저 선택한다.
        tray_view_fk_pos, tray_view_quat = self._curobo_fk_ee_pose(tray_view_joints)
        json_place_quat = self._doosan_zyz_to_wxyz(*target["above"][3:])
        quat_dot = min(1.0, abs(float(np.dot(tray_view_quat, json_place_quat))))
        quat_delta_deg = float(np.rad2deg(2.0 * np.arccos(quat_dot)))
        self.get_logger().info(
            f"MARKER_PLACE_ORIENTATION_SEARCH tray-view/json delta="
            f"{quat_delta_deg:.1f}deg")

        # ABOVE: 하향 place 자세 후보 중 도달 가능한 첫 경로를 선택한다. Measured
        # TCP는 orientation마다 파츠 끝->파지 중심 10mm 방향이 달라지므로 후보별로
        # release/above 위치를 contact point에서 다시 계산한다. Tray contact가 이미
        # 면에서 60mm 위이므로, 100mm ABOVE가 작업반경 경계에서 IK_FAIL이면 더 낮은
        # clearance도 안전 후보로 탐색한다.
        default_above_pos_m = [v / 1000.0 for v in target["above"][:3]]
        self.get_logger().info(
            f"MARKER_PLACE_ABOVE cuRobo "
            f"xyz={[round(v, 1) for v in target['above'][:3]]}mm "
            f"abc={[round(v, 1) for v in target['above'][3:]]}deg")
        above_plan = None
        selected_orientation_name = None
        above_quat = None
        selected_release_pos_m = None
        selected_above_pos_m = None
        requested_clearance = self._marker_place_above_clearance_m
        clearance_candidates = []
        for clearance_m in (requested_clearance, 0.070, 0.050, 0.030):
            if clearance_m > 0.0 and not any(
                    abs(clearance_m - existing) < 1e-6
                    for existing in clearance_candidates):
                clearance_candidates.append(clearance_m)

        for clearance_m in clearance_candidates:
            for orientation_name, candidate_quat in self._marker_place_orientation_candidates(
                    tray_view_joints, default_above_pos_m):
                candidate_release_pos_m = [v / 1000.0 for v in target["release"][:3]]
                candidate_above_pos_m = list(default_above_pos_m)
                if self._measured_tcp_model and target.get("contact_mm"):
                    contact_m = np.array([
                        float(target["contact_mm"][axis]) / 1000.0
                        for axis in ("x", "y", "z")
                    ])
                    candidate_xyzw = [
                        candidate_quat[1], candidate_quat[2],
                        candidate_quat[3], candidate_quat[0],
                    ]
                    candidate_tool_z = SciR.from_quat(candidate_xyzw).apply(
                        [0.0, 0.0, 1.0])
                    candidate_release_pos_m = (
                        contact_m - 0.010 * candidate_tool_z).tolist()
                    candidate_above_pos_m = list(candidate_release_pos_m)
                    candidate_above_pos_m[2] += clearance_m
                else:
                    candidate_above_pos_m[2] += (
                        clearance_m - requested_clearance)
                radial_distance_m = float(np.linalg.norm(candidate_above_pos_m))
                candidate_xyzw = [
                    candidate_quat[1], candidate_quat[2],
                    candidate_quat[3], candidate_quat[0],
                ]
                candidate_tool_z = SciR.from_quat(candidate_xyzw).apply([0.0, 0.0, 1.0])
                implied_flange_pos_m = (
                    np.array(candidate_above_pos_m)
                    - MEASURED_FLANGE_TO_GRASP_CENTER_M * candidate_tool_z
                )
                implied_flange_distance_m = float(np.linalg.norm(implied_flange_pos_m))
                self.get_logger().info(
                    f"MARKER_PLACE_ABOVE trying clearance={clearance_m*1000:.0f}mm "
                    f"orientation={orientation_name} tcp_r={radial_distance_m:.3f}m "
                    f"flange_r={implied_flange_distance_m:.3f}m "
                    f"goal_mm={[round(v * 1000, 1) for v in candidate_above_pos_m]}")
                candidate_plan = self.plan(
                    tray_view_joints, candidate_above_pos_m, candidate_quat,
                    num_ik_seeds=64, max_attempts=3, timeout_sec=2.0)
                if candidate_plan is not None:
                    above_plan = candidate_plan
                    above_quat = candidate_quat
                    selected_orientation_name = orientation_name
                    selected_release_pos_m = candidate_release_pos_m
                    selected_above_pos_m = candidate_above_pos_m
                    selected_clearance_m = clearance_m
                    break
            if above_plan is not None:
                break
        if above_plan is None:
            self.get_logger().error(
                "MARKER_PLACE_BLOCKED: all above orientation candidates failed; "
                "holding fruit")
            return "failed", tray_view_joints
        self.get_logger().info(
            f"MARKER_PLACE_ORIENTATION selected={selected_orientation_name} "
            f"clearance={selected_clearance_m*1000:.0f}mm")
        self.runtime_log.log(
            "marker_place_orientation_selected",
            source=selected_orientation_name,
            tray_view_fk_pos_m=tray_view_fk_pos,
            selected_quat_wxyz=above_quat,
            selected_above_pos_m=selected_above_pos_m,
            selected_release_pos_m=selected_release_pos_m,
            selected_clearance_m=selected_clearance_m,
            tray_view_quat_wxyz=tray_view_quat,
            json_quat_wxyz=json_place_quat,
            tray_view_json_angular_delta_deg=quat_delta_deg,
        )
        ok_above = self.execute_spline(*above_plan)
        above_joints = list(
            above_plan[0][-1].tolist() if ok_above else tray_view_joints)
        if not ok_above:
            self.get_logger().error("MARKER_PLACE_BLOCKED: above spline exec failed; holding fruit")
            return "failed", tray_view_joints

        if not self._execute_marker_place_release:
            self.get_logger().warn(
                "MARKER_PLACE_PREVIEW_HOLD: above reached; release disabled. "
                "Inspect clearance before enabling execute_marker_place_release.")
            return "preview_hold", list(self.current_joints or above_joints)

        # RELEASE: cuRobo Cartesian plan — avoids kinematic flip caused by BASE ABS
        release_pos_m = selected_release_pos_m
        release_quat = above_quat
        self.get_logger().info(
            f"MARKER_PLACE_RELEASE_DESCEND cuRobo "
            f"xyz={[round(v, 1) for v in target['release'][:3]]}mm "
            f"abc={[round(v, 1) for v in target['release'][3:]]}deg")
        release_plan = self.plan(above_joints, release_pos_m, release_quat)
        if release_plan is None:
            self.get_logger().error(
                "MARKER_PLACE_BLOCKED: release cuRobo plan failed; holding fruit")
            return "failed", list(self.current_joints or above_joints)
        ok_release = self.execute_spline(*release_plan)
        if not ok_release:
            self.get_logger().error(
                "MARKER_PLACE_BLOCKED: release spline exec failed; holding fruit")
            return "failed", list(self.current_joints or above_joints)
        release_joints = list(release_plan[0][-1].tolist())

        self.get_logger().info(
            f"6 marker place release gripper position_cmd={GRIPPER_PLACE_RELEASE_POS}")
        self.runtime_log.log(
            "gripper_command", command="set_position",
            position=GRIPPER_PLACE_RELEASE_POS,
            slot_index=target["slot_index"])
        self._set_gripper_position(GRIPPER_PLACE_RELEASE_POS, timeout_sec=3.0)

        # RETREAT: release pose에서 먼저 above로 상승한 뒤 tray-view로 복귀한다.
        # release에서 tray-view 관절 자세로 바로 이동하면 tray body를 가로지를 수 있다.
        above_retreat_plan = self.plan(
            release_joints, selected_above_pos_m, above_quat)
        if above_retreat_plan is None or not self.execute_spline(*above_retreat_plan):
            self.get_logger().error(
                "MARKER_PLACE_RELEASED_BUT_ABOVE_RETREAT_FAILED: holding position")
            return "failed_after_release", list(self.current_joints or release_joints)
        above_retreat_joints = list(above_retreat_plan[0][-1].tolist())

        # Known tray-view configuration으로 collision-aware joint-space 복귀.
        tray_view_deg_retreat = self._nearest_equivalent_joints(TRAY_VIEW_JOINTS_DEG)
        ok_retreat, _ = self.plan_to_fixed_joints_pose(
            above_retreat_joints, tray_view_deg_retreat,
            "MARKER_PLACE_TRAY_VIEW_RETURN")
        if not ok_retreat:
            self.get_logger().error(
                "MARKER_PLACE_RELEASED_BUT_RETREAT_FAILED: holding position")
            return "failed_after_release", list(
                self.current_joints or above_retreat_joints)

        self._marker_place_slot_idx += 1
        self.runtime_log.log(
            "marker_place_complete",
            result_code="PLACE_SEQUENCE_COMPLETE_UNVERIFIED",
            slot_index=target["slot_index"],
            tray_cells_json=target["path"],
        )
        return "success", list(self.current_joints or tray_view_joints)

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

        # Y 클램핑: berry는 벽 표면보다 뒤에 있을 수 없음 (FK drift 보정)
        detection_raw_y = float(p.y)   # 클램핑 전 원본값 — measured TCP 적응형 접근 거리 계산용
        raw_y = detection_raw_y
        wall_y_clamped = raw_y > WALL_SURFACE_Y_M
        if wall_y_clamped:
            self.get_logger().warn(
                f"Detection Y={raw_y*1000:.0f}mm > wall surface {WALL_SURFACE_Y_M*1000:.0f}mm "
                f"(FK calibration drift) — clamped to {WALL_SURFACE_Y_M*1000:.0f}mm")
            raw_y = WALL_SURFACE_Y_M
        raw_straw = np.array([p.x, raw_y, max(p.z, 0.05)])
        straw = raw_straw + np.array([
            self._pick_target_x_bias_m,
            0.0,
            self._pick_target_z_bias_m,
        ])
        straw[2] = max(straw[2], 0.05)
        is_nw_high_target = (
            self._measured_tcp_model
            and float(raw_straw[2]) >= self._nw_high_target_z_threshold_m
        )
        # NW high target만 KP1 진입 높이/open descent 거리를 별도로 줄여볼 수 있게
        # 분리한 값. SW(및 다른 NW 타겟)는 기존 CRANE_Z_OFFSET_M 그대로 유지.
        crane_z_offset_m = (
            self._nw_high_target_crane_z_offset_m
            if is_nw_high_target else CRANE_Z_OFFSET_M
        )
        if is_nw_high_target and wall_y_clamped and NW_HIGH_TARGET_Y_PLANE_RELAX_M > 0.0:
            before_y = float(straw[1])
            straw[1] += NW_HIGH_TARGET_Y_PLANE_RELAX_M
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

        x_min, x_max = DIRECT_GRASP_TARGET_X_RANGE_M
        if not (x_min <= float(raw_straw[0]) <= x_max):
            self.get_logger().warn(
                f"ABORT: pick target x={raw_straw[0]*1000:.0f}mm outside "
                f"[{x_min*1000:.0f}, {x_max*1000:.0f}]mm")
            self.pick_complete_pub.publish(Empty())
            return
        if self._measured_tcp_model and float(raw_straw[2]) > MEASURED_TCP_TARGET_Z_MAX_M:
            self.get_logger().warn(
                f"SKIP: pick target z={raw_straw[2]*1000:.0f}mm > "
                f"{MEASURED_TCP_TARGET_Z_MAX_M*1000:.0f}mm "
                "(NW high/leaf candidate guard)")
            self.runtime_log.log(
                "pick_target_skipped",
                reason="target_z_above_measured_tcp_guard",
                raw_target_m=raw_straw,
                z_max_m=MEASURED_TCP_TARGET_Z_MAX_M,
                wall_y_clamped=wall_y_clamped,
            )
            self.pick_complete_pub.publish(Empty())
            return

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
        self.get_logger().info(
            f"1 open gripper for stem descent: set_position={GRIPPER_APPROACH_POS}")
        self.runtime_log.log(
            "gripper_command",
            command="set_position",
            position=GRIPPER_APPROACH_POS,
            purpose="open_during_horizontal_approach_and_stem_descent",
        )
        self._set_gripper_position(GRIPPER_APPROACH_POS, timeout_sec=3.0)

        self._register_neighbor_obstacles(straw)
        self.motion_gen.detach_object_from_robot()

        if raw_straw[0] < -0.30 and not self._measured_tcp_model:
            straw[0] += LEFTMOST_GRASP_X_CORR_M

        # 2. Grasp (cuRobo 2-step): 6cm pre-approach → 직선 진입
        # 직전 측방 편차가 줄기 형상/검출점 영향인지 분리하기 위해 6cm를 재검증한다.
        grasp_quat_variants = (
            NW_HIGH_TARGET_GRASP_QUAT_RETRY_VARIANTS
            if is_nw_high_target else self.grasp_quat_variants()
        )
        grasp_quat_variants = list(grasp_quat_variants)
        published_roll_variant = self._published_roll_grasp_variant(input_quat_wxyz)
        if published_roll_variant is not None:
            grasp_quat_variants = [published_roll_variant] + grasp_quat_variants

        def variant_label(variant):
            frame, axis, deg = variant
            if frame == "published_roll":
                return f"published_roll({deg:+.0f}deg)"
            return f"{deg:+.0f}deg"

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
        ret_pre   = None
        ret_grasp = None
        used_grasp_offset = None
        used_grasp_variant = None
        used_approach_dir = None
        used_grasp_quat = None
        used_pre_ee_pos = None
        used_grasp_ee_pos = None
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
        measured_best_depth_m = -1.0
        # 2026-06-18: depth probing picks the deepest reachable standoff, but
        # multiple grasp_quat_variants often reach the IDENTICAL depth with
        # wildly different elbow health (J3 from ~0deg/near-singular up to
        # ~60deg/healthy) — verified by replaying a real failing run offline
        # bit-for-bit (replay_plan_call_dump.py). The old `depth_m >
        # measured_best_depth_m` strict-greater comparison always kept the
        # FIRST variant tried on a tie, which was consistently the worst one.
        # Track J3 health as a tiebreaker so an equally-deep but healthier
        # elbow from a later variant can replace it.
        measured_best_j3_deg = None
        measured_best_alignment_deg = None
        grasp_attempt = 0
        for quat_frame, axis, quat_deg in grasp_quat_variants:
            if quat_frame == "published_roll":
                q_retry = axis
            else:
                q_delta = quat_from_axis_angle(axis, np.deg2rad(quat_deg))
                if quat_frame == "base":
                    q_retry = quat_multiply_wxyz(q_delta, WALL_QUAT_WXYZ)
                else:
                    q_retry = quat_multiply_wxyz(WALL_QUAT_WXYZ, q_delta)
            approach_dir = np.array(quat_rotate_vec(q_retry, [0.0, 0.0, 1.0]))
            ee_pre = straw - (
                PRE_APPROACH_OFFSET + self._ee_to_tcp_offset_m
            ) * approach_dir
            if self._measured_tcp_model and crane_z_offset_m > 0:
                # KP1 위쪽에서 수평 진입을 끝낸 뒤, 열린 그리퍼로 BASE -Z
                # 하강하여 KP1에서 파지한다.
                ee_pre = ee_pre + np.array([0.0, 0.0, crane_z_offset_m])
            r_pre_for_variant = self.plan(
                self.current_joints, ee_pre.tolist(), q_retry, num_ik_seeds=24
            )
            if r_pre_for_variant is None:
                grasp_attempt += len(grasp_retry_offsets)
                continue
            pre_joints = r_pre_for_variant[0][-1].tolist()

            if self._measured_tcp_model:
                # Measured TCP에서 pre-approach만 보고 첫 자세를 확정하면,
                # NW처럼 자세가 빡빡한 영역에서 final 접근 IK가 계속 막힌다.
                # 실행 전 각 orientation의 final depth reachability를 probing해
                # 가장 깊게 들어갈 수 있는 자세를 고른다.
                requested_probe_depth_m = max(
                    0.060, min(MEASURED_TCP_MAX_APPROACH_CEILING_M, self._measured_tcp_max_approach_m))
                if is_nw_high_target:
                    # Repeated NW high-cell 실기에서 110mm 이상 final endpoint는
                    # 매번 IK_FAIL이었다. 90mm cuRobo + 90mm TOOL finish가
                    # 검증된 직선진입 구조이므로 불가능한 깊은 후보부터
                    # 두드리며 시간을 쓰지 않는다.
                    probe_depths = [
                        d for d in NW_HIGH_TARGET_PROBE_DEPTHS_M
                        if d <= requested_probe_depth_m + 1e-6
                    ]
                else:
                    probe_depths = [requested_probe_depth_m]
                    for depth_m in [0.150, 0.130, 0.110, 0.090, 0.070, 0.060]:
                        if 0.001 < depth_m < requested_probe_depth_m - 0.005:
                            probe_depths.append(depth_m)
                if measured_best_depth_m >= MEASURED_TCP_MIN_PRUNE_DEPTH_M:
                    # If one orientation already proved that deeper endpoints fail,
                    # do not repeat those expensive IK_FAIL probes for every later
                    # orientation. Later variants are now used mainly to find a
                    # healthier elbow at the same reachable depth.
                    probe_depths = [
                        d for d in probe_depths
                        if d <= measured_best_depth_m + 1e-6
                    ]
                    if not probe_depths:
                        probe_depths = [measured_best_depth_m]
                    self.get_logger().info(
                        "MEASURED_TCP_PROBE_PRUNED: existing best depth="
                        f"{measured_best_depth_m*1000:.0f}mm; "
                        f"next depths={[round(d*1000) for d in probe_depths]}mm")
                elif measured_best_depth_m > 0.0:
                    self.get_logger().info(
                        "MEASURED_TCP_PROBE_NOT_PRUNED: existing best depth="
                        f"{measured_best_depth_m*1000:.0f}mm < "
                        f"{MEASURED_TCP_MIN_PRUNE_DEPTH_M*1000:.0f}mm minimum; "
                        "later variants may still reach the proven 90mm TOOL finish branch")
                for depth_m in probe_depths:
                    probe_target = ee_pre + depth_m * approach_dir
                    r_final_probe = self.plan(
                        pre_joints,
                        probe_target.tolist(),
                        q_retry,
                        num_ik_seeds=24,
                        max_attempts=2,
                        timeout_sec=1.5,
                        max_joint_delta_deg=90.0,
                    )
                    grasp_attempt += 1
                    if r_final_probe is None:
                        continue
                    candidate_j3_deg = abs(float(np.rad2deg(r_final_probe[0][-1][2])))
                    best_is_published_roll = (
                        measured_best is not None
                        and measured_best[3][0] == "published_roll"
                    )
                    candidate_is_published_roll = quat_frame == "published_roll"
                    candidate_alignment_deg = (
                        0.0 if candidate_is_published_roll else abs(float(quat_deg))
                    )
                    is_deeper = depth_m > measured_best_depth_m + 1e-6
                    is_tied = abs(depth_m - measured_best_depth_m) <= 1e-6
                    if (
                        best_is_published_roll
                        and not candidate_is_published_roll
                        and measured_best_j3_deg is not None
                        and measured_best_j3_deg >= NW_HIGH_TARGET_MIN_FLAT_BRANCH_J3_DEG
                    ):
                        # If the per-target stem-aligned candidate is already
                        # reachable with a non-singular elbow, do not let the
                        # generic +15deg library steal the same-depth solution
                        # just because J3 is a little healthier. That was the
                        # observed root of "bent stems still get side approach".
                        is_tied_but_better = False
                        is_deeper = False
                    elif is_nw_high_target and is_tied:
                        candidate_flat_safe = (
                            candidate_j3_deg >= NW_HIGH_TARGET_MIN_FLAT_BRANCH_J3_DEG)
                        best_flat_safe = (
                            measured_best_j3_deg is not None
                            and measured_best_j3_deg >= NW_HIGH_TARGET_MIN_FLAT_BRANCH_J3_DEG)
                        is_tied_but_better = (
                            candidate_flat_safe
                            and (
                                not best_flat_safe
                                or measured_best_alignment_deg is None
                                or candidate_alignment_deg < measured_best_alignment_deg - 1e-6
                                or (
                                    abs(candidate_alignment_deg - measured_best_alignment_deg) <= 1e-6
                                    and (
                                        measured_best_j3_deg is None
                                        or candidate_j3_deg > measured_best_j3_deg
                                    )
                                )
                            )
                        )
                    else:
                        is_tied_but_better = (
                            is_tied
                            and (
                                measured_best_j3_deg is None
                                or candidate_j3_deg > measured_best_j3_deg
                            )
                        )
                    if is_deeper or is_tied_but_better:
                        measured_best_depth_m = depth_m
                        measured_best_j3_deg = candidate_j3_deg
                        measured_best_alignment_deg = candidate_alignment_deg
                        measured_best = (
                            r_pre_for_variant,
                            r_final_probe,
                            MEASURED_TCP_FINAL_STANDOFF_M,
                            (quat_frame, axis, quat_deg),
                            approach_dir,
                            q_retry,
                            ee_pre.copy(),
                            probe_target.copy(),
                        )
                        self.get_logger().info(
                            "MEASURED_TCP_FINAL_PROBE_BEST "
                            f"depth={depth_m*1000:.0f}mm J3={candidate_j3_deg:.1f}deg "
                            f"align={candidate_alignment_deg:.1f}deg "
                            f"variant={(quat_frame, axis, quat_deg)}"
                            + (" (tie-break: flatter safe branch)" if is_tied_but_better else ""))
                    if depth_m >= requested_probe_depth_m - 1e-6:
                        break
                if measured_best_depth_m >= requested_probe_depth_m - 1e-6:
                    break
                if (
                    measured_best_j3_deg is not None
                    and measured_best_j3_deg >= (
                        NW_HIGH_TARGET_J3_GOOD_ENOUGH_DEG
                        if is_nw_high_target else MEASURED_TCP_J3_GOOD_ENOUGH_DEG
                    )
                ):
                    good_enough_j3_deg = (
                        NW_HIGH_TARGET_J3_GOOD_ENOUGH_DEG
                        if is_nw_high_target else MEASURED_TCP_J3_GOOD_ENOUGH_DEG
                    )
                    self.get_logger().info(
                        "MEASURED_TCP_VARIANT_SEARCH_STOPPED "
                        f"J3={measured_best_j3_deg:.1f}deg >= "
                        f"{good_enough_j3_deg:.0f}deg good-enough threshold — "
                        "skipping remaining grasp_quat_variants")
                    break
                continue

            for grasp_offset in grasp_retry_offsets:
                grasp_attempt += 1
                # 2-step 구조에서 grasp endpoint는 pre-approach보다 target에
                # 가까워야 한다. 6cm pre에서 7cm offset을 허용하면 직선 진입이
                # 음수가 되어 정확도 보장 목적이 깨진다.
                if grasp_offset >= PRE_APPROACH_OFFSET:
                    continue
                ee_g_try = straw - (
                    grasp_offset + self._ee_to_tcp_offset_m
                ) * approach_dir
                r_grasp = self.plan(pre_joints, ee_g_try.tolist(), q_retry, num_ik_seeds=32)
                if r_grasp is None:
                    continue
                ret_pre   = r_pre_for_variant
                ret_grasp = r_grasp
                used_grasp_offset = grasp_offset
                used_grasp_variant = (quat_frame, axis, quat_deg)
                used_approach_dir = approach_dir
                used_grasp_quat = q_retry
                used_pre_ee_pos = ee_pre.copy()
                used_grasp_ee_pos = ee_g_try.copy()
                break
            if ret_pre is not None:
                break

        if self._measured_tcp_model and measured_best is not None:
            (
                ret_pre,
                ret_grasp,
                used_grasp_offset,
                used_grasp_variant,
                used_approach_dir,
                used_grasp_quat,
                used_pre_ee_pos,
                used_grasp_ee_pos,
            ) = measured_best
            self.runtime_log.log(
                "measured_tcp_final_probe_selected",
                selected_depth_m=measured_best_depth_m,
                grasp_variant=used_grasp_variant,
                pre_ee_pos_m=used_pre_ee_pos.tolist(),
                final_ee_pos_m=used_grasp_ee_pos.tolist(),
            )

        if (
            ret_pre is not None
            and raw_straw[0] < -0.30
            and used_grasp_offset >= 0.050
        ):
            self.get_logger().warn(
                f"LEFTMOST_DEPTH_LIMITED: deeper 30/35/40/45mm endpoints rejected; "
                f"using {used_grasp_offset*1000:.0f}mm stand-off")
            self.runtime_log.log(
                "leftmost_depth_limited",
                selected_grasp_offset_m=used_grasp_offset,
                attempted_offsets_m=[
                    value for value in LEFTMOST_GRASP_RETRY_OFFSETS
                    if value < used_grasp_offset
                ],
                reason="deeper_endpoints_rejected",
            )

        if ret_pre is None:
            self.get_logger().error(
                f"ABORT: grasp 전체 실패 — {grasp_attempt}개 후보 모두 reject "
                f"(target=({straw[0]*1000:.0f},{straw[1]*1000:.0f},{straw[2]*1000:.0f})mm "
                f"start_J=[{', '.join(f'{np.rad2deg(v):.0f}' for v in self.current_joints)}]°)")
            self._clear_neighbor_obstacles()
            self._reset_gripper()
            self.pick_complete_pub.publish(Empty())
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
        if self._measured_tcp_model:
            # 딸기마다 실제 깊이가 다름 → wall clamp 이후의 유효 Y로 진입 거리 계산.
            # clamp 전 detection_raw_y를 쓰면 FK drift가 final approach를 벽 뒤로
            # 밀어 넣어 cuRobo fallback goal이 Y>wall로 튄다.
            # baseline(180mm)보다 깊은 딸기만 추가 진입; baseline 미만으로 줄이지 않음
            baseline_approach = PRE_APPROACH_OFFSET - MEASURED_TCP_FINAL_STANDOFF_M  # 0.180m
            pre_approach_y_m = WALL_SURFACE_Y_M - PRE_APPROACH_OFFSET  # 0.612m
            effective_detection_y = raw_straw[1]
            adaptive_dist = (effective_detection_y - Y_DETECTION_BIAS_M) - pre_approach_y_m
            target_plane_dist = float(np.dot(straw - used_pre_ee_pos, used_approach_dir))
            uncapped_distance = max(baseline_approach, min(adaptive_dist, 0.260))
            final_approach_distance = max(
                0.0,
                min(uncapped_distance, self._measured_tcp_max_approach_m),
            )
            # 2026-06-20 실기 확인: z=717mm 타겟(is_nw_high_target=False, 750mm
            # 미달)도 동일한 +15deg 틸트 variant를 골랐는데, wall_y_clamped로
            # adaptive_dist가 baseline(180mm)에 floor-lock되면서 final_extra가
            # 전혀 안 붙어 깊이가 그대로 얕았다(measured_tcp_max_approach_m을
            # 200mm로 올려도 효과 없음 — uncapped_distance 자체가 180mm라
            # max approach cap이 발동할 일이 없었음). 다른 두 틸트 보정과
            # 동일하게 높이 대신 실제 틸트로 게이팅한다. 틸트가 0이면
            # (SW) 동작 변화 없음.
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
        else:
            final_approach_distance = PRE_APPROACH_OFFSET - used_grasp_offset
        if not self.execute_spline(*ret_pre):
            self.get_logger().error("ABORT: pre-approach spline 실패")
            self._clear_neighbor_obstacles()
            self._reset_gripper()
            self.pick_complete_pub.publish(Empty())
            return
        self.get_logger().info(
            f"PRE_APPROACH_REACHED — settling {PRE_APPROACH_SETTLE_SEC:.1f}s "
            f"before {final_approach_distance*1000:.0f}mm straight approach")
        time.sleep(PRE_APPROACH_SETTLE_SEC)

        if final_approach_distance > 0.001:
            requested_final_approach_distance = final_approach_distance
            precomputed_final_attempted = False
            if (
                self._measured_tcp_model
                and self._direct_curobo_final_approach_for_measured_tcp
            ):
                selected_curobo_depth_m = measured_best_depth_m
                precomputed_final_attempted = True
                approach_ok = (
                    ret_grasp is not None
                    and selected_curobo_depth_m > 0.0
                    and self.execute_spline(*ret_grasp)
                )
                self.runtime_log.log(
                    "final_approach_precomputed_curobo",
                    controller="curobo_plus_doosan_move_spline_joint",
                    requested_distance_m=final_approach_distance,
                    executed_depth_m=selected_curobo_depth_m,
                    success=approach_ok,
                    approach_dir=used_approach_dir,
                )
                if approach_ok:
                    self.get_logger().info(
                        "FINAL_APPROACH_PRECOMPUTED_CUROBO "
                        f"depth={selected_curobo_depth_m*1000:.0f}mm "
                        "(reusing probe plan; no extra IK fallback search)")
                    final_approach_distance = selected_curobo_depth_m
                    used_grasp_ee_pos = (
                        used_pre_ee_pos
                        + final_approach_distance * used_approach_dir)
                    remaining_tool_line_m = (
                        requested_final_approach_distance
                        - selected_curobo_depth_m)
                    if (
                        self._measured_tcp_tool_line_after_curobo_fallback
                        and remaining_tool_line_m >= 0.020
                    ):
                        self.get_logger().warn(
                            "FINAL_APPROACH_TOOL_FINISH: cuRobo reached "
                            f"{selected_curobo_depth_m*1000:.0f}mm only; executing "
                            f"remaining {remaining_tool_line_m*1000:.0f}mm with "
                            "TOOL +Z MoveLine like the proven SW baseline")
                        self.runtime_log.log(
                            "final_approach_tool_finish_requested",
                            reason="curobo_deep_final_approach_ik_fail",
                            curobo_depth_m=selected_curobo_depth_m,
                            tool_finish_m=remaining_tool_line_m,
                            requested_total_m=requested_final_approach_distance,
                        )
                        # 2026-06-20 실기 확인: 틸트(+15deg 등)가 있는 채로 TOOL+Z
                        # 직선을 쓰면 이 마지막 구간 전체가 대각선으로 같이
                        # 떠오른다(사용자 직접 관찰: "수평이 아니라 대각선으로 살짝
                        # 위로 올라감"). 처음엔 is_nw_high_target(z>=750mm)에만
                        # 적용했는데, z<750mm 타겟도 같은 +15deg variant를 고르면
                        # 똑같이 재현됨(실기 로그로 확인) — 즉 진짜 원인은 "높은
                        # 타겟"이 아니라 "틸트된 variant"다. 그래서 타겟 높이가
                        # 아니라 실제 선택된 approach_dir의 Z 성분으로 분기한다.
                        # 틸트가 0이면 horiz_dir == used_approach_dir이라 SW처럼
                        # 평평한 접근은 동작이 전혀 안 바뀐다.
                        if (
                            used_grasp_variant is not None
                            and used_grasp_variant[0] == "published_roll"
                        ) or abs(float(used_approach_dir[2])) > 1e-3:
                            horiz_dir = np.array(used_approach_dir, dtype=float)
                            horiz_dir[2] = 0.0
                            horiz_norm = float(np.linalg.norm(horiz_dir))
                            if horiz_norm > 1e-6:
                                horiz_dir = horiz_dir / horiz_norm
                            else:
                                horiz_dir = np.array(used_approach_dir, dtype=float)
                            if (
                                used_grasp_variant is not None
                                and used_grasp_variant[0] == "published_roll"
                            ):
                                self.get_logger().warn(
                                    "FINAL_APPROACH_TOOL_FINISH_BASE_FOR_PUBLISHED_ROLL: "
                                    "using BASE relative line; TOOL +Z returned "
                                    "success/no-motion in this branch")
                            tool_finish_ok = self.execute_base_relative_line(
                                remaining_tool_line_m * horiz_dir,
                                "FINAL_APPROACH_TOOL_FINISH",
                                vel_mm_s=FINAL_APPROACH_VEL_MM_S,
                                acc_mm_s2=FINAL_APPROACH_ACC_MM_S2,
                            )
                            tool_finish_delta = remaining_tool_line_m * horiz_dir
                            if tool_finish_ok:
                                tool_finish_executed_m = remaining_tool_line_m
                                tool_finish_executed_dir = horiz_dir
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
                        if not tool_finish_ok:
                            self.get_logger().error(
                                "FINAL_APPROACH_TOOL_FINISH failed after "
                                "precomputed cuRobo final approach")
                            approach_ok = False
                        else:
                            final_approach_distance = (
                                selected_curobo_depth_m + remaining_tool_line_m)
                            used_grasp_ee_pos = (
                                used_grasp_ee_pos + tool_finish_delta)
                            self.runtime_log.log(
                                "final_approach_tool_finish_success",
                                executed_total_m=final_approach_distance,
                                horizontal_only=tool_finish_executed_dir is not None,
                            )
                else:
                    self.get_logger().warn(
                        "FINAL_APPROACH_PRECOMPUTED_CUROBO failed; "
                        "falling back to depth search")
            elif self._measured_tcp_model:
                approach_ok = self.execute_base_relative_line(
                    final_approach_distance * used_approach_dir,
                    "FINAL_APPROACH_STRAIGHT_BASE",
                    vel_mm_s=FINAL_APPROACH_VEL_MM_S,
                    acc_mm_s2=FINAL_APPROACH_ACC_MM_S2,
                )
            else:
                approach_ok = self.execute_tool_z_line(
                    final_approach_distance,
                    min_distance_m=0.005)
            if not approach_ok:
                fallback_ok = False
                if (
                    self._measured_tcp_model
                    and ENABLE_CUROBO_FINAL_APPROACH_FALLBACK
                    and not precomputed_final_attempted
                    and used_pre_ee_pos is not None
                    and used_grasp_quat is not None
                    and self.current_joints is not None
                ):
                    depth_candidates = [final_approach_distance]
                    if self._direct_curobo_final_approach_for_measured_tcp:
                        for depth_m in [0.130, 0.110, 0.090, 0.070, 0.060]:
                            if 0.001 < depth_m < final_approach_distance - 0.005:
                                depth_candidates.append(depth_m)
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
                            final_approach_distance = depth_m
                            used_grasp_ee_pos = fallback_target.copy()
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
                                self.get_logger().warn(
                                    "FINAL_APPROACH_TOOL_FINISH: cuRobo reached "
                                    f"{depth_m*1000:.0f}mm only; executing remaining "
                                    f"{remaining_tool_line_m*1000:.0f}mm with TOOL +Z "
                                    "MoveLine like the proven SW baseline")
                                self.runtime_log.log(
                                    "final_approach_tool_finish_requested",
                                    reason="curobo_deep_final_approach_ik_fail",
                                    curobo_depth_m=depth_m,
                                    tool_finish_m=remaining_tool_line_m,
                                    requested_total_m=requested_final_approach_distance,
                                )
                                # see horizontal-only rationale at the other
                                # FINAL_APPROACH_TOOL_FINISH call site above —
                                # gated on actual tilt, not target height.
                                if (
                                    used_grasp_variant is not None
                                    and used_grasp_variant[0] == "published_roll"
                                ) or abs(float(used_approach_dir[2])) > 1e-3:
                                    horiz_dir = np.array(used_approach_dir, dtype=float)
                                    horiz_dir[2] = 0.0
                                    horiz_norm = float(np.linalg.norm(horiz_dir))
                                    if horiz_norm > 1e-6:
                                        horiz_dir = horiz_dir / horiz_norm
                                    else:
                                        horiz_dir = np.array(used_approach_dir, dtype=float)
                                    if (
                                        used_grasp_variant is not None
                                        and used_grasp_variant[0] == "published_roll"
                                    ):
                                        self.get_logger().warn(
                                            "FINAL_APPROACH_TOOL_FINISH_BASE_FOR_PUBLISHED_ROLL: "
                                            "using BASE relative line; TOOL +Z returned "
                                            "success/no-motion in this branch")
                                    tool_finish_ok = self.execute_base_relative_line(
                                        remaining_tool_line_m * horiz_dir,
                                        "FINAL_APPROACH_TOOL_FINISH",
                                        vel_mm_s=FINAL_APPROACH_VEL_MM_S,
                                        acc_mm_s2=FINAL_APPROACH_ACC_MM_S2,
                                    )
                                    tool_finish_delta = remaining_tool_line_m * horiz_dir
                                    if tool_finish_ok:
                                        tool_finish_executed_m = remaining_tool_line_m
                                        tool_finish_executed_dir = horiz_dir
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
                                if not tool_finish_ok:
                                    self.get_logger().error(
                                        "FINAL_APPROACH_TOOL_FINISH failed after "
                                        "cuRobo shallow fallback")
                                    fallback_ok = False
                                    break
                                final_approach_distance = (
                                    depth_m + remaining_tool_line_m)
                                used_grasp_ee_pos = (
                                    used_grasp_ee_pos + tool_finish_delta)
                                self.runtime_log.log(
                                    "final_approach_tool_finish_success",
                                    executed_total_m=final_approach_distance,
                                    horizontal_only=tool_finish_executed_dir is not None,
                                )
                            break
                if not fallback_ok:
                    self.get_logger().error("ABORT: 직선 진입 실패")
                    self._clear_neighbor_obstacles()
                    self._reset_gripper()
                    self.pick_complete_pub.publish(Empty())
                    return

        # 실기 확인: 모든 벽면 딸기 줄기는 모델 벽 앞면보다 ~30mm 안쪽에 위치.
        # wall_margin=-30mm이면 available = offset+30mm → 80mm extra 자동 실행.
        # rightmost(x>250mm)는 offsets[-0.03, 0.0]으로 이미 깊게 진입하므로 제외.
        extra_advance_m = 0.0
        if (
            raw_straw[0] <= 0.25
            and self._leftmost_extra_advance_request_m > 0.0
        ):
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
                extra_advance_m = 0.0
            else:
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
                    return
                used_grasp_ee_pos = (
                    used_grasp_ee_pos + extra_advance_m * used_approach_dir)

        grasp_joints = (
            list(self.current_joints)
            if self.current_joints is not None
            else ret_grasp[0][-1].tolist()
        )
        self.get_logger().info(
            f"GRASP_POSE_REACHED — offset={used_grasp_offset:+.3f}m "
            f"pre={PRE_APPROACH_OFFSET*100:.0f}cm+{final_approach_distance*1000:.0f}mm+{extra_advance_m*1000:.0f}mm "
            f"variant={used_grasp_variant} elevation={np.degrees(np.arcsin(np.clip(used_approach_dir[2], -1.0, 1.0))):+.1f}deg "
            f"(attempt {grasp_attempt}/{n_offsets * n_quats})")
        self.runtime_log.log(
            "grasp_pose_reached",
            grasp_offset_m=used_grasp_offset,
            grasp_variant=used_grasp_variant,
            approach_dir=used_approach_dir,
            extra_advance_m=extra_advance_m,
            current_joints_rad=self.current_joints,
        )

        # 수평 진입 완료 후 열린 그리퍼로 줄기를 따라 KP1까지 하강한다.
        if self._measured_tcp_model and crane_z_offset_m > 0:
            if used_grasp_ee_pos is not None:
                # 2026-06-20: 고정 mm 보정 대신, 실제 도달한 그리퍼 Z와 목표
                # KP1 Z의 차이를 그대로 하강 거리로 쓴다. +15deg variant처럼
                # 접근 중 Z가 같이 올라가는 경우에도 tilt 각도와 무관하게
                # KP1에 정확히 도달한다. 원래 is_nw_high_target에만 적용했는데,
                # z<750mm 타겟도 같은 틸트 variant를 고르면 똑같이 어긋나는 게
                # 확인돼서 모든 measured_tcp 타겟에 적용한다 — 틸트가 0이면
                # overshoot이 정확히 기존 crane_z_offset_m과 같아져서(아래 식)
                # SW처럼 평평한 접근은 결과가 전혀 안 바뀐다.
                target_kp1_z_m = float(straw[2])
                reached_z_m = float(used_grasp_ee_pos[2])
                overshoot_above_kp1_m = max(0.0, reached_z_m - target_kp1_z_m)
                open_stem_descent_m = (
                    overshoot_above_kp1_m
                    + self._nw_high_target_descent_extra_below_kp1_m
                )
                self.get_logger().warn(
                    "OPEN_DESCENT_DYNAMIC: kp1_z="
                    f"{target_kp1_z_m*1000:.0f}mm reached_z={reached_z_m*1000:.0f}mm "
                    f"overshoot={overshoot_above_kp1_m*1000:.0f}mm "
                    f"extra_below_kp1={self._nw_high_target_descent_extra_below_kp1_m*1000:.0f}mm "
                    f"-> descent={open_stem_descent_m*1000:.0f}mm")
                self.runtime_log.log(
                    "nw_high_target_open_descent_dynamic",
                    target_kp1_z_m=target_kp1_z_m,
                    reached_z_m=reached_z_m,
                    overshoot_above_kp1_m=overshoot_above_kp1_m,
                    extra_below_kp1_m=self._nw_high_target_descent_extra_below_kp1_m,
                    executed_descent_m=open_stem_descent_m,
                    selected_variant=used_grasp_variant,
                )
            else:
                open_stem_descent_m = crane_z_offset_m
            self.get_logger().info(
                f"OPEN_STEM_DESCENT — gripper={GRIPPER_APPROACH_POS}, "
                f"BASE -Z {open_stem_descent_m*1000:.0f}mm to KP1")
            if not self.execute_base_z_relative(
                    -open_stem_descent_m, "OPEN_STEM_DESCENT", CRANE_DESCENT_VEL_MM_S):
                self.get_logger().error("ABORT: open stem descent 실패")
                self._clear_neighbor_obstacles()
                self._reset_gripper()
                self.pick_complete_pub.publish(Empty())
                return

        if is_nw_high_target and self._nw_high_target_base_y_nudge_m > 0.0:
            self.get_logger().warn(
                "NW_HIGH_TARGET_BASE_Y_NUDGE: BASE +Y "
                f"{self._nw_high_target_base_y_nudge_m*1000:.0f}mm before close "
                "(pure depth correction after height alignment)")
            self.runtime_log.log(
                "nw_high_target_base_y_nudge",
                target_z_m=float(raw_straw[2]),
                base_y_nudge_m=self._nw_high_target_base_y_nudge_m,
            )
            if not self.execute_base_relative_line(
                    [0.0, self._nw_high_target_base_y_nudge_m, 0.0],
                    "NW_HIGH_TARGET_BASE_Y_NUDGE",
                    CRANE_DESCENT_VEL_MM_S,
                    FINAL_APPROACH_ACC_MM_S2):
                self.get_logger().error(
                    "ABORT: NW high target BASE +Y nudge failed")
                self._clear_neighbor_obstacles()
                self._reset_gripper()
                self.pick_complete_pub.publish(Empty())
                return

        # 3. 그리퍼 닫기 + 파지 확인
        # SafeGrasp action 서버가 있으면 close+current 감지를 원자 동작으로 수행.
        # 없으면 SetPosition+GetState fallback.
        self.get_logger().info("3 close gripper + verify grasp")
        self.runtime_log.log("gripper_command", command="close")
        grasp_result, present_pos, present_current_raw, grasp_reason = (
            self._close_and_verify_grasp())
        if grasp_result == "GRIPPER_CLOSE_FAILED":
            self.get_logger().error(
                "ABORT: gripper close failed twice — skip detach and retreat straight")
            self.runtime_log.log(
                "pick_sequence_stopped",
                result_code="GRIPPER_CLOSE_FAILED",
                action="straight_retreat_without_detach",
            )
            retreat_distance_m = (
                final_approach_distance + extra_advance_m - tool_finish_executed_m)
            if self._measured_tcp_model:
                # 2026-06-20: tool_finish 다리가 틸트가 아닌 horiz_dir로 꺾여
                # 들어갔으면 그 구간은 따로 되돌린다 — 안 그러면 retreat 전체를
                # used_approach_dir(틸트)로만 되돌리면서 실제로 안 내려간
                # 구간까지 Z를 더 내려버려 팔이 과도하게 펴진다(J2 한도 초과
                # 실기로 확인). 틸트가 없으면 tool_finish_executed_m=0이라
                # 기존과 동일하게 한 번에 되돌린다.
                retreat_ok = True
                if tool_finish_executed_m > 0.0 and tool_finish_executed_dir is not None:
                    retreat_ok = self.execute_base_relative_line(
                        -tool_finish_executed_m * tool_finish_executed_dir,
                        "CLOSE_FAIL_RETREAT_TOOL_FINISH_UNDO",
                        vel_mm_s=RETREAT_VEL_MM_S,
                        acc_mm_s2=RETREAT_ACC_MM_S2,
                    )
                if retreat_ok and retreat_distance_m > 0.0:
                    retreat_ok = self.execute_base_relative_line(
                        -retreat_distance_m * used_approach_dir,
                        "CLOSE_FAIL_RETREAT_BASE",
                        vel_mm_s=RETREAT_VEL_MM_S,
                        acc_mm_s2=RETREAT_ACC_MM_S2,
                    )
            else:
                retreat_ok = self.execute_tool_z_line(
                    -retreat_distance_m,
                    motion_label="CLOSE_FAIL_RETREAT",
                    vel_mm_s=RETREAT_VEL_MM_S,
                    acc_mm_s2=RETREAT_ACC_MM_S2,
                )
            self._clear_neighbor_obstacles()
            if retreat_ok:
                self._reset_gripper()
                self.pick_complete_pub.publish(Empty())
            else:
                self._hold_pick_sequence("gripper_close_failed_retreat_failed")
            return
        self.get_logger().info(
            f"VERIFY_GRASP: {grasp_result} present_pos={present_pos} "
            f"current_raw={present_current_raw} — {grasp_reason}")
        self.runtime_log.log(
            "verify_grasp",
            result_code=grasp_result,
            present_position=present_pos,
            present_current_raw=present_current_raw,
            reason=grasp_reason,
            close_command_pos=700,
            empty_threshold=GRASP_EMPTY_POSITION_THRESHOLD,
            current_contact_threshold_raw=self._grasp_current_contact_threshold_raw,
        )

        # 4. BASE -Z 당기기로 줄기 분리 후 직선 역진 retreat
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
        reverse_ok = True
        if self._measured_tcp_model:
            # 2026-06-20: 위 FINAL_APPROACH_TOOL_FINISH가 horiz_dir로 꺾여
            # 들어간 구간이 있으면 그 다리를 먼저 따로 되돌린 다음, 남은
            # 직선(curobo 진입 + extra_advance, 둘 다 used_approach_dir 방향)을
            # 되돌린다. 한 번에 used_approach_dir로만 되돌리면 horiz 구간의
            # 거리만큼 실제로 안 내려간 Z까지 같이 내려가버려 팔이 과신전되고
            # J2가 한도(±95°)를 넘는 실기 사고가 있었음(-97.55°/-97.7° 확인).
            # 틸트가 없으면 tool_finish_executed_m=0이라 기존과 동일하게
            # 한 번의 직선으로 되돌아간다(SW no-op).
            if tool_finish_executed_m > 0.0 and tool_finish_executed_dir is not None:
                reverse_ok = self.execute_base_relative_line(
                    -tool_finish_executed_m * tool_finish_executed_dir,
                    "RETREAT_TOOL_FINISH_UNDO",
                    vel_mm_s=RETREAT_VEL_MM_S,
                    acc_mm_s2=RETREAT_ACC_MM_S2,
                )
            if reverse_ok and reverse_distance_m > 0.0:
                reverse_ok = self.execute_base_relative_line(
                    -reverse_distance_m * used_approach_dir,
                    "RETREAT_BASE",
                    vel_mm_s=RETREAT_VEL_MM_S,
                    acc_mm_s2=RETREAT_ACC_MM_S2,
                )
        elif reverse_distance_m > 0.0:
            reverse_ok = self.execute_tool_z_line(
                -reverse_distance_m,
                motion_label="RETREAT",
                vel_mm_s=RETREAT_VEL_MM_S,
                acc_mm_s2=RETREAT_ACC_MM_S2,
            )
        if not reverse_ok:
            self.get_logger().error(
                "ABORT: straight reverse retreat failed — holding current pose")
            self._clear_neighbor_obstacles()
            self._hold_pick_sequence("straight_reverse_retreat_failed")
            return

        time.sleep(STRAIGHT_RETREAT_SETTLE_SEC)
        retreat_joints = (
            list(self.current_joints)
            if self.current_joints is not None
            else grasp_joints
        )

        # 4b. VERIFY_DETACH
        detach_result = "DETACH_UNVERIFIED"
        self.runtime_log.log(
            "verify_detach",
            result_code=detach_result,
            grasp_result=grasp_result,
            retreat_policy="pitch_detach_then_straight_reverse",
            reason="no sensor; pitch detach executed",
        )

        # Place 게이트 기본값은 fail-closed다. 센서 판독이 불가능한 실험에서만
        # allow_unverified_grasp_place를 명시적으로 켜고 사람 관찰 라벨을 남긴다.
        _allow_place = (
            grasp_result == "GRASP_CONTACT_DETECTED"
            or (
                grasp_result == "GRASP_UNVERIFIED"
                and self._allow_unverified_grasp_place
            )
        )
        if not _allow_place:
            place_block_reason = (
                "GRASP_EMPTY: jaw fully closed, nothing grabbed"
                if grasp_result == "GRASP_EMPTY"
                else "GRASP_UNVERIFIED: enable explicit override only after visual check"
            )
            self.get_logger().warn(
                f"PLACE_GATE_BLOCKED ({grasp_result}): {place_block_reason}")
            self.runtime_log.log(
                "place_gate_blocked",
                grasp_result=grasp_result,
                reason=place_block_reason,
            )

        return_start_joints = retreat_joints
        if self._enable_marker_place and _allow_place:
            place_status, place_joints = self._execute_marker_place_after_retreat(
                retreat_joints)
            if place_status == "success":
                return_start_joints = place_joints
                if (
                    self._use_taught_slot0_place_reference
                    and self._hold_after_taught_slot0_place
                ):
                    completed_slot_index = self._marker_place_slot_idx - 1
                    self._clear_neighbor_obstacles()
                    self.runtime_log.log(
                        "pick_sequence_stopped",
                        result_code="TAUGHT_TRAY_PLACE_COMPLETE_HOLD",
                        slot_index=completed_slot_index,
                        current_joints_rad=self.current_joints,
                    )
                    self.get_logger().warn(
                        f"TAUGHT_TRAY_SLOT{completed_slot_index}_PLACE_COMPLETE_HOLD: "
                        "release complete; "
                        "automatic next pick blocked until planner restart")
                    self._hold_pick_sequence("taught_tray_place_complete")
                    return
            elif place_status == "tray_complete":
                self._clear_neighbor_obstacles()
                self.runtime_log.log(
                    "pick_sequence_stopped",
                    result_code="TAUGHT_TRAY_FULL",
                    current_joints_rad=self.current_joints,
                )
                self.get_logger().warn(
                    "TAUGHT_TRAY_FULL: all 15 slots consumed; "
                    "automatic next pick blocked until tray reset")
                self._hold_pick_sequence("taught_tray_full")
                return
            elif place_status == "skip":
                # tray 없음/stale — place 생략, scan 복귀
                self.get_logger().warn("PLACE_SKIPPED: tray unavailable; returning to scan")
                self.runtime_log.log("place_skipped", reason="tray_unavailable",
                                     grasp_result=grasp_result)
            else:
                # 로봇이 이미 움직인 뒤 실패 or preview hold → latch
                self._clear_neighbor_obstacles()
                self.runtime_log.log(
                    "pick_sequence_stopped",
                    result_code=(
                        "MARKER_PLACE_PREVIEW_HOLD"
                        if place_status == "preview_hold"
                        else "MARKER_PLACE_FAILED"
                    ),
                    place_status=place_status,
                    current_joints_rad=self.current_joints,
                )
                self.get_logger().warn(
                    f"PICK_SEQUENCE_HOLD place_status={place_status}; "
                    "pick_complete not published, automatic scan paused")
                self._hold_pick_sequence(f"marker_place_{place_status}")
                return

        self.get_logger().info("7 return to pick-start scan pose")
        # 직선 retreat 또는 marker place 완료 후 이번 pick이 시작된 scan pose로
        # 복귀한다. scan_executor는 같은 SW 셀의 다음 target을 이어서 전달한다.
        pick_start_joints_deg = np.rad2deg(pick_start_joints).tolist()
        pick_start_joints_deg = self._nearest_equivalent_joints(pick_start_joints_deg)
        ok, _ = self.plan_to_fixed_joints_pose(
            return_start_joints, pick_start_joints_deg, "pick-start scan pose after pick/place",
            skip_swing_check=True)
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
            return

        self._clear_neighbor_obstacles()
        self._reset_gripper()  # 다음 파지를 위해 approach 위치(600)로 복귀
        self.pick_complete_pub.publish(Empty())
        sequence_result_code = (
            "DETACH_SUCCESS_UNVERIFIED"
            if grasp_result == "GRASP_CONTACT_DETECTED"
            else grasp_result   # GRASP_EMPTY or GRASP_UNVERIFIED
        )
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
