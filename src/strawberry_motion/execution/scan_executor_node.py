"""Fail-closed workspace scan executor for validated scan candidates.

Cell state published to /strawberry/exploration/set_cell_state:
  SCANNING      while robot is moving to the cell
  SCAN_POSE_REACHED after dwell; perception has not classified the cell yet
  PLANNING_FAIL if cuRobo or execution fails

This node never starts from joint-state arrival. Motion requires all of:
  - launch/parameter opt-in: execute_motion:=true
  - YAML flags: use_for_automated_motion=true AND collision_world_validated_for_motion=true,
    or manual_validation_mode:=true for one explicitly selected single cell
  - an explicit /strawberry/scan/start Trigger request
  - a live joint state matching the manually verified overview pose
  - one explicitly selected initial-validation cell (root/nw/root/ne/root/se/root/sw)

The collision backend uses the validated scene (RUN-20260527-012):
  robot/tool collision spheres + registered whiteboard cuboid + self_collision.
Motion remains blocked by use_for_automated_motion in the candidates YAML.
To authorize: set use_for_automated_motion=true after physical E-stop verification.

Run:
  ros2 launch strawberry_motion workspace_scan.launch.py  # preview only
Status monitoring:
  ros2 topic echo /strawberry/scan/status
"""

import threading
import time
from copy import deepcopy
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import torch
import yaml
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
import rclpy.callback_groups
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger

from ament_index_python.packages import get_package_share_directory
from dsr_msgs2.srv import MoveJoint, MoveSplineJoint

from curobo.geom.types import Cuboid, WorldConfig
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.robot import JointState as CuroboJointState, RobotConfig
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig

from strawberry_motion.execution.scan_safety import (
    joints_within_tolerance_deg,
    motion_start_allowed,
    single_cell_request_allowed,
)

_CUROBO_DIR = Path("/home/user/doosan_ws/src/e0509_gripper_description/config/curobo")
_URDF_PATH = _CUROBO_DIR / "e0509_gripper.urdf"
_ROBOT_YML = _CUROBO_DIR / "e0509_gripper.yml"
_SPHERES_PATH = _CUROBO_DIR / "e0509_spheres.yml"
_CANDIDATES_FNAME = "scan_pose_candidates_refit_candidate.yaml"
_COLLISION_WORLD_FNAME = "scan_collision_world.yaml"
_JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

# Single-cell test gate — all 4 cells validated (RUN-20260527-012)
_INITIAL_SINGLE_CELL_CANDIDATES = ["root/nw", "root/ne", "root/se", "root/sw"]
# Full traversal Z-order (nw→ne top row, sw→se bottom row)
_ALL_CELLS_ZORDER = ["root/nw", "root/ne", "root/se", "root/sw"]

_MAX_SPLINE_PTS = 12
_SPLINE_TIME_SCALE = 0.75
_SPLINE_MIN_TIME = 0.5
_SCAN_DWELL_SEC = 1.5
_OVERVIEW_TOLERANCE_DEG = 1.0
_DEFAULT_SCAN_MOVEJ_VEL_DEG_S = 80.0
_DEFAULT_SCAN_MOVEJ_ACC_DEG_S2 = 100.0
_DEFAULT_OVERVIEW_RETURN_VEL_DEG_S = 80.0
_DEFAULT_OVERVIEW_RETURN_ACC_DEG_S2 = 100.0
_DEFAULT_MOVEJ_SERVICE_TIMEOUT_SEC = 30.0
# True: _init_motion_gen loads robot spheres + whiteboard cuboid + self-collision
# (validated in RUN-20260527-012). Motion is still gated by use_for_automated_motion
# in the candidates YAML, which the operator sets after physical E-stop verification.
_COLLISION_BACKEND_READY_FOR_MOTION = True

_OP_LIMITS_DEG = [
    (-225.0, 225.0),
    (-95.0,   95.0),
    (-155.0, 155.0),
    (-170.0, 170.0),
    (-130.0, 130.0),
    (-225.0, 225.0),
]

_JOINT_LIMITS_RAD = [
    (-6.273185, 6.273185),
    (-1.648063, 1.648063),
    (-2.6953,   2.6953  ),
    (-6.273185, 6.273185),
    (-2.346194, 2.346194),
    (-6.273185, 6.273185),
]


def _mat4_to_pos_quat_wxyz(mat4: np.ndarray) -> Tuple[List[float], List[float]]:
    pos = mat4[:3, 3].tolist()
    q_xyzw = Rotation.from_matrix(mat4[:3, :3]).as_quat()
    q_wxyz = [float(q_xyzw[3]), float(q_xyzw[0]), float(q_xyzw[1]), float(q_xyzw[2])]
    return pos, q_wxyz


class ScanExecutorNode(Node):

    def __init__(self) -> None:
        super().__init__("scan_executor_node")

        self._current_joints: Optional[List[float]] = None
        self._started = False
        self._mg: Optional[MotionGen] = None
        self._detection_count: int = 0
        self._detection_lock = threading.Lock()
        self.declare_parameter("execute_motion", False)
        self.declare_parameter("target_cell", "")
        self.declare_parameter("manual_validation_mode", False)
        self.declare_parameter("scan_movej_vel_deg_s", _DEFAULT_SCAN_MOVEJ_VEL_DEG_S)
        self.declare_parameter("scan_movej_acc_deg_s2", _DEFAULT_SCAN_MOVEJ_ACC_DEG_S2)
        self.declare_parameter(
            "overview_return_vel_deg_s", _DEFAULT_OVERVIEW_RETURN_VEL_DEG_S
        )
        self.declare_parameter(
            "overview_return_acc_deg_s2", _DEFAULT_OVERVIEW_RETURN_ACC_DEG_S2
        )
        self.declare_parameter(
            "movej_service_timeout_sec", _DEFAULT_MOVEJ_SERVICE_TIMEOUT_SEC
        )
        self._execute_motion = bool(self.get_parameter("execute_motion").value)
        self._target_cell = str(self.get_parameter("target_cell").value)
        self._manual_validation_mode = bool(
            self.get_parameter("manual_validation_mode").value
        )
        self._scan_movej_vel = float(self.get_parameter("scan_movej_vel_deg_s").value)
        self._scan_movej_acc = float(self.get_parameter("scan_movej_acc_deg_s2").value)
        self._overview_return_vel = float(
            self.get_parameter("overview_return_vel_deg_s").value
        )
        self._overview_return_acc = float(
            self.get_parameter("overview_return_acc_deg_s2").value
        )
        self._movej_service_timeout_sec = float(
            self.get_parameter("movej_service_timeout_sec").value
        )

        pkg = get_package_share_directory("strawberry_motion")
        candidates_path = Path(pkg) / "config" / _CANDIDATES_FNAME
        with candidates_path.open() as fh:
            data = yaml.safe_load(fh)
        candidate_cfg = data["scan_pose_candidates"]
        self._overview_joints_deg = [
            float(v) for v in candidate_cfg.get("curobo_start_joints_deg", [])
        ]
        if len(self._overview_joints_deg) != 6:
            raise RuntimeError(
                "%s missing valid curobo_start_joints_deg" % _CANDIDATES_FNAME
            )
        self._candidate_authorized = bool(
            candidate_cfg.get("use_for_automated_motion", False)
            and candidate_cfg.get("collision_world_validated_for_motion", False)
            and _COLLISION_BACKEND_READY_FOR_MOTION
        )
        self._targets: Dict[str, dict] = {
            t["cell_id"]: t
            for t in candidate_cfg["targets"]
            if t.get("tcp_transform_base") is not None
        }
        self.get_logger().info(
            "Loaded %d scan targets from %s" % (len(self._targets), candidates_path)
        )

        if not self._candidate_authorized:
            self.get_logger().warn(
                "Motion locked: set use_for_automated_motion=true in %s "
                "after physical E-stop verification." % _CANDIDATES_FNAME
            )
        if self._manual_validation_mode:
            self.get_logger().warn(
                "manual_validation_mode=true: single-cell MoveJoint validation is "
                "allowed, but target_cell=all remains blocked unless YAML is authorized."
            )
        self.get_logger().info(
            "MoveJoint speeds: scan vel=%.1f acc=%.1f, overview return vel=%.1f acc=%.1f"
            % (
                self._scan_movej_vel,
                self._scan_movej_acc,
                self._overview_return_vel,
                self._overview_return_acc,
            )
        )
        self.get_logger().info(
            "MoveJoint service dispatch timeout: %.1fs; arrival is verified from /joint_states"
            % self._movej_service_timeout_sec
        )

        cb = rclpy.callback_groups.ReentrantCallbackGroup()
        self.create_subscription(JointState, "/dsr01/joint_states", self._joint_cb, 10)
        self.create_subscription(
            PoseStamped, "/dsr01/curobo/pick_pose", self._pick_cb, 10
        )
        self._state_pub = self.create_publisher(
            String, "/strawberry/exploration/set_cell_state", 10
        )
        self._status_pub = self.create_publisher(String, "/strawberry/scan/status", 10)
        self.create_service(Trigger, "/strawberry/scan/start", self._start_cb, callback_group=cb)
        self._cli_spline = self.create_client(
            MoveSplineJoint, "/dsr01/motion/move_spline_joint", callback_group=cb
        )
        self._cli_movej = self.create_client(
            MoveJoint, "/dsr01/motion/move_joint", callback_group=cb
        )

        self.get_logger().info(
            "scan_executor_node ready; explicit /strawberry/scan/start required"
        )

    def _init_motion_gen(self) -> None:
        if self._mg is not None:
            return
        self.get_logger().info(
            "Initialising cuRobo MotionGen (spheres + whiteboard + self-collision)"
        )
        tensor_args = TensorDeviceType(device=torch.device("cuda:0"))

        with _ROBOT_YML.open() as fh:
            robot_data = deepcopy(yaml.safe_load(fh))
        kine = robot_data["robot_cfg"]["kinematics"]
        kine["urdf_path"] = str(_URDF_PATH)
        kine["collision_spheres"] = str(_SPHERES_PATH)
        robot_cfg = RobotConfig.from_dict(robot_data, tensor_args=tensor_args)

        pkg = get_package_share_directory("strawberry_motion")
        world_yaml = Path(pkg) / "config" / _COLLISION_WORLD_FNAME
        with world_yaml.open() as fh:
            world_meta = yaml.safe_load(fh)["scan_collision_world"]
        cuboids = [
            Cuboid(
                name=o["name"],
                pose=[float(v) for v in o["pose_wxyz"]],
                dims=[float(v) for v in o["dims_m"]],
            )
            for o in world_meta["objects"]
            if o.get("enabled", True) and o.get("type") == "cuboid"
        ]
        world_cfg = WorldConfig(cuboid=cuboids)

        mg_cfg = MotionGenConfig.load_from_robot_config(
            robot_cfg, world_cfg, tensor_args=tensor_args,
            num_trajopt_seeds=16, num_graph_seeds=16,
            collision_cache={"obb": 30, "mesh": 10},
            use_cuda_graph=False,
            self_collision_check=True,
            self_collision_opt=True,
        )
        self._mg = MotionGen(mg_cfg)
        self._mg.warmup(warmup_js_trajopt=False)
        self._mg.detach_object_from_robot()
        self.get_logger().info("cuRobo MotionGen ready")

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _joint_cb(self, msg: JointState) -> None:
        jmap = {n: p for n, p in zip(msg.name, msg.position)}
        joints = [jmap.get(n) for n in _JOINT_NAMES]
        if None not in joints:
            self._current_joints = joints

    def _pick_cb(self, _msg: PoseStamped) -> None:
        with self._detection_lock:
            self._detection_count += 1

    def _start_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        allowed, reason = motion_start_allowed(
            execute_motion=self._execute_motion,
            candidate_authorized=self._candidate_authorized,
            has_joint_state=self._current_joints is not None,
            manual_validation_mode=(
                self._manual_validation_mode and self._target_cell != "all"
            ),
        )
        if self._started:
            allowed, reason = False, "scan already started"
        if allowed:
            if self._target_cell == "all":
                # 4-cell traversal mode: bypasses single-cell gate
                reason = "traversal mode all cells accepted"
            else:
                allowed, reason = single_cell_request_allowed(
                    self._target_cell, _INITIAL_SINGLE_CELL_CANDIDATES
                )
        if allowed and not joints_within_tolerance_deg(
            self._current_joints or [], self._overview_joints_deg, _OVERVIEW_TOLERANCE_DEG
        ):
            allowed = False
            reason = "current joints do not match verified overview pose within 1.0 deg"
        response.success = allowed
        response.message = reason
        if not allowed:
            self._pub_status("START_REJECTED " + reason)
            return response
        self._started = True
        self._pub_status("START_ACCEPTED explicit request; initial pose verified")
        threading.Thread(target=self._scan_sequence, daemon=True).start()
        return response

    # ── helpers ───────────────────────────────────────────────────────────────

    def _pub_state(self, cell_id: str, state: str) -> None:
        msg = String()
        msg.data = "%s=%s" % (cell_id, state)
        self._state_pub.publish(msg)

    def _pub_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)
        self.get_logger().info(text)

    def _traj_ok(self, traj: np.ndarray, label: str) -> bool:
        deg = np.rad2deg(traj)
        for i, (lo, hi) in enumerate(_OP_LIMITS_DEG):
            vmin, vmax = float(np.min(deg[:, i])), float(np.max(deg[:, i]))
            if vmin < lo or vmax > hi:
                self.get_logger().warn(
                    "%s J%d %.1f~%.1f° outside op limits %.1f~%.1f°"
                    % (label, i + 1, vmin, vmax, lo, hi)
                )
                return False
        return True

    def _plan(
        self, start_joints: List[float], pos: List[float], quat_wxyz: List[float], label: str,
        max_retries: int = 5,
    ) -> Optional[Tuple[np.ndarray, float]]:
        start = CuroboJointState.from_position(
            position=torch.tensor(
                [start_joints], device="cuda:0", dtype=torch.float32
            ),
            joint_names=_JOINT_NAMES,
        )
        goal = Pose(
            position=torch.tensor([pos], device="cuda:0", dtype=torch.float32),
            quaternion=torch.tensor([quat_wxyz], device="cuda:0", dtype=torch.float32),
        )
        if self._mg is None:
            self.get_logger().error("MotionGen unavailable")
            return None
        for attempt in range(max_retries):
            result = self._mg.plan_single(
                start, goal, MotionGenPlanConfig(enable_graph=True, max_attempts=4)
            )
            if not result.success.item():
                self.get_logger().warn(
                    "%s plan attempt %d/%d failed: %s"
                    % (label, attempt + 1, max_retries, getattr(result, "status", "?"))
                )
                continue
            traj = result.get_interpolated_plan().position.cpu().numpy()
            if not self._traj_ok(traj, label):
                self.get_logger().warn(
                    "%s traj limits violated on attempt %d/%d — retrying"
                    % (label, attempt + 1, max_retries)
                )
                continue
            endpoint_rad = traj[-1].tolist()
            endpoint_deg = [round(float(np.rad2deg(j)), 1) for j in endpoint_rad]
            motion_time = float(result.motion_time.item())
            self.get_logger().info(
                "%s plan endpoint_deg=[%s]  curobo_time=%.2fs  attempt=%d"
                % (label, " ".join("%.1f" % d for d in endpoint_deg), motion_time, attempt + 1)
            )
            return traj, motion_time, endpoint_rad
        self.get_logger().error("%s plan failed after %d attempts" % (label, max_retries))
        return None

    def _exec_spline(
        self, traj_rad: np.ndarray, vel: float = 120.0, min_time: float = 3.0
    ) -> bool:
        if not self._cli_spline.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("MoveSplineJoint service not available")
            return False
        deg = np.rad2deg(traj_rad)
        n = deg.shape[0]
        if n > _MAX_SPLINE_PTS:
            idx = np.linspace(0, n - 1, _MAX_SPLINE_PTS, dtype=int)
            deg = deg[idx]
            n = _MAX_SPLINE_PTS
        # Skip the first waypoint (current/start position).
        # MoveSplineJoint moves from the robot's current position through the
        # given via-points. Including the start as waypoint[0] causes a
        # near-zero first segment that Doosan silently rejects when the robot's
        # actual joints don't perfectly match the planned start.
        deg = deg[1:]
        n = len(deg)
        req = MoveSplineJoint.Request()
        req.pos_cnt = n
        for row in deg:
            pt = Float64MultiArray()
            pt.data = row.tolist()
            req.pos.append(pt)
        req.vel = [float(vel)] * 6
        req.acc = [float(vel) * 1.5] * 6
        # Compute minimum feasible time from actual trajectory arc length.
        # cuRobo plans aggressively (often < 1 s) but Doosan rejects if
        # req.time < max_joint_arc / vel.  Use 1.5x safety margin, 3 s min.
        path_lengths = np.sum(np.abs(np.diff(np.rad2deg(traj_rad), axis=0)), axis=0)
        req.time = float(max(np.max(path_lengths) / vel * 1.5, min_time))
        req.mode = 0
        req.sync_type = 0
        future = self._cli_spline.call_async(req)
        t0 = time.time()
        while not future.done() and (time.time() - t0) < 60.0:
            time.sleep(0.05)
        if not future.done():
            self.get_logger().error("Spline future timed out after 60s")
            return False
        resp = future.result()
        if resp is None:
            self.get_logger().error("Spline future result is None")
            return False
        self.get_logger().info(
            "Spline response: success=%s  msg=%r  pos_cnt=%d  req_time=%.2fs"
            % (resp.success, getattr(resp, "msg", "N/A"), n, req.time)
        )
        if not resp.success:
            self.get_logger().error("MoveSplineJoint returned success=False")
        return bool(resp.success)

    def _wait_for_joints(
        self, target_rad: List[float], tolerance_deg: float, timeout_sec: float
    ) -> bool:
        deadline = time.time() + timeout_sec
        target_deg = np.rad2deg(target_rad).tolist()
        while time.time() < deadline:
            if self._current_joints and joints_within_tolerance_deg(
                self._current_joints, target_deg, tolerance_deg
            ):
                return True
            time.sleep(0.1)
        return False

    def _movej(self, joints_deg: List[float], vel: float = 40.0, acc: float = 40.0) -> bool:
        if not self._cli_movej.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("MoveJoint service not available")
            return False
        req = MoveJoint.Request()
        req.pos = [float(v) for v in joints_deg]
        req.vel = vel
        req.acc = acc
        req.time = 0.0
        req.radius = 0.0
        req.mode = 0
        req.blend_type = 0
        req.sync_type = 0
        future = self._cli_movej.call_async(req)
        t0 = time.time()
        while not future.done() and (time.time() - t0) < self._movej_service_timeout_sec:
            time.sleep(0.05)
        if not future.done():
            self.get_logger().warn(
                "MoveJoint service response not returned within %.1fs; treating command as dispatched and verifying arrival from /joint_states"
                % self._movej_service_timeout_sec
            )
            return True
        ok = future.done() and future.result() and future.result().success
        self.get_logger().info(
            "MoveJoint response: success=%s  target=[%s]  vel=%.1f  acc=%.1f"
            % (
                ok,
                " ".join("%.1f" % v for v in joints_deg),
                vel,
                acc,
            )
        )
        if not ok:
            self.get_logger().error("MoveJoint failed")
        return bool(ok)

    def _is_at_overview(self) -> bool:
        return self._current_joints is not None and joints_within_tolerance_deg(
            self._current_joints, self._overview_joints_deg, _OVERVIEW_TOLERANCE_DEG
        )

    def _wait_at_overview(self, timeout_sec: float = 10.0) -> bool:
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            if self._is_at_overview():
                return True
            time.sleep(0.05)
        return False

    @staticmethod
    def _spline_vel_for_j1_swing(traj_rad: np.ndarray) -> float:
        """Return spline velocity (deg/s) scaled down for large J1 arcs.

        SE requires a 114° J1 swing (geometrically unavoidable at 0.50 m).
        Running at 120°/s through that arc looks violent. Cap at 60°/s when
        swing >= 90°, 80°/s when >= 60°, otherwise 120°/s.
        """
        j1 = np.rad2deg(traj_rad[:, 0])
        swing = float(np.max(j1) - np.min(j1))
        if swing >= 90.0:
            return 60.0
        if swing >= 60.0:
            return 80.0
        return 120.0

    # ── scan sequence (runs in background thread) ─────────────────────────────

    def _scan_sequence(self) -> None:
        if self._target_cell == "all":
            scan_order = [c for c in _ALL_CELLS_ZORDER if c in self._targets]
            self._pub_status("TRAVERSAL_SCAN_STARTED cells=%s" % scan_order)
        else:
            scan_order = [self._target_cell]
            self._pub_status("SINGLE_CELL_SCAN_STARTED target=%s" % self._target_cell)

        cell_detections: Dict[str, int] = {}

        for cell_id in scan_order:
            if cell_id not in self._targets:
                self.get_logger().warn("%s not in candidates — skipping" % cell_id)
                continue

            target = self._targets[cell_id]
            endpoint_deg = target.get("endpoint_joints_deg")
            if endpoint_deg is None:
                self._pub_status(
                    "CONFIG_ERROR %s missing endpoint_joints_deg in YAML — aborting" % cell_id
                )
                self._pub_state(cell_id, "PLANNING_FAIL")
                return

            endpoint_rad = [float(np.deg2rad(d)) for d in endpoint_deg]

            self._pub_state(cell_id, "SCANNING")
            self._pub_status(
                "MOVING_TO %s  endpoint_deg=[%s]  (direct MoveJoint, YAML pose)"
                % (cell_id, " ".join("%.1f" % d for d in endpoint_deg))
            )

            if not self._movej(
                endpoint_deg, vel=self._scan_movej_vel, acc=self._scan_movej_acc
            ):
                self._pub_status("EXEC_FAIL %s MoveJoint failed — aborting scan sequence" % cell_id)
                self._pub_state(cell_id, "PLANNING_FAIL")
                self._pub_status("RETURNING_TO_OVERVIEW after failure")
                self._movej(
                    self._overview_joints_deg,
                    vel=self._overview_return_vel,
                    acc=self._overview_return_acc,
                )
                return

            arrival_timeout = 90.0
            arrived = self._wait_for_joints(endpoint_rad, 3.0, arrival_timeout)
            if not arrived:
                self._pub_status(
                    "EXEC_TIMEOUT %s — robot did not arrive at endpoint within %.0fs; aborting"
                    % (cell_id, arrival_timeout)
                )
                self._pub_state(cell_id, "PLANNING_FAIL")
                self._movej(
                    self._overview_joints_deg,
                    vel=self._overview_return_vel,
                    acc=self._overview_return_acc,
                )
                return

            # Reset per-cell detection counter just before dwell so only
            # picks arriving while at this scan pose are counted.
            with self._detection_lock:
                self._detection_count = 0
            joints_now = self._current_joints or []
            joints_deg_str = " ".join("%.1f" % np.rad2deg(j) for j in joints_now)
            self._pub_status(
                "AT_SCAN_POSE %s joints_deg=[%s] — dwell %.1fs"
                % (cell_id, joints_deg_str, _SCAN_DWELL_SEC)
            )
            time.sleep(_SCAN_DWELL_SEC)

            with self._detection_lock:
                count = self._detection_count
            cell_detections[cell_id] = count

            if count > 0:
                self._pub_state(cell_id, "TARGET_FOUND")
                self._pub_status(
                    "TARGET_FOUND %s %d pick candidate(s) detected" % (cell_id, count)
                )
            else:
                self._pub_state(cell_id, "SCANNED_EMPTY")
                self._pub_status("SCANNED_EMPTY %s no detection in dwell window" % cell_id)

            # Return to overview between cells so every next-cell plan starts
            # from the validated overview state (prevents J6 wind-up).
            if cell_id != scan_order[-1]:
                self._pub_status("INTER_CELL_OVERVIEW_RESET")
                if not self._movej(
                    self._overview_joints_deg,
                    vel=self._overview_return_vel,
                    acc=self._overview_return_acc,
                ):
                    self._pub_status("ABORT inter-cell overview return failed")
                    return
                if not self._wait_at_overview(timeout_sec=45.0):
                    self._pub_status("ABORT overview not confirmed between cells")
                    return

        # Return to overview at the end of the full scan sequence.
        self._pub_status("RETURNING_TO_OVERVIEW")
        if not self._movej(
            self._overview_joints_deg,
            vel=self._overview_return_vel,
            acc=self._overview_return_acc,
        ):
            self._pub_status("ABORT overview return failed after scan sequence")
            return
        if not self._wait_at_overview():
            self._pub_status("ABORT overview pose was not confirmed after scan sequence")
            return
        joints_ov = self._current_joints or []
        joints_ov_str = " ".join("%.1f" % np.rad2deg(j) for j in joints_ov)
        self._pub_status("AT_OVERVIEW joints_deg=[%s]" % joints_ov_str)

        # Publish harvest priority order (most detections first).
        if cell_detections:
            harvest_order = sorted(
                cell_detections.items(), key=lambda x: x[1], reverse=True
            )
            order_str = "  ".join(
                "%s:%d" % (cid, cnt) for cid, cnt in harvest_order
            )
            self._pub_status("HARVEST_PRIORITY_ORDER %s" % order_str)

        self._pub_status("SCAN_COMPLETE")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanExecutorNode()
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
