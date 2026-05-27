"""Fail-closed workspace scan executor for validated scan candidates.

Cell state published to /strawberry/exploration/set_cell_state:
  SCANNING      while robot is moving to the cell
  SCAN_POSE_REACHED after dwell; perception has not classified the cell yet
  PLANNING_FAIL if cuRobo or execution fails

This node never starts from joint-state arrival. Motion requires all of:
  - launch/parameter opt-in: execute_motion:=true
  - config authorization and collision-world validation flags
  - an explicit /strawberry/scan/start Trigger request
  - a live joint state matching the manually verified overview pose

The current candidate config intentionally keeps authorization false.

Run:
  ros2 launch strawberry_motion workspace_scan.launch.py  # preview only
Status monitoring:
  ros2 topic echo /strawberry/scan/status
"""

import threading
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import torch
import yaml
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
import rclpy.callback_groups
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger

from ament_index_python.packages import get_package_share_directory
from dsr_msgs2.srv import MoveJoint, MoveSplineJoint

from curobo.geom.types import WorldConfig  # noqa: F401 – kept for WorldConfig(cuboid=[])
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.robot import JointState as CuroboJointState, RobotConfig
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig

from strawberry_motion.execution.scan_safety import (
    joints_within_tolerance_deg,
    motion_start_allowed,
)

_CUROBO_DIR = Path("/home/user/doosan_ws/src/e0509_gripper_description/config/curobo")
_URDF_PATH = _CUROBO_DIR / "e0509_gripper.urdf"
_ROBOT_YML = _CUROBO_DIR / "e0509_gripper.yml"
_ENVIRONMENT_YAML = Path(
    "/home/user/doosan_ws/src/e0509_gripper_description/config/environment.yaml"
)
_JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

# Z-order: left-to-right top row, then right-to-left bottom row
_SCAN_ORDER = ["root/nw", "root/ne", "root/se", "root/sw"]

_OVERVIEW_JOINTS_DEG = [97.84, -94.40, 65.95, -10.93, 95.49, -94.79]

_MAX_SPLINE_PTS = 12
_SPLINE_TIME_SCALE = 0.75
_SPLINE_MIN_TIME = 0.5
_SCAN_DWELL_SEC = 1.5
_OVERVIEW_TOLERANCE_DEG = 1.0
# This stays false until a robot/tool collision model and measured panel world
# are used by this executor, then validated in an offline run record.
_COLLISION_BACKEND_READY_FOR_MOTION = False

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

    @staticmethod
    def _load_env_cuboids():
        from curobo.geom.types import Cuboid
        if not _ENVIRONMENT_YAML.exists():
            return []
        with _ENVIRONMENT_YAML.open() as fh:
            data = yaml.safe_load(fh) or {}
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
            except Exception:
                pass
        return cuboids

    def __init__(self) -> None:
        super().__init__("scan_executor_node")

        self._current_joints: Optional[List[float]] = None
        self._started = False
        self._mg: Optional[MotionGen] = None
        self.declare_parameter("execute_motion", False)
        self._execute_motion = bool(self.get_parameter("execute_motion").value)

        pkg = get_package_share_directory("strawberry_motion")
        candidates_path = Path(pkg) / "config" / "scan_pose_candidates.yaml"
        with candidates_path.open() as fh:
            data = yaml.safe_load(fh)
        candidate_cfg = data["scan_pose_candidates"]
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
                "Motion locked: scan candidates are preview/dry-run evidence only; "
                "a collision-aware execution backend is not validated."
            )

        cb = rclpy.callback_groups.ReentrantCallbackGroup()
        self.create_subscription(JointState, "/dsr01/joint_states", self._joint_cb, 10)
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
        self.get_logger().info("Initialising cuRobo MotionGen")
        tensor_args = TensorDeviceType(device=torch.device("cuda:0"))
        # This backend is unreachable while the current config is unauthorized.
        # Replace this empty-world setup with the verified collision scene before
        # setting collision_world_validated_for_motion=true.
        robot_cfg = RobotConfig.from_basic(
            urdf_path=str(_URDF_PATH),
            base_link="base_link",
            ee_link="gripper_rh_p12_rn_base",
            tensor_args=tensor_args,
        )
        world_cfg = WorldConfig(cuboid=[])
        mg_cfg = MotionGenConfig.load_from_robot_config(
            robot_cfg, world_cfg, tensor_args=tensor_args,
            num_trajopt_seeds=16, num_graph_seeds=16,
            collision_cache={"obb": 10, "mesh": 5},
            use_cuda_graph=False,
        )
        self._mg = MotionGen(mg_cfg)
        self._mg.warmup(warmup_js_trajopt=False)
        self.get_logger().info("cuRobo MotionGen ready")

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _joint_cb(self, msg: JointState) -> None:
        jmap = {n: p for n, p in zip(msg.name, msg.position)}
        joints = [jmap.get(n) for n in _JOINT_NAMES]
        if None not in joints:
            self._current_joints = joints

    def _start_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        allowed, reason = motion_start_allowed(
            execute_motion=self._execute_motion,
            candidate_authorized=self._candidate_authorized,
            has_joint_state=self._current_joints is not None,
        )
        if self._started:
            allowed, reason = False, "scan already started"
        if allowed and not joints_within_tolerance_deg(
            self._current_joints or [], _OVERVIEW_JOINTS_DEG, _OVERVIEW_TOLERANCE_DEG
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
        self, start_joints: List[float], pos: List[float], quat_wxyz: List[float], label: str
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
        result = self._mg.plan_single(
            start, goal, MotionGenPlanConfig(enable_graph=True, max_attempts=4)
        )
        if not result.success.item():
            self.get_logger().error(
                "%s plan failed: %s" % (label, getattr(result, "status", "?"))
            )
            return None
        traj = result.get_interpolated_plan().position.cpu().numpy()
        if not self._traj_ok(traj, label):
            return None
        return traj, float(result.motion_time.item())

    def _exec_spline(self, traj_rad: np.ndarray, motion_time: float) -> bool:
        if not self._cli_spline.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("MoveSplineJoint service not available")
            return False
        deg = np.rad2deg(traj_rad)
        n = deg.shape[0]
        if n > _MAX_SPLINE_PTS:
            idx = np.linspace(0, n - 1, _MAX_SPLINE_PTS, dtype=int)
            deg = deg[idx]
            n = _MAX_SPLINE_PTS
        req = MoveSplineJoint.Request()
        req.pos_cnt = n
        for row in deg:
            pt = Float64MultiArray()
            pt.data = row.tolist()
            req.pos.append(pt)
        req.vel = [120.0] * 6
        req.acc = [180.0] * 6
        req.time = max(motion_time * _SPLINE_TIME_SCALE, _SPLINE_MIN_TIME)
        req.mode = 0
        req.sync_type = 0
        future = self._cli_spline.call_async(req)
        t0 = time.time()
        while not future.done() and (time.time() - t0) < 60.0:
            time.sleep(0.05)
        ok = future.done() and future.result() and future.result().success
        if not ok:
            self.get_logger().error("Spline execution failed/timeout")
        return bool(ok)

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
        while not future.done() and (time.time() - t0) < 60.0:
            time.sleep(0.05)
        ok = future.done() and future.result() and future.result().success
        if not ok:
            self.get_logger().error("MoveJoint failed")
        return bool(ok)

    def _is_at_overview(self) -> bool:
        return self._current_joints is not None and joints_within_tolerance_deg(
            self._current_joints, _OVERVIEW_JOINTS_DEG, _OVERVIEW_TOLERANCE_DEG
        )

    def _wait_at_overview(self, timeout_sec: float = 10.0) -> bool:
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            if self._is_at_overview():
                return True
            time.sleep(0.05)
        return False

    # ── scan sequence (runs in background thread) ─────────────────────────────

    def _scan_sequence(self) -> None:
        self._init_motion_gen()
        # The operator must manually place the robot at this verified overview
        # pose before requesting start; the executor does not move there blindly.
        current_joints = np.deg2rad(_OVERVIEW_JOINTS_DEG).tolist()
        self._pub_status("SCAN_STARTED order=%s" % str(_SCAN_ORDER))

        for cell_id in _SCAN_ORDER:
            if cell_id not in self._targets:
                self.get_logger().warn("%s not in candidates — skipping" % cell_id)
                continue

            target = self._targets[cell_id]
            mat4 = np.array(target["tcp_transform_base"])
            pos, quat = _mat4_to_pos_quat_wxyz(mat4)

            self._pub_state(cell_id, "SCANNING")
            self._pub_status("MOVING_TO %s  pos=[%.3f, %.3f, %.3f]" % (cell_id, *pos))

            ret = self._plan(current_joints, pos, quat, cell_id)
            if ret is None:
                self._pub_status("PLAN_FAIL %s — aborting scan sequence" % cell_id)
                self._pub_state(cell_id, "PLANNING_FAIL")
                self._pub_status("RETURNING_TO_OVERVIEW after failure")
                self._movej(_OVERVIEW_JOINTS_DEG, vel=40.0, acc=40.0)
                return

            if not self._exec_spline(*ret):
                self._pub_status("EXEC_FAIL %s — aborting scan sequence" % cell_id)
                self._pub_state(cell_id, "PLANNING_FAIL")
                self._pub_status("RETURNING_TO_OVERVIEW after failure")
                self._movej(_OVERVIEW_JOINTS_DEG, vel=40.0, acc=40.0)
                return

            self._pub_status("AT_SCAN_POSE %s — dwell %.1fs" % (cell_id, _SCAN_DWELL_SEC))
            time.sleep(_SCAN_DWELL_SEC)

            self._pub_state(cell_id, "SCAN_POSE_REACHED")
            self._pub_status("SCAN_POSE_REACHED %s; detector result pending" % cell_id)

            # Return to overview between cells so each plan uses the same
            # start state as the dry-run, preventing J6 branch flip issues.
            self._pub_status("RETURNING_TO_OVERVIEW after %s" % cell_id)
            if not self._movej(_OVERVIEW_JOINTS_DEG, vel=20.0, acc=20.0):
                self._pub_status("ABORT overview return failed after %s" % cell_id)
                return
            if not self._wait_at_overview():
                self._pub_status("ABORT overview pose was not confirmed after %s" % cell_id)
                return
            current_joints = np.deg2rad(_OVERVIEW_JOINTS_DEG).tolist()

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
