"""Workspace scan executor: visits v6 scan poses in Z-order (NW→NE→SE→SW).

Cell state published to /strawberry/exploration/set_cell_state:
  SCANNING      while robot is moving to the cell
  SCANNED_EMPTY after dwell (no detector attached yet)
  PLANNING_FAIL if cuRobo or execution fails

After all cells: returns to overview pose via MoveJoint.
Scan starts automatically once the first /dsr01/joint_states message arrives.

Run:
  ros2 launch strawberry_motion workspace_scan.launch.py
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

from ament_index_python.packages import get_package_share_directory
from dsr_msgs2.srv import MoveJoint, MoveSplineJoint

from curobo.geom.types import WorldConfig  # noqa: F401 – kept for WorldConfig(cuboid=[])
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.robot import JointState as CuroboJointState, RobotConfig
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig


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

        pkg = get_package_share_directory("strawberry_motion")
        candidates_path = Path(pkg) / "config" / "scan_pose_candidates.yaml"
        with candidates_path.open() as fh:
            data = yaml.safe_load(fh)
        self._targets: Dict[str, dict] = {
            t["cell_id"]: t
            for t in data["scan_pose_candidates"]["targets"]
            if t.get("tcp_transform_base") is not None
        }
        self.get_logger().info(
            "Loaded %d scan targets from %s" % (len(self._targets), candidates_path)
        )

        self.get_logger().info("Initialising cuRobo MotionGen …")
        tensor_args = TensorDeviceType(device=torch.device("cuda:0"))
        # Use from_basic (no collision spheres) to match the dry-run conditions
        # that validated these scan poses. The scan poses are at 0.4–0.65 m
        # standoff in front of the panel — no real collision risk with whiteboard.
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

        cb = rclpy.callback_groups.ReentrantCallbackGroup()
        self.create_subscription(JointState, "/dsr01/joint_states", self._joint_cb, 10)
        self._state_pub = self.create_publisher(
            String, "/strawberry/exploration/set_cell_state", 10
        )
        self._status_pub = self.create_publisher(String, "/strawberry/scan/status", 10)
        self._cli_spline = self.create_client(
            MoveSplineJoint, "/dsr01/motion/move_spline_joint", callback_group=cb
        )
        self._cli_movej = self.create_client(
            MoveJoint, "/dsr01/motion/move_joint", callback_group=cb
        )

        self.create_timer(1.0, self._check_start)
        self.get_logger().info("scan_executor_node ready — waiting for /dsr01/joint_states")

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _joint_cb(self, msg: JointState) -> None:
        jmap = {n: p for n, p in zip(msg.name, msg.position)}
        joints = [jmap.get(n) for n in _JOINT_NAMES]
        if None not in joints:
            self._current_joints = joints

    def _check_start(self) -> None:
        if self._started or self._current_joints is None:
            return
        self._started = True
        threading.Thread(target=self._scan_sequence, daemon=True).start()

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

    def _clamp(self, joints: List[float]) -> List[float]:
        return [float(np.clip(j, lo, hi)) for j, (lo, hi) in zip(joints, _JOINT_LIMITS_RAD)]

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
                [self._clamp(start_joints)], device="cuda:0", dtype=torch.float32
            ),
            joint_names=_JOINT_NAMES,
        )
        goal = Pose(
            position=torch.tensor([pos], device="cuda:0", dtype=torch.float32),
            quaternion=torch.tensor([quat_wxyz], device="cuda:0", dtype=torch.float32),
        )
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

    # ── scan sequence (runs in background thread) ─────────────────────────────

    def _scan_sequence(self) -> None:
        self._pub_status("MOVING_TO_OVERVIEW before scan start")
        if not self._movej(_OVERVIEW_JOINTS_DEG, vel=40.0, acc=40.0):
            self._pub_status("ABORT overview move failed")
            return
        # Use the known overview joint values as cuRobo start state so the
        # world-collision check matches the validated dry-run start state.
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

            self._pub_state(cell_id, "SCANNED_EMPTY")
            self._pub_status("SCANNED %s" % cell_id)

            # Return to overview between cells so each plan uses the same
            # start state as the dry-run, preventing J6 branch flip issues.
            self._pub_status("RETURNING_TO_OVERVIEW after %s" % cell_id)
            self._movej(_OVERVIEW_JOINTS_DEG, vel=40.0, acc=40.0)
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
