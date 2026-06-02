"""Publish active scan pose TCP/camera candidates as RViz markers in base_link.

Reads scan_pose_candidates_refit_candidate.yaml and publishes per-cell markers:
  - TCP/gripper frame axes from the taught base_link transform
  - camera optical axis derived from eye-in-hand calibration
  - optional gray line from TCP position to cell center for task-frame context

This preview is intentionally tied to the current v12 gripper-centered poses.
Older camera-centered generated preview markers are disabled in workspace.yaml.

Color coding:
  PHYSICAL_VIEW_CONFIRMED* -> green
  PLAN_VALID               -> green
  IK_FAIL*                 -> red
  anything else -> orange

Run standalone (no ROS2):
  python3 -m strawberry_motion.visualization.scan_pose_tcp_preview_node --once
"""

from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np
import yaml

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


_PANEL_CELL_XY = {
    "root/nw": (-0.2725,  0.1975),
    "root/ne": ( 0.2775,  0.1975),
    "root/sw": (-0.2725, -0.2025),
    "root/se": ( 0.2775, -0.2025),
}

_STATUS_COLOR = {
    "PLAN_VALID":           (0.1, 0.9, 0.2, 0.9),   # green
    "IK_FAIL":              (1.0, 0.1, 0.1, 0.9),   # red
    "IK_FAIL_USE_ALTERNATIVE": (1.0, 0.5, 0.0, 0.9),  # orange
}
_DEFAULT_COLOR = (1.0, 0.5, 0.0, 0.9)
_AXIS_LEN_M = 0.14
_CAM_AXIS_LEN_M = 0.18
_TASK_LINE_COLOR = (0.6, 0.6, 0.6, 0.55)
_TCP_AXIS_COLORS = {
    "x": (1.0, 0.1, 0.1, 0.85),
    "y": (0.1, 0.9, 0.1, 0.85),
    "z": (0.1, 0.35, 1.0, 0.85),
}
_CAM_COLOR = (0.0, 0.9, 1.0, 0.9)


def _cell_center_base(panel_T: np.ndarray, cell_id: str) -> np.ndarray:
    cx, cy = _PANEL_CELL_XY[cell_id]
    p_panel = np.array([cx, cy, 0.0, 1.0])
    return (panel_T @ p_panel)[:3]


def _expand_path(path_text: str) -> Path:
    return Path(path_text.replace("~", str(Path.home()))).expanduser()


def _load_camera_transform(candidate_cfg: dict) -> Optional[np.ndarray]:
    calib_path = candidate_cfg.get("source_calibration_local")
    if not calib_path:
        return None
    path = _expand_path(str(calib_path))
    if not path.exists():
        return None
    data = np.load(path)
    if "T_cam_to_gripper" not in data:
        return None
    return np.array(data["T_cam_to_gripper"], dtype=float)


def _load_active_targets(candidates_path: Path, registration_path: Path):
    with candidates_path.open() as f:
        data = yaml.safe_load(f)
    with registration_path.open() as f:
        reg = yaml.safe_load(f)["panel_registration"]
    panel_T = np.array(reg["transform"]["matrix"])

    candidate_cfg = data["scan_pose_candidates"]
    t_cam_to_tcp = _load_camera_transform(candidate_cfg)
    targets = candidate_cfg["targets"]
    result = []
    for t in targets:
        if t.get("tcp_transform_base") is None:
            continue
        mat4 = np.array(t["tcp_transform_base"])
        tcp_pos = mat4[:3, 3]
        cell_center = _cell_center_base(panel_T, t["cell_id"])
        status = t.get("curobo_status", "UNKNOWN")
        camera_mat4 = mat4 @ t_cam_to_tcp if t_cam_to_tcp is not None else None
        result.append(
            {
                "cell_id": t["cell_id"],
                "tcp_mat4": mat4,
                "tcp_pos": tcp_pos,
                "cell_center": cell_center,
                "status": status,
                "approach": t.get("approach", "?"),
                "camera_mat4": camera_mat4,
                "joints": t.get("endpoint_joints_deg", []),
            }
        )
    return result


class ScanPoseTcpPreviewNode(Node):

    def __init__(self) -> None:
        super().__init__("scan_pose_tcp_preview_node")

        pkg = get_package_share_directory("strawberry_motion")
        self._candidates_path = (
            Path(pkg) / "config" / "scan_pose_candidates_refit_candidate.yaml"
        )
        self._registration_path = Path(pkg) / "config" / "panel_registration.yaml"

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub = self.create_publisher(
            MarkerArray, "/strawberry/scan_poses/tcp_preview", latched
        )
        self.create_timer(2.0, self._publish)
        self._publish()
        self.get_logger().info("scan_pose_tcp_preview_node ready")

    def _publish(self) -> None:
        try:
            targets = _load_active_targets(self._candidates_path, self._registration_path)
        except Exception as exc:
            self.get_logger().warning("Failed to load candidates: %s" % exc)
            return

        ma = MarkerArray()
        clear = Marker(); clear.action = Marker.DELETEALL
        ma.markers.append(clear)

        marker_id = 0
        for idx, target in enumerate(targets):
            cell_id = target["cell_id"]
            tcp_pos = target["tcp_pos"]
            tcp_mat4 = target["tcp_mat4"]
            cell_center = target["cell_center"]
            status = target["status"]
            approach = target["approach"]
            color = _STATUS_COLOR.get(status, _DEFAULT_COLOR)
            if str(status).startswith("PHYSICAL_VIEW_CONFIRMED"):
                color = _STATUS_COLOR["PLAN_VALID"]

            # Thin task-context line: TCP position -> cell center.
            task_line = self._arrow_marker(
                "v12_tcp_to_cell_center",
                marker_id,
                tcp_pos,
                cell_center,
                _TASK_LINE_COLOR,
                scale=(0.004, 0.010, 0.014),
            )
            marker_id += 1
            ma.markers.append(task_line)

            # TCP/gripper axes. These show the actual taught gripper frame, not
            # the old camera-centered generated pose.
            for axis_i, axis_name in enumerate(("x", "y", "z")):
                axis_tip = tcp_pos + tcp_mat4[:3, axis_i] * _AXIS_LEN_M
                axis = self._arrow_marker(
                    "v12_tcp_axes",
                    marker_id,
                    tcp_pos,
                    axis_tip,
                    _TCP_AXIS_COLORS[axis_name],
                    scale=(0.008, 0.018, 0.024),
                )
                marker_id += 1
                ma.markers.append(axis)

            camera_mat4 = target.get("camera_mat4")
            if camera_mat4 is not None:
                cam_pos = camera_mat4[:3, 3]
                cam_tip = cam_pos + camera_mat4[:3, 2] * _CAM_AXIS_LEN_M
                cam_arrow = self._arrow_marker(
                    "v12_camera_optical_axis",
                    marker_id,
                    cam_pos,
                    cam_tip,
                    _CAM_COLOR,
                    scale=(0.008, 0.018, 0.024),
                )
                marker_id += 1
                ma.markers.append(cam_arrow)

                cam_sphere = self._sphere_marker(
                    "v12_camera_centers", marker_id, cam_pos, _CAM_COLOR, 0.030
                )
                marker_id += 1
                ma.markers.append(cam_sphere)

            # Sphere at TCP position
            sphere = self._sphere_marker(
                "v12_tcp_centers", marker_id, tcp_pos, color, 0.040
            )
            marker_id += 1
            ma.markers.append(sphere)

            # Label at TCP position
            text = Marker()
            text.header.frame_id = "base_link"
            text.header.stamp = self.get_clock().now().to_msg()
            text.ns = "v12_tcp_labels"
            text.id = marker_id
            marker_id += 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = tcp_pos[0]
            text.pose.position.y = tcp_pos[1]
            text.pose.position.z = tcp_pos[2] + 0.06
            text.pose.orientation.w = 1.0
            text.scale.z = 0.035
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0
            text.text = (
                "%s\n%s [%s]\nTCP base [%.3f %.3f %.3f]"
                % (cell_id, status, approach, tcp_pos[0], tcp_pos[1], tcp_pos[2])
            )
            ma.markers.append(text)

        self._pub.publish(ma)
        self.get_logger().debug("Published %d v12 TCP/camera marker groups" % len(targets))

    def _arrow_marker(
        self,
        namespace: str,
        marker_id: int,
        tail_xyz: np.ndarray,
        tip_xyz: np.ndarray,
        color: Tuple[float, float, float, float],
        scale: Tuple[float, float, float],
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x, marker.scale.y, marker.scale.z = scale
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        tail = Point(); tail.x, tail.y, tail.z = [float(v) for v in tail_xyz]
        tip = Point(); tip.x, tip.y, tip.z = [float(v) for v in tip_xyz]
        marker.points = [tail, tip]
        return marker

    def _sphere_marker(
        self,
        namespace: str,
        marker_id: int,
        xyz: np.ndarray,
        color: Tuple[float, float, float, float],
        diameter_m: float,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(xyz[0])
        marker.pose.position.y = float(xyz[1])
        marker.pose.position.z = float(xyz[2])
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = diameter_m
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        return marker


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanPoseTcpPreviewNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
