"""Publish v4 scan pose TCP candidates as RViz markers in base_link frame.

Reads scan_pose_candidates.yaml and publishes one ARROW marker per cell:
  tail = TCP position (ee_link in base_link frame)
  tip  = estimated cell center in base_link frame (from panel_registration)

Color coding:
  PLAN_VALID  -> green
  IK_FAIL*   -> red
  anything else -> orange

Run standalone (no ROS2):
  python3 -m strawberry_motion.visualization.scan_pose_tcp_preview_node --once
"""

from pathlib import Path
from typing import List, Tuple

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


def _cell_center_base(panel_T: np.ndarray, cell_id: str) -> np.ndarray:
    cx, cy = _PANEL_CELL_XY[cell_id]
    p_panel = np.array([cx, cy, 0.0, 1.0])
    return (panel_T @ p_panel)[:3]


def _load_v4_targets(candidates_path: Path, registration_path: Path):
    with candidates_path.open() as f:
        data = yaml.safe_load(f)
    with registration_path.open() as f:
        reg = yaml.safe_load(f)["panel_registration"]
    panel_T = np.array(reg["transform"]["matrix"])

    targets = data["scan_pose_candidates"]["targets"]
    result = []
    for t in targets:
        if t.get("tcp_transform_base") is None:
            continue
        mat4 = np.array(t["tcp_transform_base"])
        tcp_pos = mat4[:3, 3]
        cell_center = _cell_center_base(panel_T, t["cell_id"])
        status = t.get("curobo_status", "UNKNOWN")
        result.append((t["cell_id"], tcp_pos, cell_center, status, t.get("approach", "?")))
    return result


class ScanPoseTcpPreviewNode(Node):

    def __init__(self) -> None:
        super().__init__("scan_pose_tcp_preview_node")

        pkg = get_package_share_directory("strawberry_motion")
        self._candidates_path = Path(pkg) / "config" / "scan_pose_candidates.yaml"
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
            targets = _load_v4_targets(self._candidates_path, self._registration_path)
        except Exception as exc:
            self.get_logger().warning("Failed to load candidates: %s" % exc)
            return

        ma = MarkerArray()
        clear = Marker(); clear.action = Marker.DELETEALL
        ma.markers.append(clear)

        for idx, (cell_id, tcp_pos, cell_center, status, approach) in enumerate(targets):
            color = _STATUS_COLOR.get(status, _DEFAULT_COLOR)

            # Arrow: TCP position -> cell center
            arrow = Marker()
            arrow.header.frame_id = "base_link"
            arrow.header.stamp = self.get_clock().now().to_msg()
            arrow.ns = "v4_tcp_arrows"
            arrow.id = idx
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.scale.x = 0.010   # shaft diameter
            arrow.scale.y = 0.020   # head diameter
            arrow.scale.z = 0.025   # head length
            arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = color
            tail = Point(); tail.x, tail.y, tail.z = tcp_pos.tolist()
            tip  = Point(); tip.x,  tip.y,  tip.z  = cell_center.tolist()
            arrow.points = [tail, tip]
            ma.markers.append(arrow)

            # Sphere at TCP position
            sphere = Marker()
            sphere.header.frame_id = "base_link"
            sphere.header.stamp = self.get_clock().now().to_msg()
            sphere.ns = "v4_tcp_spheres"
            sphere.id = idx
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position = tail
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.04
            sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = color
            ma.markers.append(sphere)

            # Label at TCP position
            text = Marker()
            text.header.frame_id = "base_link"
            text.header.stamp = self.get_clock().now().to_msg()
            text.ns = "v4_tcp_labels"
            text.id = idx
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = tcp_pos[0]
            text.pose.position.y = tcp_pos[1]
            text.pose.position.z = tcp_pos[2] + 0.06
            text.pose.orientation.w = 1.0
            text.scale.z = 0.035
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0
            text.text = "%s\n%s\n[%s]" % (cell_id, status, approach)
            ma.markers.append(text)

        self._pub.publish(ma)
        self.get_logger().debug("Published %d v4 TCP markers" % len(targets))


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
