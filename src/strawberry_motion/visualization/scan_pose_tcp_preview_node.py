"""Publish a clean RViz preview of the active v12 scan poses.

Reads scan_pose_candidates_refit_candidate.yaml and publishes only the markers
needed for physical scan-pose validation:
  - green sphere: taught scan TCP position executed by MoveJoint
  - green sphere/arrow: taught scan TCP and gripper forward axis
  - cyan sphere/arrow: camera center and camera optical +Z axis
  - base_link label near the RViz base axes

Per-cell TCP coordinate labels are intentionally hidden to keep the validation
view clean. Older camera-centered generated preview markers are disabled in
workspace.yaml.
"""

from pathlib import Path
from typing import Optional, Tuple

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
_CAM_AXIS_LEN_M = 0.14
_TCP_COLOR = (0.1, 0.9, 0.2, 0.95)
_TCP_AXIS_LEN_M = 0.12
_CAM_COLOR = (0.0, 0.9, 1.0, 0.9)
_LABEL_COLOR = (1.0, 1.0, 1.0, 0.95)


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
                "panel_T": panel_T,
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
            # TCP position actually sent by YAML MoveJoint.
            sphere = self._sphere_marker(
                "v12_scan_tcp_position", marker_id, tcp_pos, _TCP_COLOR, 0.035
            )
            marker_id += 1
            ma.markers.append(sphere)

            # The gripper/tool approach direction is the TCP +X axis in the
            # current gripper_rh frame convention. This is the important axis
            # for gripper-centered scan validation; camera optical direction is
            # shown separately in cyan.
            tcp_forward_tip = tcp_pos + tcp_mat4[:3, 0] * _TCP_AXIS_LEN_M
            tcp_forward = self._arrow_marker(
                "v12_gripper_forward_axis",
                marker_id,
                tcp_pos,
                tcp_forward_tip,
                _TCP_COLOR,
                scale=(0.006, 0.014, 0.020),
            )
            marker_id += 1
            ma.markers.append(tcp_forward)

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
                    scale=(0.006, 0.014, 0.020),
                )
                marker_id += 1
                ma.markers.append(cam_arrow)

                cam_sphere = self._sphere_marker(
                    "v12_camera_center", marker_id, cam_pos, _CAM_COLOR, 0.022
                )
                marker_id += 1
                ma.markers.append(cam_sphere)

        ma.markers.append(self._base_link_label(marker_id))
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


    def _text_marker(
        self,
        namespace: str,
        marker_id: int,
        xyz: np.ndarray,
        text_value: str,
        height_m: float,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = float(xyz[0])
        marker.pose.position.y = float(xyz[1])
        marker.pose.position.z = float(xyz[2])
        marker.pose.orientation.w = 1.0
        marker.scale.z = height_m
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = _LABEL_COLOR
        marker.text = text_value
        return marker

    def _base_link_label(self, marker_id: int) -> Marker:
        return self._text_marker(
            "v12_base_link_label",
            marker_id,
            np.array([0.055, 0.0, 0.055]),
            "base_link",
            0.035,
        )



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
