"""Publish quadtree workspace cells and next observation cell for RViz."""

from pathlib import Path
from typing import Dict, Tuple

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from strawberry_motion.exploration.quadtree_map import QuadtreeCell, QuadtreeMap
from strawberry_motion.exploration.region_state import RegionState
from strawberry_motion.exploration.scan_pose_generator import (
    ObservationPosePreview,
    generate_observation_pose_previews,
)
from strawberry_motion.exploration.workspace_config import load_workspace_config


Color = Tuple[float, float, float, float]
STATE_COLORS: Dict[RegionState, Color] = {
    RegionState.UNSCANNED: (0.2, 0.6, 1.0, 1.0),
    RegionState.SCANNING: (1.0, 1.0, 0.0, 1.0),   # yellow: robot is moving here now
    RegionState.SCAN_POSE_REACHED: (0.0, 1.0, 1.0, 1.0),  # waits for perception result
    RegionState.SCANNED_EMPTY: (0.5, 0.5, 0.5, 1.0),
    RegionState.TARGET_FOUND: (1.0, 0.8, 0.0, 1.0),
    RegionState.PICKABLE: (0.1, 0.9, 0.2, 1.0),
    RegionState.OCCLUDED: (1.0, 0.5, 0.0, 1.0),
    RegionState.PLANNING_FAIL: (1.0, 0.1, 0.1, 1.0),
    RegionState.HARVESTED: (0.1, 0.8, 0.8, 1.0),
    RegionState.REVISIT: (0.8, 0.2, 1.0, 1.0),
}


class WorkspaceMarkerNode(Node):
    """Expose initial exploration state for RViz and later integration."""

    def __init__(self) -> None:
        super().__init__("workspace_marker_node")
        default_config = (
            Path(get_package_share_directory("strawberry_motion"))
            / "config"
            / "workspace.yaml"
        )
        self.declare_parameter("config_file", str(default_config))
        self.config = load_workspace_config(Path(self.get_parameter("config_file").value))
        self.tree = QuadtreeMap(
            self.config.bounds, self.config.max_depth, root_split=self.config.root_split
        )
        self._subdivide_to_depth("root", self.config.initial_subdivision_depth)

        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_publisher = self.create_publisher(
            MarkerArray, "/strawberry/exploration/workspace_cells", latched_qos
        )
        self.next_cell_publisher = self.create_publisher(
            String, "/strawberry/exploration/next_cell", latched_qos
        )
        self.state_subscription = self.create_subscription(
            String,
            "/strawberry/exploration/set_cell_state",
            self._on_state_update,
            10,
        )
        self.timer = self.create_timer(1.0, self._publish_state)
        self._publish_state()
        self.get_logger().info(
            "Workspace visualization ready: frame=%s, leaf_cells=%d"
            % (self.config.frame_id, len(list(self.tree.leaf_cells())))
        )

    def _subdivide_to_depth(self, cell_id: str, remaining_depth: int) -> None:
        if remaining_depth <= 0:
            return
        for child in self.tree.subdivide(cell_id):
            self._subdivide_to_depth(child.cell_id, remaining_depth - 1)

    def _on_state_update(self, message: String) -> None:
        try:
            cell_id, state_name = (part.strip() for part in message.data.split("=", 1))
            state = RegionState(state_name.upper())
            self.tree.update_state(cell_id, state)
        except (ValueError, KeyError) as exc:
            self.get_logger().warning(
                "Rejected state update '%s': %s" % (message.data, exc)
            )
            return
        self.get_logger().info("Updated %s -> %s" % (cell_id, state.value))
        self._publish_state()

    def _publish_state(self) -> None:
        marker_array = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)

        cells = list(self.tree.leaf_cells())
        for index, cell in enumerate(cells):
            base_id = index * 3
            marker_array.markers.append(self._cell_outline_marker(cell, base_id))
            marker_array.markers.append(self._cell_center_marker(cell, base_id + 1))
            marker_array.markers.append(self._cell_label_marker(cell, base_id + 2))
        if self.config.preview_standoff_m is not None:
            for index, preview in enumerate(
                generate_observation_pose_previews(cells, self.config.preview_standoff_m)
            ):
                marker_array.markers.append(self._scan_pose_preview_marker(preview, 20000 + index))
                marker_array.markers.append(self._scan_camera_preview_marker(preview, 21000 + index))

        next_cell = self.tree.next_scan_cell()
        # Keep /next_cell for logs/automation, but do not draw the magenta dot in
        # RViz. The scan-pose preview already shows the physically taught TCP
        # targets and the extra dot made the validation view noisy.
        self.marker_publisher.publish(marker_array)

        selection = String()
        selection.data = next_cell.cell_id if next_cell is not None else ""
        self.next_cell_publisher.publish(selection)

    def _base_marker(self, namespace: str, marker_id: int, marker_type: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.config.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _cell_outline_marker(self, cell: QuadtreeCell, marker_id: int) -> Marker:
        marker = self._base_marker("workspace_cells", marker_id, Marker.LINE_STRIP)
        marker.scale.x = 0.004
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = STATE_COLORS[cell.state]
        bounds = cell.bounds
        marker.points = [
            self._point(bounds.min_x, bounds.min_y),
            self._point(bounds.max_x, bounds.min_y),
            self._point(bounds.max_x, bounds.max_y),
            self._point(bounds.min_x, bounds.max_y),
            self._point(bounds.min_x, bounds.min_y),
        ]
        return marker

    def _cell_center_marker(self, cell: QuadtreeCell, marker_id: int) -> Marker:
        marker = self._base_marker("workspace_cell_centers", marker_id, Marker.SPHERE)
        marker.pose.position.x, marker.pose.position.y = cell.center
        marker.pose.position.z = 0.008
        marker.scale.x = marker.scale.y = marker.scale.z = 0.018
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (1.0, 0.85, 0.0, 1.0)
        return marker

    def _cell_label_marker(self, cell: QuadtreeCell, marker_id: int) -> Marker:
        marker = self._base_marker("workspace_cell_labels", marker_id, Marker.TEXT_VIEW_FACING)
        marker.pose.position.x, marker.pose.position.y = cell.center
        marker.pose.position.y -= 0.045
        marker.pose.position.z = 0.018
        marker.scale.z = 0.045
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (1.0, 0.85, 0.0, 1.0)
        marker.text = cell.cell_id.split("/")[-1].upper()
        return marker

    def _scan_pose_preview_marker(
        self, preview: ObservationPosePreview, marker_id: int
    ) -> Marker:
        marker = self._base_marker("scan_pose_previews", marker_id, Marker.ARROW)
        marker.scale.x = 0.008
        marker.scale.y = 0.018
        marker.scale.z = 0.022
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (1.0, 0.75, 0.0, 0.7)
        marker.points = [
            self._point_3d(preview.x, preview.y, preview.z),
            self._point(preview.x, preview.y),
        ]
        return marker

    def _scan_camera_preview_marker(
        self, preview: ObservationPosePreview, marker_id: int
    ) -> Marker:
        marker = self._base_marker("scan_camera_previews", marker_id, Marker.SPHERE)
        marker.pose.position = self._point_3d(preview.x, preview.y, preview.z)
        marker.scale.x = marker.scale.y = marker.scale.z = 0.04
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (1.0, 0.75, 0.0, 0.9)
        return marker

    @staticmethod
    def _point(x: float, y: float) -> Point:
        return WorkspaceMarkerNode._point_3d(x, y, 0.0)

    @staticmethod
    def _point_3d(x: float, y: float, z: float) -> Point:
        point = Point()
        point.x = x
        point.y = y
        point.z = z
        return point


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WorkspaceMarkerNode()
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
