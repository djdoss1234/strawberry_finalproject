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
from strawberry_motion.exploration.workspace_config import load_workspace_config


Color = Tuple[float, float, float, float]
STATE_COLORS: Dict[RegionState, Color] = {
    RegionState.UNSCANNED: (0.2, 0.6, 1.0, 1.0),
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
            marker_array.markers.append(self._cell_outline_marker(cell, index * 2))
            marker_array.markers.append(self._cell_label_marker(cell, index * 2 + 1))

        next_cell = self.tree.next_scan_cell()
        if next_cell is not None:
            marker_array.markers.append(self._next_cell_marker(next_cell))
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

    def _cell_label_marker(self, cell: QuadtreeCell, marker_id: int) -> Marker:
        marker = self._base_marker("workspace_labels", marker_id, Marker.TEXT_VIEW_FACING)
        marker.scale.z = 0.035
        marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.0
        marker.pose.position.x, marker.pose.position.y = cell.center
        marker.pose.position.z = 0.01
        marker.text = "%s\n%s" % (cell.cell_id, cell.state.value)
        return marker

    def _next_cell_marker(self, cell: QuadtreeCell) -> Marker:
        marker = self._base_marker("next_scan_cell", 10000, Marker.SPHERE)
        marker.pose.position.x, marker.pose.position.y = cell.center
        marker.pose.position.z = 0.015
        marker.scale.x = marker.scale.y = marker.scale.z = 0.025
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (1.0, 0.0, 1.0, 1.0)
        return marker

    @staticmethod
    def _point(x: float, y: float) -> Point:
        point = Point()
        point.x = x
        point.y = y
        point.z = 0.0
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
