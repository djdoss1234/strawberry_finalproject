"""Launch the quadtree workspace markers with a capture-ready RViz view."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    rviz_config = (
        Path(get_package_share_directory("strawberry_motion"))
        / "rviz"
        / "workspace_exploration.rviz"
    )
    return LaunchDescription(
        [
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="workspace_visualization_frame_publisher",
                arguments=[
                    "--x",
                    "0",
                    "--y",
                    "0",
                    "--z",
                    "0",
                    "--roll",
                    "0",
                    "--pitch",
                    "0",
                    "--yaw",
                    "0",
                    "--frame-id",
                    "workspace_visualization_world",
                    "--child-frame-id",
                    "cultivation_panel",
                ],
                output="screen",
            ),
            Node(
                package="strawberry_motion",
                executable="workspace_marker_node",
                name="workspace_marker_node",
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="workspace_rviz",
                arguments=["-d", str(rviz_config)],
                output="screen",
            ),
        ]
    )
