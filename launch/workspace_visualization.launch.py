from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="strawberry_motion",
                executable="workspace_marker_node",
                name="workspace_marker_node",
                output="screen",
            ),
        ]
    )
