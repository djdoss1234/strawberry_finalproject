"""Launch workspace scan visualization; robot execution is opt-in and locked.

By default this launch starts TF, markers, scan-pose preview and RViz only.
Even with enable_robot_execution:=true, the executor requires an explicit
Trigger request and an authorized collision-aware candidate config.
Monitor progress:
  ros2 topic echo /strawberry/scan/status
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("strawberry_motion")
    rviz_config = Path(pkg) / "rviz" / "workspace_exploration.rviz"
    registration_file = Path(pkg) / "config" / "panel_registration.yaml"

    with registration_file.open("r", encoding="utf-8") as stream:
        registration = yaml.safe_load(stream)["panel_registration"]
    translation = registration["transform"]["translation_m"]
    rotation = registration["transform"]["rotation_xyzw"]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_robot_execution",
                default_value="false",
                description="Start the locked scan executor node; never moves without explicit authorization.",
            ),
            DeclareLaunchArgument(
                "target_cell",
                default_value="",
                description="Initial physical validation permits one explicitly selected root/nw or root/ne cell only.",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="workspace_visualization_frame_publisher",
                arguments=[
                    "--x", str(translation["x"]),
                    "--y", str(translation["y"]),
                    "--z", str(translation["z"]),
                    "--qx", str(rotation["x"]),
                    "--qy", str(rotation["y"]),
                    "--qz", str(rotation["z"]),
                    "--qw", str(rotation["w"]),
                    "--frame-id", registration["parent_frame_id"],
                    "--child-frame-id", registration["frame_id"],
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
                package="strawberry_motion",
                executable="scan_pose_tcp_preview_node",
                name="scan_pose_tcp_preview_node",
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="workspace_rviz",
                arguments=["-d", str(rviz_config)],
                output="screen",
            ),
            Node(
                package="strawberry_motion",
                executable="scan_executor_node",
                name="scan_executor_node",
                condition=IfCondition(LaunchConfiguration("enable_robot_execution")),
                parameters=[
                    {
                        "execute_motion": True,
                        "target_cell": LaunchConfiguration("target_cell"),
                    }
                ],
                output="screen",
            ),
        ]
    )
