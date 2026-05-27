"""Launch the quadtree workspace markers with a capture-ready RViz view."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml


def generate_launch_description() -> LaunchDescription:
    rviz_config = (
        Path(get_package_share_directory("strawberry_motion"))
        / "rviz"
        / "workspace_exploration.rviz"
    )
    registration_file = (
        Path(get_package_share_directory("strawberry_motion"))
        / "config"
        / "panel_registration.yaml"
    )
    with registration_file.open("r", encoding="utf-8") as stream:
        registration = yaml.safe_load(stream)["panel_registration"]
    translation = registration["transform"]["translation_m"]
    rotation = registration["transform"]["rotation_xyzw"]
    return LaunchDescription(
        [
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="workspace_visualization_frame_publisher",
                arguments=[
                    "--x",
                    str(translation["x"]),
                    "--y",
                    str(translation["y"]),
                    "--z",
                    str(translation["z"]),
                    "--qx",
                    str(rotation["x"]),
                    "--qy",
                    str(rotation["y"]),
                    "--qz",
                    str(rotation["z"]),
                    "--qw",
                    str(rotation["w"]),
                    "--frame-id",
                    registration["parent_frame_id"],
                    "--child-frame-id",
                    registration["frame_id"],
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
        ]
    )
