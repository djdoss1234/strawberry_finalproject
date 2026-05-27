"""Display the measured panel refit candidate in RViz without robot motion."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("strawberry_motion"))
    with (share / "config" / "panel_registration_refit_candidate.yaml").open(
        "r", encoding="utf-8"
    ) as stream:
        registration = yaml.safe_load(stream)["panel_registration_refit_candidate"]
    translation = registration["transform"]["translation_m"]
    rotation = registration["transform"]["rotation_xyzw"]
    return LaunchDescription(
        [
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="panel_refit_candidate_frame_publisher",
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
                package="rviz2",
                executable="rviz2",
                name="panel_refit_review_rviz",
                arguments=["-d", str(share / "rviz" / "workspace_exploration.rviz")],
                output="screen",
            ),
        ]
    )
