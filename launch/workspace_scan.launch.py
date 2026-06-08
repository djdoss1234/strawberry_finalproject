"""Launch workspace scan visualization; robot execution is opt-in and locked.

By default this launch starts TF, markers, scan-pose preview and RViz only.
Even with enable_robot_execution:=true, the executor requires an explicit
Trigger request. Full traversal requires an authorized collision-aware candidate
config; single-cell validation can be enabled with manual_validation_mode:=true.
Monitor progress:
  ros2 topic echo /strawberry/scan/status
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("strawberry_motion")
    moveit_pkg = get_package_share_directory("e0509_gripper_moveit_config")
    rviz_config = Path(pkg) / "rviz" / "workspace_exploration.rviz"
    registration_file = Path(pkg) / "config" / "panel_registration.yaml"
    moveit_launch = Path(moveit_pkg) / "launch" / "demo.launch.py"

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
                description="Initial physical validation permits one explicitly selected root/nw/root/ne/root/se/root/sw cell only.",
            ),
            DeclareLaunchArgument(
                "manual_validation_mode",
                default_value="false",
                description="Allow one explicit single-cell MoveJoint validation while automated traversal remains locked.",
            ),
            DeclareLaunchArgument(
                "scan_movej_vel_deg_s",
                default_value="60.0",
                description="MoveJoint velocity for scan pose validation in deg/s.",
            ),
            DeclareLaunchArgument(
                "scan_movej_acc_deg_s2",
                default_value="90.0",
                description="MoveJoint acceleration for scan pose validation in deg/s^2.",
            ),
            DeclareLaunchArgument(
                "overview_return_vel_deg_s",
                default_value="60.0",
                description="MoveJoint velocity for return-to-overview in deg/s.",
            ),
            DeclareLaunchArgument(
                "overview_return_acc_deg_s2",
                default_value="90.0",
                description="MoveJoint acceleration for return-to-overview in deg/s^2.",
            ),
            DeclareLaunchArgument(
                "movej_service_timeout_sec",
                default_value="30.0",
                description="Seconds to wait for MoveJoint service response before relying on joint-state arrival verification.",
            ),
            DeclareLaunchArgument(
                "enable_pick_integration",
                default_value="true",
                description="Forward detected pick poses to the pick executor after each scan dwell.",
            ),
            DeclareLaunchArgument(
                "scan_dwell_sec",
                default_value="5.0",
                description="Seconds to collect stable perception targets after reaching each scan pose.",
            ),
            DeclareLaunchArgument(
                "return_to_overview_at_end",
                default_value="true",
                description="Return to verified overview pose after scan sequence. Set false for harvest/VLA recovery experiments.",
            ),
            DeclareLaunchArgument(
                "enable_runtime_curobo_preview",
                default_value="false",
                description="Compute and log a cuRobo runtime plan before each cell move; execution still uses YAML MoveJoint.",
            ),
            DeclareLaunchArgument(
                "runtime_curobo_preview_retries",
                default_value="2",
                description="cuRobo planning retries for runtime preview logging.",
            ),
            DeclareLaunchArgument(
                "enable_fusion_detection",
                default_value="false",
                description="Start strawberry_fusion_node (seg+pose dual YOLO). Requires RealSense + model weights.",
            ),
            DeclareLaunchArgument(
                "fusion_seg_model",
                default_value="~/Downloads/share_yolo/share_yolo/strawberry_seg_best.pt",
                description="Path to seg YOLO weights (ripe/unripe/sick).",
            ),
            DeclareLaunchArgument(
                "fusion_pose_model",
                default_value="~/Downloads/share_yolo/share_yolo/strawberry_pose_best.pt",
                description="Path to pose YOLO weights (3 stem keypoints).",
            ),
            DeclareLaunchArgument(
                "fusion_show_display",
                default_value="true",
                description="Show OpenCV fusion visualization window.",
            ),
            DeclareLaunchArgument(
                "enable_moveit",
                default_value="false",
                description="Also start MoveIt move_group for parallel planning-scene/trajectory checks.",
            ),
            DeclareLaunchArgument(
                "moveit_rviz",
                default_value="false",
                description="Start MoveIt's own RViz. Usually false because workspace RViz is already launched.",
            ),
            DeclareLaunchArgument(
                "moveit_environment",
                default_value="false",
                description="Start e0509_gripper_description environment_visualizer with MoveIt.",
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
                package="e0509_gripper_description",
                executable="strawberry_fusion_node.py",
                name="strawberry_fusion_node",
                condition=IfCondition(LaunchConfiguration("enable_fusion_detection")),
                parameters=[
                    {
                        "seg_model": LaunchConfiguration("fusion_seg_model"),
                        "pose_model": LaunchConfiguration("fusion_pose_model"),
                        "show_display": LaunchConfiguration("fusion_show_display"),
                    }
                ],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(moveit_launch)),
                condition=IfCondition(LaunchConfiguration("enable_moveit")),
                launch_arguments={
                    "rviz": LaunchConfiguration("moveit_rviz"),
                    "environment": LaunchConfiguration("moveit_environment"),
                    "fake_joint_gui": "false",
                }.items(),
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
                        "manual_validation_mode": LaunchConfiguration("manual_validation_mode"),
                        "scan_movej_vel_deg_s": LaunchConfiguration("scan_movej_vel_deg_s"),
                        "scan_movej_acc_deg_s2": LaunchConfiguration("scan_movej_acc_deg_s2"),
                        "overview_return_vel_deg_s": LaunchConfiguration(
                            "overview_return_vel_deg_s"
                        ),
                        "overview_return_acc_deg_s2": LaunchConfiguration(
                            "overview_return_acc_deg_s2"
                        ),
                        "movej_service_timeout_sec": LaunchConfiguration(
                            "movej_service_timeout_sec"
                        ),
                        "enable_pick_integration": LaunchConfiguration(
                            "enable_pick_integration"
                        ),
                        "scan_dwell_sec": LaunchConfiguration("scan_dwell_sec"),
                        "return_to_overview_at_end": LaunchConfiguration(
                            "return_to_overview_at_end"
                        ),
                        "enable_runtime_curobo_preview": LaunchConfiguration(
                            "enable_runtime_curobo_preview"
                        ),
                        "runtime_curobo_preview_retries": LaunchConfiguration(
                            "runtime_curobo_preview_retries"
                        ),
                    }
                ],
                output="screen",
            ),
        ]
    )
