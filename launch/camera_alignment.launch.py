from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    input_topic = LaunchConfiguration("input_topic")
    output_topic = LaunchConfiguration("output_topic")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "input_topic",
                default_value="/camera/camera/color/image_raw",
                description="RGB image topic used for overview alignment",
            ),
            DeclareLaunchArgument(
                "output_topic",
                default_value="/strawberry/alignment/overlay_image",
                description="Image topic carrying the crosshair overlay",
            ),
            Node(
                package="strawberry_motion",
                executable="camera_alignment_node",
                name="camera_alignment_node",
                output="screen",
                parameters=[
                    {
                        "input_topic": input_topic,
                        "output_topic": output_topic,
                    }
                ],
            ),
        ]
    )
