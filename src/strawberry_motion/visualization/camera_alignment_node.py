"""Publish an RGB camera stream with workspace alignment guides overlaid."""

from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from strawberry_motion.visualization.alignment_overlay import (
    CrosshairStyle,
    draw_alignment_overlay,
)


class CameraAlignmentNode(Node):
    """Overlay a crosshair for matching the camera center to the tape intersection."""

    def __init__(self) -> None:
        super().__init__("camera_alignment_node")
        self.declare_parameter("input_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("output_topic", "/strawberry/alignment/overlay_image")
        self.declare_parameter("crosshair_length_px", 60)
        self.declare_parameter("line_thickness_px", 2)
        self.declare_parameter("guide_margin_ratio", 0.08)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.style = CrosshairStyle(
            crosshair_length_px=int(self.get_parameter("crosshair_length_px").value),
            line_thickness_px=int(self.get_parameter("line_thickness_px").value),
            guide_margin_ratio=float(self.get_parameter("guide_margin_ratio").value),
        )
        self.publisher = self.create_publisher(Image, output_topic, qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            Image, input_topic, self._on_image, qos_profile_sensor_data
        )
        self._logged_first_frame = False
        self.get_logger().info(
            "Camera alignment overlay ready: input=%s, output=%s"
            % (input_topic, output_topic)
        )

    def _on_image(self, message: Image) -> None:
        image_bgr = image_message_to_bgr(message)
        if image_bgr is None:
            self.get_logger().warning(
                "Unsupported or malformed image: encoding=%s, size=%dx%d, step=%d"
                % (message.encoding, message.width, message.height, message.step)
            )
            return

        overlay_bgr = draw_alignment_overlay(image_bgr, self.style)
        self.publisher.publish(bgr_to_image_message(overlay_bgr, message))
        if not self._logged_first_frame:
            self.get_logger().info(
                "Publishing alignment overlay for %dx%d image; match tape crossing to image center"
                % (message.width, message.height)
            )
            self._logged_first_frame = True


def image_message_to_bgr(message: Image) -> Optional[np.ndarray]:
    """Convert common ROS image encodings without depending on cv_bridge."""
    channels_by_encoding = {
        "bgr8": 3,
        "rgb8": 3,
        "bgra8": 4,
        "rgba8": 4,
        "mono8": 1,
    }
    channels = channels_by_encoding.get(message.encoding.lower())
    if channels is None or message.height <= 0 or message.width <= 0:
        return None

    row_bytes = message.width * channels
    if message.step < row_bytes or len(message.data) < message.step * message.height:
        return None

    rows = np.frombuffer(message.data, dtype=np.uint8, count=message.step * message.height)
    pixels = rows.reshape((message.height, message.step))[:, :row_bytes]
    if channels == 1:
        image = pixels.reshape((message.height, message.width))
    else:
        image = pixels.reshape((message.height, message.width, channels))

    encoding = message.encoding.lower()
    if encoding == "bgr8":
        return image.copy()
    if encoding == "rgb8":
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if encoding == "bgra8":
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    if encoding == "rgba8":
        return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
    return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)


def bgr_to_image_message(image_bgr: np.ndarray, source: Image) -> Image:
    """Create a bgr8 ROS message while retaining timestamp and camera frame."""
    output = Image()
    output.header = source.header
    output.height, output.width = image_bgr.shape[:2]
    output.encoding = "bgr8"
    output.is_bigendian = 0
    output.step = output.width * 3
    output.data = image_bgr.tobytes()
    return output


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraAlignmentNode()
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
