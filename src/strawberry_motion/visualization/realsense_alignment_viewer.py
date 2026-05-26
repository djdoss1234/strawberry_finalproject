"""Low-latency RealSense viewer for manually aligning the workspace origin."""

import argparse
import time
from typing import Dict, Optional, Sequence, Tuple

import cv2
import numpy as np

from strawberry_motion.visualization.alignment_overlay import (
    CrosshairStyle,
    draw_alignment_overlay,
)


WINDOW_NAME = "Strawberry Overview Alignment - press q to quit"
BASE_AXIS_KEYS: Dict[int, Tuple[str, Tuple[float, float, float]]] = {
    ord("a"): ("LEFT  -X", (-1.0, 0.0, 0.0)),
    ord("d"): ("RIGHT +X", (1.0, 0.0, 0.0)),
    ord("w"): ("UP    +Z", (0.0, 0.0, 1.0)),
    ord("s"): ("DOWN  -Z", (0.0, 0.0, -1.0)),
    ord("r"): ("DEPTH +Y", (0.0, 1.0, 0.0)),
    ord("f"): ("DEPTH -Y", (0.0, -1.0, 0.0)),
}


def parse_args(args=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Display a low-latency RealSense RGB view with a center crosshair."
    )
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--crosshair-length-px", type=int, default=60)
    parser.add_argument("--guide-margin-ratio", type=float, default=0.08)
    parser.add_argument(
        "--enable-robot-control",
        action="store_true",
        help="Enable safe step Cartesian movement through Doosan MoveLine.",
    )
    parser.add_argument("--step-mm", type=float, default=5.0)
    parser.add_argument("--linear-velocity-mm-s", type=float, default=15.0)
    parser.add_argument("--linear-acceleration-mm-s2", type=float, default=30.0)
    return parser.parse_args(args)


def delta_for_key(key: int, step_mm: float) -> Optional[Tuple[str, Sequence[float]]]:
    """Map an alignment key to a base-frame relative Cartesian displacement."""
    command = BASE_AXIS_KEYS.get(key)
    if command is None:
        return None
    label, unit_delta = command
    delta = [component * step_mm for component in unit_delta] + [0.0, 0.0, 0.0]
    return label, delta


class CartesianStepControl:
    """Issue bounded relative TCP translations while the alignment image is open."""

    def __init__(self, velocity: float, acceleration: float) -> None:
        import rclpy
        from dsr_msgs2.srv import GetCurrentPosx, MoveLine

        self.rclpy = rclpy
        self.GetCurrentPosx = GetCurrentPosx
        self.MoveLine = MoveLine
        rclpy.init()
        self.node = rclpy.create_node("alignment_cartesian_step_control")
        self.move_client = self.node.create_client(MoveLine, "/dsr01/motion/move_line")
        self.pose_client = self.node.create_client(
            GetCurrentPosx, "/dsr01/aux_control/get_current_posx"
        )
        self.velocity = max(1.0, velocity)
        self.acceleration = max(1.0, acceleration)
        if not self.move_client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("/dsr01/motion/move_line service is unavailable")
        print("Robot step control enabled: BASE frame X/Z/Y translation only.")
        print("Start with a small step and confirm key direction before approaching the board.")

    def move_relative(self, delta: Sequence[float]) -> bool:
        request = self.MoveLine.Request()
        request.pos = [float(value) for value in delta]
        request.vel = [self.velocity, 5.0]
        request.acc = [self.acceleration, 10.0]
        request.time = 0.0
        request.radius = 0.0
        request.ref = 0  # DR_BASE
        request.mode = 1  # DR_MV_MOD_REL
        request.blend_type = 0
        request.sync_type = 0  # Complete one bounded step before accepting the next.
        future = self.move_client.call_async(request)
        self.rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        response = future.result() if future.done() else None
        return bool(response and response.success)

    def print_tcp(self) -> None:
        if not self.pose_client.wait_for_service(timeout_sec=0.5):
            print("TCP service unavailable")
            return
        request = self.GetCurrentPosx.Request()
        request.ref = 0
        future = self.pose_client.call_async(request)
        self.rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.0)
        response = future.result() if future.done() else None
        if not response or not response.success or not response.task_pos_info:
            print("Could not read current TCP pose")
            return
        tcp = response.task_pos_info[0].data[:6]
        print("TCP(base): [%s]" % ", ".join("%.3f" % value for value in tcp))

    def shutdown(self) -> None:
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()


def main(args=None) -> None:
    import pyrealsense2 as rs

    options = parse_args(args)
    step_control = None
    if options.enable_robot_control:
        step_control = CartesianStepControl(
            options.linear_velocity_mm_s, options.linear_acceleration_mm_s2
        )
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(
        rs.stream.color,
        options.width,
        options.height,
        rs.format.bgr8,
        options.fps,
    )
    style = CrosshairStyle(
        crosshair_length_px=options.crosshair_length_px,
        guide_margin_ratio=options.guide_margin_ratio,
    )

    pipeline.start(config)
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    last_time = time.monotonic()
    shown_frames = 0
    display_fps = 0.0
    print("Low-latency alignment viewer started. Align the yellow crosshair to the tape crossing.")
    print("Press q or ESC to close.")
    if step_control is not None:
        print("Robot keys: A/D left-right BASE X, W/S up-down BASE Z, R/F BASE Y depth, P TCP.")

    try:
        while True:
            frame = pipeline.wait_for_frames().get_color_frame()
            if not frame:
                continue

            image_bgr = np.asanyarray(frame.get_data())
            display = draw_alignment_overlay(image_bgr, style)
            shown_frames += 1
            now = time.monotonic()
            elapsed = now - last_time
            if elapsed >= 1.0:
                display_fps = shown_frames / elapsed
                shown_frames = 0
                last_time = now

            cv2.putText(
                display,
                "LIVE %.1f FPS | q: quit" % display_fps,
                (12, display.shape[0] - 14),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
            if step_control is not None:
                cv2.putText(
                    display,
                    "STEP %.1fmm | A/D:X  W/S:Z  R/F:Y  P:TCP"
                    % options.step_mm,
                    (12, display.shape[0] - 42),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
            cv2.imshow(WINDOW_NAME, display)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if step_control is not None and key == ord("p"):
                step_control.print_tcp()
            if step_control is not None:
                move = delta_for_key(key, options.step_mm)
                if move is not None:
                    label, delta = move
                    print("%s %.1f mm -> %s" % (label, options.step_mm, delta[:3]))
                    if not step_control.move_relative(delta):
                        print("MoveLine relative step failed; stop and check robot state.")
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        if step_control is not None:
            step_control.shutdown()


if __name__ == "__main__":
    main()
