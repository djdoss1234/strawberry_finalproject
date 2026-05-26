"""Low-latency RealSense viewer for manually aligning the workspace origin."""

import argparse
import time

import cv2
import numpy as np

from strawberry_motion.visualization.alignment_overlay import (
    CrosshairStyle,
    draw_alignment_overlay,
)


WINDOW_NAME = "Strawberry Overview Alignment - press q to quit"


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
        help="Disabled: robot motion requires planner and joint-limit safety validation.",
    )
    return parser.parse_args(args)


def reject_unsafe_motion_option(options: argparse.Namespace) -> None:
    """Fail closed if an installed command attempts the withdrawn motion feature."""
    if options.enable_robot_control:
        raise RuntimeError(
            "Robot control is disabled: Cartesian alignment motion was withdrawn "
            "until joint-limit, IK branch, and collision validation are enforced."
        )


def main(args=None) -> None:
    options = parse_args(args)
    reject_unsafe_motion_option(options)

    import pyrealsense2 as rs

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
            cv2.imshow(WINDOW_NAME, display)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
