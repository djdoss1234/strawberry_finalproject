"""Read-only RGB-D landmark capture for panel registration error measurement."""

import argparse
from pathlib import Path

import cv2
import numpy as np
import yaml

from strawberry_motion.registration.panel_frame_capture import (
    depth_pixel_to_base,
    e0509_tcp_fk,
)
from strawberry_motion.registration.panel_registration_validator import (
    DEFAULT_LANDMARKS_PANEL_M,
    evaluate_landmarks,
)
from strawberry_motion.visualization.alignment_overlay import (
    CrosshairStyle,
    draw_alignment_overlay,
)


WINDOW_NAME = "Panel Landmark Capture - click landmark | s: save | r: reset | q: quit"
ORDER = ["origin_crossing", "paper_inner_nw", "paper_inner_ne", "paper_inner_sw", "paper_inner_se"]


def parse_args(args=None):
    from ament_index_python.packages import get_package_share_directory

    share = Path(get_package_share_directory("strawberry_motion"))
    parser = argparse.ArgumentParser(description="Read-only panel landmark capture.")
    parser.add_argument("--calibration-file", required=True)
    parser.add_argument(
        "--recorded-poses-file", default=str(share / "config" / "recorded_poses.yaml")
    )
    parser.add_argument(
        "--panel-registration-file", default=str(share / "config" / "panel_registration.yaml")
    )
    parser.add_argument("--output", required=True)
    parser.add_argument("--patch-radius-px", type=int, default=3)
    parser.add_argument("--fast-display", action="store_true",
                        help="Show color frames without per-frame depth alignment; align depth only on click.")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    return parser.parse_args(args)


def _base_camera(options):
    calibration = np.load(options.calibration_file)
    with Path(options.recorded_poses_file).open("r", encoding="utf-8") as stream:
        pose = yaml.safe_load(stream)["overview_alignment"]["joint_position_deg"]
    joints = np.radians([pose[f"j{i}"] for i in range(1, 7)])
    return e0509_tcp_fk(joints) @ calibration["T_cam_to_gripper"]


def main(args=None):
    import pyrealsense2 as rs

    options = parse_args(args)
    t_base_camera = _base_camera(options)
    with Path(options.panel_registration_file).open("r", encoding="utf-8") as stream:
        registration = yaml.safe_load(stream)["panel_registration"]
    transform_matrix = registration["transform"]["matrix"]
    observations = {}
    pending_click = [None]

    def mouse_callback(event, x, y, flags, param):
        del flags, param
        if event == cv2.EVENT_LBUTTONDOWN:
            pending_click[0] = (x, y)

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, options.width, options.height, rs.format.bgr8, options.fps)
    config.enable_stream(rs.stream.depth, options.width, options.height, rs.format.z16, options.fps)
    pipeline.start(config)
    align = rs.align(rs.stream.color)
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(WINDOW_NAME, mouse_callback)
    print("Read-only capture. Keep the manually aligned overview pose fixed.")
    print("Click in order: origin crossing, inner-paper NW, NE, SW, SE.")
    print("IMPORTANT: click WHITE PAPER just inside the tape corner, not the black tape.")

    try:
        while True:
            raw_frames = pipeline.wait_for_frames()
            frames = raw_frames if options.fast_display else align.process(raw_frames)
            color = frames.get_color_frame()
            if not color:
                continue
            image = draw_alignment_overlay(np.asanyarray(color.get_data()), CrosshairStyle())
            next_id = ORDER[len(observations)] if len(observations) < len(ORDER) else "complete"
            cv2.putText(
                image, f"NEXT: {next_id} | WHITE PAPER INSIDE TAPE ONLY | s: save | r: reset",
                (12, image.shape[0] - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.48,
                (0, 255, 0), 2, cv2.LINE_AA,
            )
            cv2.putText(
                image, "DO NOT CLICK BLACK TAPE: DEPTH MAY JUMP",
                (12, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.62,
                (0, 0, 255), 2, cv2.LINE_AA,
            )
            for landmark_id, item in observations.items():
                u, v = item["pixel_uv"]
                cv2.circle(image, (u, v), 5, (0, 180, 255), -1)
                cv2.putText(image, landmark_id, (u + 6, v - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 180, 255), 1)
            if pending_click[0] is not None and len(observations) < len(ORDER):
                u, v = pending_click.pop()
                pending_click.append(None)
                click_frames = align.process(raw_frames if options.fast_display else frames)
                depth = click_frames.get_depth_frame()
                if not depth:
                    print("Depth frame unavailable at clicked point; click the same landmark again.")
                    continue
                intrinsics = depth.profile.as_video_stream_profile().intrinsics
                point = depth_pixel_to_base(
                    rs, depth, intrinsics, t_base_camera, u, v, options.patch_radius_px
                )
                if point is None:
                    print("Depth invalid at clicked point; click the same landmark again.")
                else:
                    landmark_id = ORDER[len(observations)]
                    observations[landmark_id] = {
                        "point_panel_m": DEFAULT_LANDMARKS_PANEL_M[landmark_id],
                        "pixel_uv": [int(u), int(v)],
                        "observed_base_m": [float(value) for value in point.tolist()],
                    }
                    print(f"Captured {landmark_id}: pixel=({u}, {v}) base={point}")
            cv2.imshow(WINDOW_NAME, image)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("r"):
                observations.clear()
                print("Capture reset.")
            if key == ord("s"):
                if len(observations) != len(ORDER):
                    print(f"Need all five landmarks before saving; have {len(observations)}.")
                    continue
                evaluation = evaluate_landmarks(transform_matrix, observations)
                output = {
                    "panel_registration_validation": {
                        "capture_method": "read_only_realsense_landmark_clicks_at_overview_pose",
                        "panel_registration_source": str(options.panel_registration_file),
                        "landmark_definition": "tape_crossing_and_white_paper_inner_corners_approx_20mm_inset",
                        "observations": observations,
                        "evaluation": evaluation,
                        "use_for_automated_motion": False,
                    }
                }
                output_path = Path(options.output).expanduser()
                output_path.parent.mkdir(parents=True, exist_ok=True)
                with output_path.open("w", encoding="utf-8") as stream:
                    yaml.safe_dump(output, stream, sort_keys=False, allow_unicode=True)
                print(f"Saved: {output_path}")
                print(f"RMS={evaluation['rms_error_mm']} mm MAX={evaluation['max_error_mm']} mm")
                print(f"MAX plane offset={evaluation['max_abs_plane_offset_mm']} mm")
                print("This evidence does not authorize robot motion.")
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
