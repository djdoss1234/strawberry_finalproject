"""Estimate a cultivation panel frame from aligned RGB-D observations only."""

import argparse
from pathlib import Path

import cv2
import numpy as np
import yaml

from strawberry_motion.visualization.alignment_overlay import (
    CrosshairStyle,
    draw_alignment_overlay,
)


def _rotation_x(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def _rotation_y(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)


def _rotation_z(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def _transform(xyz, rpy, joint_angle=0.0) -> np.ndarray:
    matrix = np.eye(4)
    matrix[:3, 3] = xyz
    fixed = _rotation_z(rpy[2]) @ _rotation_y(rpy[1]) @ _rotation_x(rpy[0])
    matrix[:3, :3] = fixed @ _rotation_z(joint_angle)
    return matrix


def e0509_tcp_fk(joints_rad) -> np.ndarray:
    """Return the same TCP transform used by the mini-project calibration path."""
    transform = np.eye(4)
    transform = transform @ _transform([0, 0, 0.2045], [0, 0, 0], joints_rad[0])
    transform = transform @ _transform([0, 0, 0], [0, -np.pi / 2, -np.pi / 2], joints_rad[1])
    transform = transform @ _transform([0.373, 0, 0], [0, 0, np.pi / 2], joints_rad[2])
    transform = transform @ _transform([0, -0.373, 0], [np.pi / 2, 0, 0], joints_rad[3])
    transform = transform @ _transform([0, 0, 0], [-np.pi / 2, 0, 0], joints_rad[4])
    transform = transform @ _transform([0, -0.1725, 0], [np.pi / 2, 0, 0], joints_rad[5])
    return transform @ _transform([0, 0, 0], [np.pi, -np.pi / 2, 0])


def estimate_panel_transform(
    origin_base: np.ndarray, right_base: np.ndarray, up_base: np.ndarray
) -> np.ndarray:
    """Build a panel pose with +X image-right, +Y image-up, +Z toward camera."""
    x_axis = right_base - origin_base
    x_axis = x_axis / np.linalg.norm(x_axis)
    y_axis = up_base - origin_base
    y_axis = y_axis - x_axis * np.dot(y_axis, x_axis)
    y_axis = y_axis / np.linalg.norm(y_axis)
    z_axis = np.cross(x_axis, y_axis)
    z_axis = z_axis / np.linalg.norm(z_axis)
    result = np.eye(4)
    result[:3, :3] = np.column_stack([x_axis, y_axis, z_axis])
    result[:3, 3] = origin_base
    return result


def parse_args(args=None):
    from ament_index_python.packages import get_package_share_directory

    package_share = Path(get_package_share_directory("strawberry_motion"))
    parser = argparse.ArgumentParser(
        description="Read depth at the aligned panel origin and print base_link transform."
    )
    parser.add_argument("--calibration-file", required=True)
    parser.add_argument(
        "--recorded-poses-file",
        default=str(package_share / "config" / "recorded_poses.yaml"),
    )
    parser.add_argument("--sample-offset-px", type=int, default=100)
    parser.add_argument("--patch-radius-px", type=int, default=3)
    return parser.parse_args(args)


def median_depth(depth_frame, u: int, v: int, radius: int):
    depths = []
    for py in range(v - radius, v + radius + 1):
        for px in range(u - radius, u + radius + 1):
            distance = depth_frame.get_distance(px, py)
            if 0.05 < distance < 3.0:
                depths.append(distance)
    return float(np.median(depths)) if depths else None


def depth_pixel_to_base(rs, depth_frame, intrinsics, t_base_camera, u, v, radius):
    """Return an observed point in base_link, or None if depth is invalid."""
    depth = median_depth(depth_frame, u, v, radius)
    if depth is None:
        return None
    point_camera = rs.rs2_deproject_pixel_to_point(intrinsics, [u, v], depth)
    homogeneous = np.array([*point_camera, 1.0], dtype=float)
    return (t_base_camera @ homogeneous)[:3]


def print_panel_transform(rs, depth_frame, intrinsics, t_base_camera, options):
    width, height = intrinsics.width, intrinsics.height
    center = (width // 2, height // 2)
    pixels = {
        "origin": center,
        "right": (center[0] + options.sample_offset_px, center[1]),
        "up": (center[0], center[1] - options.sample_offset_px),
    }
    points_base = {}
    for name, (u, v) in pixels.items():
        point_base = depth_pixel_to_base(
            rs, depth_frame, intrinsics, t_base_camera, u, v, options.patch_radius_px
        )
        if point_base is None:
            print("No valid depth for %s sample; keep camera still and retry." % name)
            return
        points_base[name] = point_base

    panel = estimate_panel_transform(
        points_base["origin"], points_base["right"], points_base["up"]
    )
    print("base_link -> cultivation_panel candidate (meters):")
    print(np.array2string(panel, precision=6, suppress_small=True))
    print("translation_m:", np.array2string(panel[:3, 3], precision=6))
    print("This is read-only capture output; do not use for motion until RViz validation.")


def main(args=None) -> None:
    import pyrealsense2 as rs

    options = parse_args(args)
    calibration = np.load(options.calibration_file)
    with Path(options.recorded_poses_file).open("r", encoding="utf-8") as stream:
        recorded = yaml.safe_load(stream)["overview_alignment"]
    joints_deg = recorded["joint_position_deg"]
    joints_rad = np.radians([joints_deg["j%d" % i] for i in range(1, 7)])
    t_base_camera = e0509_tcp_fk(joints_rad) @ calibration["T_cam_to_gripper"]

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    pipeline.start(config)
    align = rs.align(rs.stream.color)
    style = CrosshairStyle()
    print("Read-only panel capture viewer. Align at the saved overview pose; press p to print transform, q to quit.")
    try:
        while True:
            frames = align.process(pipeline.wait_for_frames())
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue
            image = draw_alignment_overlay(np.asanyarray(color_frame.get_data()), style)
            cv2.putText(
                image,
                "p: print panel TF candidate (read-only) | q: quit",
                (12, image.shape[0] - 14),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
            cv2.imshow("Read-only Panel Frame Capture", image)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("p"):
                intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
                print_panel_transform(rs, depth_frame, intrinsics, t_base_camera, options)
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
