#!/usr/bin/env python3
"""One-time StrawberryFusionNode parameter declare/load helper.

Runs exactly once, from StrawberryFusionNode.__init__, with no runtime
behavior to encapsulate — kept as a plain function rather than a client/
executor class. Moved verbatim out of __init__; parameter names, defaults,
and computed values are unchanged.
"""

import os


def _as_bool(value) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def declare_and_load_fusion_params(node):
    """Declare and read all fusion-node ROS parameters, assigning them as
    node._x attributes (same names/defaults as the original __init__ body).
    Returns (seg_path, pose_path, calib_path) for the caller to load
    models/calibration with."""
    node.declare_parameter(
        "seg_model",
        "~/Downloads/share_yolo/share_yolo/strawberry_seg_best.pt")
    node.declare_parameter(
        "pose_model",
        "~/Downloads/share_yolo/share_yolo/strawberry_pose_best.pt")
    node.declare_parameter(
        "calib_npz",
        "~/doosan_ws/src/e0509_gripper_description/config/calibration_eye_in_hand_1.npz")
    node.declare_parameter("yolo_conf",    0.25)
    node.declare_parameter("kp_conf_min",  0.40)   # keypoint visibility threshold
    node.declare_parameter("pick_kp_conf_min", 0.60)
    node.declare_parameter("require_all_stem_keypoints", True)
    node.declare_parameter("stem_segment_min_m", 0.005)
    node.declare_parameter("stem_segment_max_m", 0.100)
    node.declare_parameter("stem_total_max_m", 0.160)
    # KP0 바로 위는 과실/잎과 닿기 쉽고, 꺾인 줄기는 전체 chord를 따르면
    # 측방으로 빗나간다. KP0->KP1 국소 구간의 80% 지점을 목표로 삼는다.
    # 아래 계산에서 segment 길이의 0.8배로 cap되므로 0.080m는 상한값이다.
    node.declare_parameter("stem_grasp_offset_from_kp0_m", 0.080)
    node.declare_parameter("stem_grasp_direction_mode", "kp0_to_kp1")
    node.declare_parameter("grasp_target_base_z_trim_m", 0.000)
    node.declare_parameter("infer_every",  3)       # run inference every N camera frames
    node.declare_parameter("stable_hits_required", 4)
    node.declare_parameter("target_position_window_size", 9)
    node.declare_parameter("target_position_min_samples", 7)
    node.declare_parameter("target_position_max_spread_m", 0.012)
    node.declare_parameter("track_match_distance_m", 0.035)
    node.declare_parameter("track_ttl_sec", 1.5)
    node.declare_parameter("publish_period_sec", 1.0)
    node.declare_parameter("target_lock_enabled", True)
    node.declare_parameter("target_lock_ttl_sec", 3.0)
    node.declare_parameter("target_switch_distance_m", 0.090)
    node.declare_parameter("pick_target_min_z_m", 0.0)
    node.declare_parameter("pick_target_max_z_m", 1.05)
    node.declare_parameter("prefer_lower_z_target", False)
    node.declare_parameter("show_display", True)
    node.declare_parameter("show_cell_grid", True)

    seg_path   = os.path.expanduser(node.get_parameter("seg_model").value)
    pose_path  = os.path.expanduser(node.get_parameter("pose_model").value)
    calib_path = os.path.expanduser(node.get_parameter("calib_npz").value)
    node._conf    = node.get_parameter("yolo_conf").value
    node._kp_min  = node.get_parameter("kp_conf_min").value
    node._pick_kp_min = float(node.get_parameter("pick_kp_conf_min").value)
    node._require_all_stem_kps = bool(
        node.get_parameter("require_all_stem_keypoints").value)
    node._stem_segment_min_m = float(
        node.get_parameter("stem_segment_min_m").value)
    node._stem_segment_max_m = float(
        node.get_parameter("stem_segment_max_m").value)
    node._stem_total_max_m = float(
        node.get_parameter("stem_total_max_m").value)
    node._stem_grasp_offset_m = max(
        0.0, float(node.get_parameter("stem_grasp_offset_from_kp0_m").value))
    node._stem_grasp_direction_mode = str(
        node.get_parameter("stem_grasp_direction_mode").value)
    node._grasp_target_base_z_trim_m = float(
        node.get_parameter("grasp_target_base_z_trim_m").value)
    node._infer_n = max(1, node.get_parameter("infer_every").value)
    node._stable_hits = max(1, int(node.get_parameter("stable_hits_required").value))
    node._position_window_size = max(
        3, int(node.get_parameter("target_position_window_size").value))
    node._position_min_samples = min(
        node._position_window_size,
        max(3, int(node.get_parameter("target_position_min_samples").value)),
    )
    node._position_max_spread_m = float(
        node.get_parameter("target_position_max_spread_m").value)
    node._track_match_dist = float(node.get_parameter("track_match_distance_m").value)
    node._track_ttl_sec = float(node.get_parameter("track_ttl_sec").value)
    node._publish_period_sec = float(node.get_parameter("publish_period_sec").value)
    node._target_lock_enabled = _as_bool(node.get_parameter("target_lock_enabled").value)
    node._target_lock_ttl_sec = float(node.get_parameter("target_lock_ttl_sec").value)
    node._target_switch_dist = float(node.get_parameter("target_switch_distance_m").value)
    node._pick_target_min_z_m = float(node.get_parameter("pick_target_min_z_m").value)
    node._pick_target_max_z_m = float(node.get_parameter("pick_target_max_z_m").value)
    node._prefer_lower_z_target = _as_bool(
        node.get_parameter("prefer_lower_z_target").value)
    node._display = node.get_parameter("show_display").value
    node._show_cell_grid = _as_bool(node.get_parameter("show_cell_grid").value)
    return seg_path, pose_path, calib_path
