#!/usr/bin/env python3
"""Grasp candidate policy helpers for the harvest planner."""

from dataclasses import dataclass

import numpy as np

from harvest_math import (
    quat_from_axis_angle,
    quat_multiply_wxyz,
    quat_rotate_vec,
)
from harvest_motion_params import (
    GRASP_QUAT_RETRY_VARIANTS,
    GRASP_RETRY_OFFSETS,
    LEFTMOST_GRASP_RETRY_OFFSETS,
    MEASURED_TCP_J3_GOOD_ENOUGH_DEG,
    MEASURED_TCP_MAX_APPROACH_CEILING_M,
    MEASURED_TCP_FINAL_STANDOFF_M,
    MEASURED_TCP_GRASP_QUAT_RETRY_VARIANTS,
    MEASURED_TCP_MIN_PRUNE_DEPTH_M,
    NW_HIGH_TARGET_J3_GOOD_ENOUGH_DEG,
    NW_HIGH_TARGET_GRASP_QUAT_RETRY_VARIANTS,
    NW_HIGH_TARGET_MIN_FLAT_BRANCH_J3_DEG,
    NW_HIGH_TARGET_PROBE_DEPTHS_M,
    PRE_APPROACH_OFFSET,
    WALL_QUAT_WXYZ,
)


@dataclass
class GraspSearchResult:
    """Selected grasp plan and metadata from the grasp probing loop."""

    ret_pre: object = None
    ret_grasp: object = None
    grasp_offset_m: float = None
    grasp_variant: object = None
    approach_dir: object = None
    grasp_quat: object = None
    pre_ee_pos: object = None
    grasp_ee_pos: object = None
    attempt_count: int = 0
    measured_best_depth_m: float = -1.0
    measured_best_j3_deg: float = None
    measured_best_alignment_deg: float = None

    @property
    def found(self) -> bool:
        return self.ret_pre is not None

    def apply_measured_best(self, measured_best):
        (
            self.ret_pre,
            self.ret_grasp,
            self.grasp_offset_m,
            self.grasp_variant,
            self.approach_dir,
            self.grasp_quat,
            self.pre_ee_pos,
            self.grasp_ee_pos,
        ) = measured_best

    def update_measured_best(self, depth_m: float, j3_deg: float,
                             alignment_deg: float, measured_best):
        self.measured_best_depth_m = depth_m
        self.measured_best_j3_deg = j3_deg
        self.measured_best_alignment_deg = alignment_deg
        return measured_best

    def select_legacy_grasp(self, ret_pre, ret_grasp, grasp_offset_m,
                            grasp_variant, approach_dir, grasp_quat,
                            pre_ee_pos, grasp_ee_pos):
        self.ret_pre = ret_pre
        self.ret_grasp = ret_grasp
        self.grasp_offset_m = grasp_offset_m
        self.grasp_variant = grasp_variant
        self.approach_dir = approach_dir
        self.grasp_quat = grasp_quat
        self.pre_ee_pos = pre_ee_pos.copy()
        self.grasp_ee_pos = grasp_ee_pos.copy()


def grasp_offsets_for_target(straw, measured_tcp_model: bool):
    if measured_tcp_model:
        return [MEASURED_TCP_FINAL_STANDOFF_M]
    if straw[0] > 0.25:
        return [-0.03, 0.0]
    if straw[0] < -0.30:
        return LEFTMOST_GRASP_RETRY_OFFSETS
    return GRASP_RETRY_OFFSETS


def grasp_quat_variants_for_target(measured_tcp_model: bool,
                                   is_nw_high_target: bool):
    if is_nw_high_target:
        return list(NW_HIGH_TARGET_GRASP_QUAT_RETRY_VARIANTS)
    if measured_tcp_model:
        return list(MEASURED_TCP_GRASP_QUAT_RETRY_VARIANTS)
    return list(GRASP_QUAT_RETRY_VARIANTS)


def variant_label(variant):
    frame, _axis, deg = variant
    if frame == "published_roll":
        return f"published_roll({deg:+.0f}deg)"
    return f"{deg:+.0f}deg"


def grasp_variant_pose(variant, straw, ee_to_tcp_offset_m: float,
                       measured_tcp_model: bool,
                       crane_z_offset_m: float):
    """Return `(q_retry, approach_dir, ee_pre)` for one grasp variant."""
    quat_frame, axis, quat_deg = variant
    if quat_frame == "published_roll":
        q_retry = axis
    else:
        q_delta = quat_from_axis_angle(axis, np.deg2rad(quat_deg))
        if quat_frame == "base":
            q_retry = quat_multiply_wxyz(q_delta, WALL_QUAT_WXYZ)
        else:
            q_retry = quat_multiply_wxyz(WALL_QUAT_WXYZ, q_delta)

    approach_dir = np.array(quat_rotate_vec(q_retry, [0.0, 0.0, 1.0]))
    ee_pre = np.array(straw, dtype=float) - (
        PRE_APPROACH_OFFSET + float(ee_to_tcp_offset_m)
    ) * approach_dir
    if measured_tcp_model and crane_z_offset_m > 0:
        ee_pre = ee_pre + np.array([0.0, 0.0, float(crane_z_offset_m)])
    return q_retry, approach_dir, ee_pre


def legacy_grasp_endpoint(straw, grasp_offset_m: float,
                          ee_to_tcp_offset_m: float, approach_dir):
    """Return legacy final grasp ee endpoint for a grasp offset."""
    return np.array(straw, dtype=float) - (
        float(grasp_offset_m) + float(ee_to_tcp_offset_m)
    ) * np.array(approach_dir, dtype=float)


def measured_tcp_probe_depths(requested_depth_m: float, is_nw_high_target: bool,
                              measured_best_depth_m: float):
    if is_nw_high_target:
        probe_depths = [
            d for d in NW_HIGH_TARGET_PROBE_DEPTHS_M
            if d <= requested_depth_m + 1e-6
        ]
    else:
        probe_depths = [requested_depth_m]
        for depth_m in [0.150, 0.130, 0.110, 0.090, 0.070, 0.060]:
            if 0.001 < depth_m < requested_depth_m - 0.005:
                probe_depths.append(depth_m)

    pruned = False
    if measured_best_depth_m >= MEASURED_TCP_MIN_PRUNE_DEPTH_M:
        probe_depths = [
            d for d in probe_depths
            if d <= measured_best_depth_m + 1e-6
        ]
        if not probe_depths:
            probe_depths = [measured_best_depth_m]
        pruned = True
    return probe_depths, pruned


def requested_measured_tcp_probe_depth(max_approach_m: float):
    return max(0.060, min(MEASURED_TCP_MAX_APPROACH_CEILING_M, float(max_approach_m)))


def measured_tcp_probe_log_message(probe_pruned: bool, measured_best_depth_m: float,
                                   probe_depths):
    if probe_pruned:
        return (
            "MEASURED_TCP_PROBE_PRUNED: existing best depth="
            f"{measured_best_depth_m*1000:.0f}mm; "
            f"next depths={[round(d*1000) for d in probe_depths]}mm"
        )
    if measured_best_depth_m > 0.0:
        return (
            "MEASURED_TCP_PROBE_NOT_PRUNED: existing best depth="
            f"{measured_best_depth_m*1000:.0f}mm < "
            f"{MEASURED_TCP_MIN_PRUNE_DEPTH_M*1000:.0f}mm minimum; "
            "later variants may still reach the proven 90mm TOOL finish branch"
        )
    return None


def measured_tcp_good_enough_j3_deg(is_nw_high_target: bool):
    return (
        NW_HIGH_TARGET_J3_GOOD_ENOUGH_DEG
        if is_nw_high_target else MEASURED_TCP_J3_GOOD_ENOUGH_DEG
    )


def should_stop_measured_variant_search(search_result: GraspSearchResult,
                                        requested_depth_m: float,
                                        is_nw_high_target: bool):
    if search_result.measured_best_depth_m >= requested_depth_m - 1e-6:
        return True, "reached_requested_depth", None
    if search_result.measured_best_j3_deg is None:
        return False, None, None
    threshold = measured_tcp_good_enough_j3_deg(is_nw_high_target)
    if search_result.measured_best_j3_deg >= threshold:
        return True, "j3_good_enough", threshold
    return False, None, threshold


def should_replace_measured_best(
        depth_m: float,
        candidate_j3_deg: float,
        quat_frame: str,
        quat_deg: float,
        measured_best,
        measured_best_depth_m: float,
        measured_best_j3_deg,
        measured_best_alignment_deg,
        is_nw_high_target: bool):
    candidate_is_published_roll = quat_frame == "published_roll"
    candidate_alignment_deg = 0.0 if candidate_is_published_roll else abs(float(quat_deg))
    best_is_published_roll = (
        measured_best is not None
        and measured_best[3][0] == "published_roll"
    )
    is_deeper = depth_m > measured_best_depth_m + 1e-6
    is_tied = abs(depth_m - measured_best_depth_m) <= 1e-6

    if (
        best_is_published_roll
        and not candidate_is_published_roll
        and measured_best_j3_deg is not None
        and measured_best_j3_deg >= NW_HIGH_TARGET_MIN_FLAT_BRANCH_J3_DEG
    ):
        return False, candidate_alignment_deg, False

    if is_nw_high_target and is_tied:
        candidate_flat_safe = candidate_j3_deg >= NW_HIGH_TARGET_MIN_FLAT_BRANCH_J3_DEG
        best_flat_safe = (
            measured_best_j3_deg is not None
            and measured_best_j3_deg >= NW_HIGH_TARGET_MIN_FLAT_BRANCH_J3_DEG
        )
        is_tied_but_better = (
            candidate_flat_safe
            and (
                not best_flat_safe
                or measured_best_alignment_deg is None
                or candidate_alignment_deg < measured_best_alignment_deg - 1e-6
                or (
                    abs(candidate_alignment_deg - measured_best_alignment_deg) <= 1e-6
                    and (
                        measured_best_j3_deg is None
                        or candidate_j3_deg > measured_best_j3_deg
                    )
                )
            )
        )
    else:
        is_tied_but_better = (
            is_tied
            and (
                measured_best_j3_deg is None
                or candidate_j3_deg > measured_best_j3_deg
            )
        )
    return bool(is_deeper or is_tied_but_better), candidate_alignment_deg, is_tied_but_better


def measured_best_tuple(r_pre_for_variant, r_final_probe, variant,
                        approach_dir, q_retry, ee_pre, probe_target):
    """Pack the historical measured-best tuple format in one place."""
    return (
        r_pre_for_variant,
        r_final_probe,
        MEASURED_TCP_FINAL_STANDOFF_M,
        variant,
        approach_dir,
        q_retry,
        ee_pre.copy(),
        probe_target.copy(),
    )


def leftmost_depth_limited(raw_x_m: float, selected_grasp_offset_m: float):
    return raw_x_m < -0.30 and selected_grasp_offset_m >= 0.050


def leftmost_rejected_offsets(selected_grasp_offset_m: float):
    return [
        value for value in LEFTMOST_GRASP_RETRY_OFFSETS
        if value < selected_grasp_offset_m
    ]
