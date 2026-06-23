#!/usr/bin/env python3
"""Grasp search executor for the harvest planner.

This wraps the per-grasp_quat_variant final-approach depth probe and the
legacy grasp-offset search. It is kept as a thin node-dependent class (like
HarvestGripperClient) rather than pure functions, since both searches must
call the live cuRobo `plan(...)` and log through the node's logger/runtime
log. The actual policy/math (probe depth selection, tie-break rules, depth
candidate lists) stays in grasp_candidate_policy.py as pure functions.
"""

from harvest_motion_params import PRE_APPROACH_OFFSET
from grasp_candidate_policy import (
    GraspSearchResult,
    legacy_grasp_endpoint,
    measured_best_tuple,
    measured_tcp_probe_log_message,
    measured_tcp_probe_depths,
    requested_measured_tcp_probe_depth,
    should_replace_measured_best,
    should_stop_measured_variant_search,
)

import numpy as np


class GraspSearchExecutor:
    """Runs the measured-TCP depth probe and legacy offset search."""

    def __init__(self, node, runtime_log, plan_fn,
                 measured_tcp_max_approach_m: float,
                 ee_to_tcp_offset_m: float,
                 flat_grasp_only: bool = False):
        self.node = node
        self.runtime_log = runtime_log
        self._plan = plan_fn
        self._measured_tcp_max_approach_m = measured_tcp_max_approach_m
        self._ee_to_tcp_offset_m = ee_to_tcp_offset_m
        self._flat_grasp_only = flat_grasp_only

    def _log(self):
        return self.node.get_logger()

    def run_measured_tcp_depth_probe(self, ee_pre, approach_dir, q_retry,
                                     pre_joints, r_pre_for_variant, variant,
                                     grasp_search: GraspSearchResult,
                                     measured_best, is_nw_high_target: bool):
        """Probe final-approach depths for one grasp_quat_variant.

        Mutates `grasp_search` in place (attempt_count, measured_best_*) and
        returns the possibly-updated `measured_best` tuple plus whether the
        outer grasp_quat_variants loop should stop.
        """
        quat_frame, axis, quat_deg = variant
        requested_probe_depth_m = requested_measured_tcp_probe_depth(
            self._measured_tcp_max_approach_m)
        probe_depths, probe_pruned = measured_tcp_probe_depths(
            requested_probe_depth_m,
            is_nw_high_target,
            grasp_search.measured_best_depth_m,
        )
        probe_log_message = measured_tcp_probe_log_message(
            probe_pruned,
            grasp_search.measured_best_depth_m,
            probe_depths,
        )
        if probe_log_message:
            self._log().info(probe_log_message)
        for depth_m in probe_depths:
            probe_target = ee_pre + depth_m * approach_dir
            r_final_probe = self._plan(
                pre_joints,
                probe_target.tolist(),
                q_retry,
                num_ik_seeds=24,
                max_attempts=2,
                timeout_sec=1.5,
                max_joint_delta_deg=90.0,
            )
            grasp_search.attempt_count += 1
            if r_final_probe is None:
                continue
            candidate_j3_deg = abs(float(np.rad2deg(r_final_probe[0][-1][2])))
            should_replace, candidate_alignment_deg, is_tied_but_better = (
                should_replace_measured_best(
                    depth_m=depth_m,
                    candidate_j3_deg=candidate_j3_deg,
                    quat_frame=quat_frame,
                    quat_deg=quat_deg,
                    measured_best=measured_best,
                    measured_best_depth_m=grasp_search.measured_best_depth_m,
                    measured_best_j3_deg=grasp_search.measured_best_j3_deg,
                    measured_best_alignment_deg=grasp_search.measured_best_alignment_deg,
                    is_nw_high_target=is_nw_high_target,
                    flat_grasp_only=self._flat_grasp_only,
                )
            )
            if should_replace:
                measured_best = grasp_search.update_measured_best(
                    depth_m,
                    candidate_j3_deg,
                    candidate_alignment_deg,
                    measured_best_tuple(
                        r_pre_for_variant,
                        r_final_probe,
                        (quat_frame, axis, quat_deg),
                        approach_dir,
                        q_retry,
                        ee_pre,
                        probe_target,
                    ),
                )
                self._log().info(
                    "MEASURED_TCP_FINAL_PROBE_BEST "
                    f"depth={depth_m*1000:.0f}mm J3={candidate_j3_deg:.1f}deg "
                    f"align={candidate_alignment_deg:.1f}deg "
                    f"variant={(quat_frame, axis, quat_deg)}"
                    + (" (tie-break: flatter safe branch)" if is_tied_but_better else ""))
            if depth_m >= requested_probe_depth_m - 1e-6:
                break
        should_stop, stop_reason, good_enough_j3_deg = (
            should_stop_measured_variant_search(
                grasp_search,
                requested_probe_depth_m,
                is_nw_high_target,
            )
        )
        should_break_outer = False
        if stop_reason == "reached_requested_depth":
            should_break_outer = True
        elif should_stop and stop_reason == "j3_good_enough":
            self._log().info(
                "MEASURED_TCP_VARIANT_SEARCH_STOPPED "
                f"J3={grasp_search.measured_best_j3_deg:.1f}deg >= "
                f"{good_enough_j3_deg:.0f}deg good-enough threshold — "
                "skipping remaining grasp_quat_variants")
            should_break_outer = True
        return measured_best, should_break_outer

    def try_legacy_grasp_offsets(self, grasp_retry_offsets, straw, approach_dir,
                                 q_retry, pre_joints, r_pre_for_variant, variant,
                                 ee_pre, grasp_search: GraspSearchResult):
        """Try legacy grasp_retry_offsets for one grasp_quat_variant.

        Mutates `grasp_search` in place via select_legacy_grasp() on the
        first reachable offset, then stops (matching the historical
        first-reachable-offset-wins behavior).
        """
        quat_frame, axis, quat_deg = variant
        for grasp_offset in grasp_retry_offsets:
            grasp_search.attempt_count += 1
            # 2-step 구조에서 grasp endpoint는 pre-approach보다 target에
            # 가까워야 한다. 6cm pre에서 7cm offset을 허용하면 직선 진입이
            # 음수가 되어 정확도 보장 목적이 깨진다.
            if grasp_offset >= PRE_APPROACH_OFFSET:
                continue
            ee_g_try = legacy_grasp_endpoint(
                straw,
                grasp_offset,
                self._ee_to_tcp_offset_m,
                approach_dir,
            )
            r_grasp = self._plan(pre_joints, ee_g_try.tolist(), q_retry, num_ik_seeds=32)
            if r_grasp is None:
                continue
            grasp_search.select_legacy_grasp(
                r_pre_for_variant,
                r_grasp,
                grasp_offset,
                (quat_frame, axis, quat_deg),
                approach_dir,
                q_retry,
                ee_pre,
                ee_g_try,
            )
            break
