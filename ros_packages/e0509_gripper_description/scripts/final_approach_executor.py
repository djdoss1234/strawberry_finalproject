#!/usr/bin/env python3
"""Final-approach execution for the harvest planner.

Wraps the straight-line/cuRobo-spline decision for the last segment of the
pick approach (precomputed depth-probe reuse, cuRobo IK fallback depths, and
the TOOL +Z "finish" move that covers any remaining distance). Kept as a
thin node-dependent class (like GraspSearchExecutor/TrayPlaceExecutor) since
it must call the live cuRobo `plan(...)`/Doosan motion clients and log
through the node's logger/runtime log. Pure math (target-plane distance,
fallback depth candidates, retreat direction) stays in
approach_retreat_policy.py.
"""

import numpy as np

from approach_retreat_policy import (
    FinalApproachState,
    final_approach_fallback_depths,
    measured_tcp_approach_distance,
    tool_finish_base_direction,
)
from harvest_motion_params import (
    ENABLE_CUROBO_FINAL_APPROACH_FALLBACK,
    FINAL_APPROACH_ACC_MM_S2,
    FINAL_APPROACH_VEL_MM_S,
    MEASURED_TCP_FINAL_STANDOFF_M,
    MEASURED_TCP_MAX_APPROACH_CEILING_M,
    PRE_APPROACH_OFFSET,
    WALL_SURFACE_Y_M,
    Y_DETECTION_BIAS_M,
)


class FinalApproachExecutor:
    """Runs the last approach segment (precomputed reuse / fallback / tool finish)."""

    def __init__(self, node, runtime_log, plan_fn, execute_spline_fn,
                 execute_base_relative_line_fn, execute_tool_z_line_fn,
                 current_joints_getter,
                 measured_tcp_model: bool,
                 flat_grasp_only: bool,
                 flat_grasp_target_plane_margin_m: float,
                 direct_curobo_final_approach_for_measured_tcp: bool,
                 measured_tcp_tool_line_after_curobo_fallback: bool,
                 nw_high_target_final_extra_m: float,
                 measured_tcp_max_approach_m: float):
        self.node = node
        self.runtime_log = runtime_log
        self._plan = plan_fn
        self._execute_spline = execute_spline_fn
        self._execute_base_relative_line = execute_base_relative_line_fn
        self._execute_tool_z_line = execute_tool_z_line_fn
        self._current_joints = current_joints_getter
        self._measured_tcp_model = measured_tcp_model
        self._flat_grasp_only = flat_grasp_only
        self._flat_grasp_target_plane_margin_m = flat_grasp_target_plane_margin_m
        self._direct_curobo_final_approach_for_measured_tcp = (
            direct_curobo_final_approach_for_measured_tcp)
        self._measured_tcp_tool_line_after_curobo_fallback = (
            measured_tcp_tool_line_after_curobo_fallback)
        self._nw_high_target_final_extra_m = nw_high_target_final_extra_m
        self._measured_tcp_max_approach_m = measured_tcp_max_approach_m

    def _log(self):
        return self.node.get_logger()

    def execute_tool_finish(self, remaining_tool_line_m: float,
                            curobo_depth_m: float,
                            requested_total_m: float,
                            used_grasp_variant,
                            used_approach_dir,
                            failure_context: str):
        self._log().warn(
            "FINAL_APPROACH_TOOL_FINISH: cuRobo reached "
            f"{curobo_depth_m*1000:.0f}mm only; executing "
            f"remaining {remaining_tool_line_m*1000:.0f}mm with "
            "TOOL +Z MoveLine like the proven SW baseline")
        self.runtime_log.log(
            "final_approach_tool_finish_requested",
            reason="curobo_deep_final_approach_ik_fail",
            curobo_depth_m=curobo_depth_m,
            tool_finish_m=remaining_tool_line_m,
            requested_total_m=requested_total_m,
        )
        finish_direction = tool_finish_base_direction(
            used_grasp_variant,
            used_approach_dir,
        )
        if finish_direction.use_base_line:
            if finish_direction.is_published_roll:
                self._log().warn(
                    "FINAL_APPROACH_TOOL_FINISH_BASE_FOR_PUBLISHED_ROLL: "
                    "using BASE relative line; TOOL +Z returned "
                    "success/no-motion in this branch")
            tool_finish_ok = self._execute_base_relative_line(
                remaining_tool_line_m * finish_direction.direction,
                "FINAL_APPROACH_TOOL_FINISH",
                vel_mm_s=FINAL_APPROACH_VEL_MM_S,
                acc_mm_s2=FINAL_APPROACH_ACC_MM_S2,
            )
            tool_finish_delta = remaining_tool_line_m * finish_direction.direction
            tool_finish_dir = finish_direction.direction if tool_finish_ok else None
        else:
            tool_finish_ok = self._execute_tool_z_line(
                remaining_tool_line_m,
                motion_label="FINAL_APPROACH_TOOL_FINISH",
                vel_mm_s=FINAL_APPROACH_VEL_MM_S,
                acc_mm_s2=FINAL_APPROACH_ACC_MM_S2,
                min_distance_m=0.005,
            )
            tool_finish_delta = remaining_tool_line_m * np.array(
                used_approach_dir, dtype=float)
            tool_finish_dir = None

        if not tool_finish_ok:
            self._log().error(
                "FINAL_APPROACH_TOOL_FINISH failed after "
                f"{failure_context}")
        return tool_finish_ok, tool_finish_delta, remaining_tool_line_m, tool_finish_dir

    def compute_distance(self, raw_straw, straw,
                         used_pre_ee_pos,
                         used_approach_dir,
                         used_grasp_offset: float,
                         is_nw_high_target: bool,
                         wall_y_clamped: bool = False):
        if not self._measured_tcp_model:
            return PRE_APPROACH_OFFSET - used_grasp_offset

        approach_distance = measured_tcp_approach_distance(
            raw_y_m=raw_straw[1],
            straw=straw,
            used_pre_ee_pos=used_pre_ee_pos,
            used_approach_dir=used_approach_dir,
            max_approach_m=self._measured_tcp_max_approach_m,
            y_detection_bias_m=Y_DETECTION_BIAS_M,
            pre_approach_offset_m=PRE_APPROACH_OFFSET,
            final_standoff_m=MEASURED_TCP_FINAL_STANDOFF_M,
            wall_surface_y_m=WALL_SURFACE_Y_M,
        )
        target_plane_dist = approach_distance.target_plane_dist_m
        uncapped_distance = approach_distance.uncapped_distance_m
        final_approach_distance = approach_distance.final_distance_m

        if self._flat_grasp_only and wall_y_clamped:
            # straw[1] (and therefore target_plane_dist) was clamped to the
            # registered wall surface, so it understates how far the real
            # target actually is -- capping to it here would systematically
            # stop short of the stem. Skip the cap and fall through to the
            # same near-max push used for non-flat/legacy targets instead.
            self._log().warn(
                "FLAT_GRASP_TARGET_PLANE_CAP_SKIPPED: wall_y_clamped=True, "
                "target_plane_dist is unreliable here; using uncapped "
                f"distance {final_approach_distance*1000:.0f}mm instead of "
                f"capping to target_plane={target_plane_dist*1000:.0f}mm")
            self.runtime_log.log(
                "flat_grasp_target_plane_cap_skipped",
                reason="wall_y_clamped",
                target_plane_distance_m=target_plane_dist,
                uncapped_distance_m=final_approach_distance,
            )
        elif self._flat_grasp_only:
            before_flat_cap_m = final_approach_distance
            flat_target_plane_m = (
                target_plane_dist + self._flat_grasp_target_plane_margin_m
            )
            # In flat/SW-style validation the target plane distance is the
            # physically meaningful straight segment.  The legacy measured-TCP
            # baseline uses a negative standoff that forces 180 mm even when the
            # target plane is only ~60-70 mm away, which makes Doosan MoveLine
            # push past the fruit and frequently return success/no-motion.
            # Keep a small configurable margin so the jaws reach past the
            # detected KP1 plane instead of closing in front of the stem.
            final_approach_distance = max(
                0.0,
                min(final_approach_distance, flat_target_plane_m),
            )
            if abs(before_flat_cap_m - final_approach_distance) > 1e-6:
                self._log().warn(
                    "FLAT_GRASP_TARGET_PLANE_CAP: "
                    f"{before_flat_cap_m*1000:.0f}mm -> "
                    f"{final_approach_distance*1000:.0f}mm "
                    "(SW-style flat approach uses target-plane distance "
                    f"+ {self._flat_grasp_target_plane_margin_m*1000:.0f}mm margin)")
                self.runtime_log.log(
                    "flat_grasp_target_plane_cap",
                    before_m=before_flat_cap_m,
                    after_m=final_approach_distance,
                    target_plane_distance_m=target_plane_dist,
                    margin_m=self._flat_grasp_target_plane_margin_m,
                )

        if (
            is_nw_high_target
            and
            abs(float(used_approach_dir[2])) > 1e-3
            and self._nw_high_target_final_extra_m > 0.0
        ):
            before_extra_m = final_approach_distance
            final_approach_distance = min(
                MEASURED_TCP_MAX_APPROACH_CEILING_M,
                final_approach_distance + self._nw_high_target_final_extra_m,
            )
            self._log().warn(
                "NW_HIGH_TARGET_FINAL_EXTRA: "
                f"{before_extra_m*1000:.0f}mm -> "
                f"{final_approach_distance*1000:.0f}mm "
                f"(target_z={raw_straw[2]*1000:.0f}mm, "
                "observed shallow by ~30mm)")
            self.runtime_log.log(
                "nw_high_target_final_extra",
                target_z_m=float(raw_straw[2]),
                before_m=before_extra_m,
                after_m=final_approach_distance,
                requested_extra_m=self._nw_high_target_final_extra_m,
                ceiling_m=MEASURED_TCP_MAX_APPROACH_CEILING_M,
            )
        if final_approach_distance + 1e-6 < uncapped_distance:
            self._log().warn(
                "MEASURED_TCP_APPROACH_CAPPED: requested "
                f"{uncapped_distance*1000:.0f}mm -> "
                f"{final_approach_distance*1000:.0f}mm "
                f"(target_plane={target_plane_dist*1000:.0f}mm, "
                "NW measured-TCP experimental depth cap)")
            self.runtime_log.log(
                "measured_tcp_approach_capped",
                requested_distance_m=uncapped_distance,
                capped_distance_m=final_approach_distance,
                target_plane_distance_m=target_plane_dist,
                max_approach_m=self._measured_tcp_max_approach_m,
                reason="nw_measured_tcp_experimental_depth_cap",
            )
        return final_approach_distance

    def try_precomputed(self, final_state: FinalApproachState,
                        ret_grasp,
                        measured_best_depth_m: float,
                        requested_distance_m: float,
                        used_pre_ee_pos,
                        used_grasp_variant,
                        used_approach_dir):
        if (
            not self._measured_tcp_model
            or not self._direct_curobo_final_approach_for_measured_tcp
        ):
            return False, False

        selected_curobo_depth_m = float(measured_best_depth_m)
        depth_mismatch_m = abs(selected_curobo_depth_m - float(requested_distance_m))
        if self._flat_grasp_only and depth_mismatch_m > 0.005:
            self._log().warn(
                "FINAL_APPROACH_PRECOMPUTED_CUROBO_SKIPPED: "
                f"probe depth {selected_curobo_depth_m*1000:.0f}mm differs from "
                f"requested {requested_distance_m*1000:.0f}mm by "
                f"{depth_mismatch_m*1000:.0f}mm (>5mm); planning exact "
                "target-plane fallback instead")
            return False, False
        if selected_curobo_depth_m > float(requested_distance_m) + 1e-6:
            self._log().warn(
                "FINAL_APPROACH_PRECOMPUTED_CUROBO_SKIPPED: "
                f"probe depth {selected_curobo_depth_m*1000:.0f}mm is deeper "
                f"than requested {requested_distance_m*1000:.0f}mm; "
                "planning exact target-plane fallback instead")
            return False, False
        approach_ok = (
            ret_grasp is not None
            and selected_curobo_depth_m > 0.0
            and self._execute_spline(*ret_grasp)
        )
        self.runtime_log.log(
            "final_approach_precomputed_curobo",
            controller="curobo_plus_doosan_move_spline_joint",
            requested_distance_m=requested_distance_m,
            executed_depth_m=selected_curobo_depth_m,
            success=approach_ok,
            approach_dir=used_approach_dir,
        )
        if approach_ok:
            self._log().info(
                "FINAL_APPROACH_PRECOMPUTED_CUROBO "
                f"depth={selected_curobo_depth_m*1000:.0f}mm "
                "(reusing probe plan; no extra IK fallback search)")
            final_state.distance_m = selected_curobo_depth_m
            final_state.grasp_ee_pos = (
                used_pre_ee_pos + final_state.distance_m * used_approach_dir)
            remaining_tool_line_m = requested_distance_m - selected_curobo_depth_m
            if (
                self._measured_tcp_tool_line_after_curobo_fallback
                and remaining_tool_line_m >= 0.020
            ):
                (
                    tool_finish_ok,
                    tool_finish_delta,
                    _finish_m,
                    tool_finish_dir,
                ) = self.execute_tool_finish(
                    remaining_tool_line_m,
                    selected_curobo_depth_m,
                    requested_distance_m,
                    used_grasp_variant,
                    used_approach_dir,
                    "precomputed cuRobo final approach",
                )
                if not tool_finish_ok:
                    approach_ok = False
                else:
                    final_state.apply_tool_finish(
                        selected_curobo_depth_m,
                        remaining_tool_line_m,
                        tool_finish_delta,
                        tool_finish_dir,
                    )
                    self.runtime_log.log(
                        "final_approach_tool_finish_success",
                        executed_total_m=final_state.distance_m,
                        horizontal_only=final_state.tool_finish_executed_dir is not None,
                    )
        else:
            self._log().warn(
                "FINAL_APPROACH_PRECOMPUTED_CUROBO failed; "
                "falling back to depth search")
        return True, approach_ok

    def try_fallback(self, final_state: FinalApproachState,
                     requested_final_approach_distance: float,
                     used_pre_ee_pos,
                     used_grasp_quat,
                     used_grasp_variant,
                     used_approach_dir,
                     precomputed_final_attempted: bool) -> bool:
        fallback_ok = False
        if not (
            self._measured_tcp_model
            and ENABLE_CUROBO_FINAL_APPROACH_FALLBACK
            and not precomputed_final_attempted
            and used_pre_ee_pos is not None
            and used_grasp_quat is not None
            and self._current_joints() is not None
        ):
            return fallback_ok

        depth_candidates = final_approach_fallback_depths(
            requested_final_approach_distance,
            self._direct_curobo_final_approach_for_measured_tcp,
        )
        for depth_m in depth_candidates:
            fallback_target = (
                used_pre_ee_pos
                + depth_m * used_approach_dir
            )
            self._log().warn(
                "FINAL_APPROACH_STRAIGHT_BASE failed; trying cuRobo "
                f"fallback depth={depth_m*1000:.0f}mm "
                f"xyz={[round(v*1000, 1) for v in fallback_target]}mm")
            self.runtime_log.log(
                "final_approach_fallback_requested",
                reason="doosan_moveline_failed_or_no_motion",
                target_pos_m=fallback_target.tolist(),
                approach_dir=used_approach_dir,
                depth_m=depth_m,
            )
            fallback_plan = self._plan(
                self._current_joints(),
                fallback_target.tolist(),
                used_grasp_quat,
                num_ik_seeds=24,
                max_attempts=2,
                timeout_sec=1.5,
                max_joint_delta_deg=90.0,
            )
            if fallback_plan is None:
                continue
            fallback_ok = self._execute_spline(*fallback_plan)
            if fallback_ok:
                final_state.distance_m = depth_m
                final_state.grasp_ee_pos = fallback_target.copy()
                self.runtime_log.log(
                    "final_approach_fallback_success",
                    controller="curobo_plus_doosan_move_spline_joint",
                    executed_depth_m=depth_m,
                )
                remaining_tool_line_m = (
                    requested_final_approach_distance - depth_m)
                if (
                    self._direct_curobo_final_approach_for_measured_tcp
                    and self._measured_tcp_tool_line_after_curobo_fallback
                    and remaining_tool_line_m >= 0.020
                ):
                    (
                        tool_finish_ok,
                        tool_finish_delta,
                        tool_finish_executed_m,
                        tool_finish_executed_dir,
                    ) = self.execute_tool_finish(
                        remaining_tool_line_m,
                        depth_m,
                        requested_final_approach_distance,
                        used_grasp_variant,
                        used_approach_dir,
                        "cuRobo shallow fallback",
                    )
                    if not tool_finish_ok:
                        fallback_ok = False
                        break
                    final_state.apply_tool_finish(
                        depth_m,
                        remaining_tool_line_m,
                        tool_finish_delta,
                        tool_finish_executed_dir,
                    )
                    self.runtime_log.log(
                        "final_approach_tool_finish_success",
                        executed_total_m=final_state.distance_m,
                        horizontal_only=final_state.tool_finish_executed_dir is not None,
                    )
                break
        return fallback_ok

    def execute(self, final_state: FinalApproachState,
               final_approach_distance: float,
               ret_grasp,
               measured_best_depth_m: float,
               used_pre_ee_pos,
               used_grasp_quat,
               used_grasp_variant,
               used_approach_dir) -> bool:
        if final_approach_distance <= 0.001:
            return True

        requested_final_approach_distance = final_approach_distance
        precomputed_final_attempted, approach_ok = (
            self.try_precomputed(
                final_state,
                ret_grasp,
                measured_best_depth_m,
                requested_final_approach_distance,
                used_pre_ee_pos,
                used_grasp_variant,
                used_approach_dir,
            )
        )
        if not precomputed_final_attempted and self._measured_tcp_model:
            approach_ok = self._execute_base_relative_line(
                final_approach_distance * used_approach_dir,
                "FINAL_APPROACH_STRAIGHT_BASE",
                vel_mm_s=FINAL_APPROACH_VEL_MM_S,
                acc_mm_s2=FINAL_APPROACH_ACC_MM_S2,
            )
        elif not precomputed_final_attempted:
            approach_ok = self._execute_tool_z_line(
                final_approach_distance,
                min_distance_m=0.005)
        if not approach_ok:
            fallback_ok = self.try_fallback(
                final_state,
                requested_final_approach_distance,
                used_pre_ee_pos,
                used_grasp_quat,
                used_grasp_variant,
                used_approach_dir,
                precomputed_final_attempted,
            )
            if not fallback_ok:
                return False
        return True
