#!/usr/bin/env python3
"""Pick-sequence orchestration for the harvest planner.

Wraps the full pick_pose callback sequence (target prep -> grasp search ->
pre-approach -> final approach -> open-stem descent -> close/verify ->
detach+retreat -> optional marker place -> return to scan pose). Kept as a
thin node-dependent class (like TrayPlaceExecutor/GraspSearchExecutor) since
it drives the live cuRobo plan/Doosan motion clients, gripper client, scene
obstacle manager, and tray place executor, and logs through the node's
logger/runtime log. Pure policy/math stays in pick_target_policy.py,
grasp_candidate_policy.py, harvest_result_policy.py, place_sequence_policy.py,
and open_stem_descent_policy.py.
"""

import time

import numpy as np

from approach_retreat_policy import (
    build_straight_retreat_steps,
    FinalApproachState,
)
from grasp_candidate_policy import (
    GraspSearchResult,
    grasp_quat_variants_for_target,
    grasp_variant_pose,
    leftmost_depth_limited,
    leftmost_rejected_offsets,
    variant_label,
)
from harvest_math import quat_normalize_wxyz
from harvest_motion_params import (
    CRANE_DESCENT_VEL_MM_S,
    DETACH_PULL_DOWN_MM,
    DETACH_PULL_VEL_MM_S,
    FINAL_APPROACH_ACC_MM_S2,
    GRIPPER_APPROACH_POS,
    LEFTMOST_EXTRA_ADVANCE_VEL_MM_S,
    LEFTMOST_GRASP_X_CORR_M,
    NW_HIGH_TARGET_Y_PLANE_RELAX_M,
    PRE_APPROACH_OFFSET,
    PRE_APPROACH_SETTLE_SEC,
    STRAIGHT_RETREAT_SETTLE_SEC,
    WALL_SURFACE_Y_M,
)
from harvest_result_policy import (
    allow_place_after_grasp,
    pick_sequence_result_code,
    place_gate_block_reason,
)
from open_stem_descent_policy import compute_open_stem_descent_m
from pick_target_policy import prepare_pick_target
from place_sequence_policy import classify_place_outcome


class PickSequenceExecutor:
    """Runs the full pick_pose sequence for one detected target."""

    def __init__(self, node, runtime_log,
                 gripper_client, motion_gen, grasp_search_executor,
                 tray_place_executor,
                 plan_fn, execute_spline_fn,
                 execute_base_z_relative_fn, execute_base_relative_line_fn,
                 execute_tool_z_line_fn, execute_pitch_detach_fn,
                 execute_retreat_steps_fn,
                 plan_to_fixed_joints_pose_fn, nearest_equivalent_joints_fn,
                 grasp_candidates_for_target_fn, published_roll_grasp_variant_fn,
                 close_and_verify_grasp_fn,
                 compute_final_approach_distance_fn, execute_final_approach_fn,
                 register_neighbor_obstacles_fn, clear_neighbor_obstacles_fn,
                 reset_gripper_fn, abort_pick_with_complete_fn,
                 publish_pick_complete_fn, hold_pick_sequence_fn,
                 current_joints_getter,
                 marker_place_slot_idx_getter, increment_marker_place_slot_idx_fn,
                 measured_tcp_model: bool,
                 measured_tcp_plan_only: bool,
                 flat_grasp_only: bool,
                 ee_to_tcp_offset_m: float,
                 pick_target_x_bias_m: float,
                 pick_target_z_bias_m: float,
                 nw_high_target_z_threshold_m: float,
                 nw_high_target_crane_z_offset_m: float,
                 nw_high_target_descent_extra_below_kp1_m: float,
                 nw_high_target_base_y_nudge_m: float,
                 leftmost_extra_advance_request_m: float,
                 leftmost_wall_safety_margin_m: float,
                 leftmost_allow_wall_model_override: bool,
                 allow_unverified_grasp_place: bool,
                 enable_marker_place: bool,
                 execute_marker_place_release: bool,
                 use_taught_slot0_place_reference: bool,
                 hold_after_taught_slot0_place: bool):
        self.node = node
        self.runtime_log = runtime_log
        self._gripper_client = gripper_client
        self._motion_gen = motion_gen
        self._grasp_search_executor = grasp_search_executor
        self._tray_place_executor = tray_place_executor
        self._plan = plan_fn
        self._execute_spline = execute_spline_fn
        self._execute_base_z_relative = execute_base_z_relative_fn
        self._execute_base_relative_line = execute_base_relative_line_fn
        self._execute_tool_z_line = execute_tool_z_line_fn
        self._execute_pitch_detach_fn = execute_pitch_detach_fn
        self._execute_retreat_steps_fn = execute_retreat_steps_fn
        self._plan_to_fixed_joints_pose_fn = plan_to_fixed_joints_pose_fn
        self._nearest_equivalent_joints_fn = nearest_equivalent_joints_fn
        self._grasp_candidates_for_target_fn = grasp_candidates_for_target_fn
        self._published_roll_grasp_variant_fn = published_roll_grasp_variant_fn
        self._close_and_verify_grasp_fn = close_and_verify_grasp_fn
        self._compute_final_approach_distance_fn = compute_final_approach_distance_fn
        self._execute_final_approach_fn = execute_final_approach_fn
        self._register_neighbor_obstacles_fn = register_neighbor_obstacles_fn
        self._clear_neighbor_obstacles_fn = clear_neighbor_obstacles_fn
        self._reset_gripper_fn = reset_gripper_fn
        self._abort_pick_with_complete_fn = abort_pick_with_complete_fn
        self._publish_pick_complete_fn = publish_pick_complete_fn
        self._hold_pick_sequence_fn = hold_pick_sequence_fn
        self._current_joints = current_joints_getter
        self._marker_place_slot_idx_getter = marker_place_slot_idx_getter
        self._increment_marker_place_slot_idx_fn = increment_marker_place_slot_idx_fn
        self._measured_tcp_model = measured_tcp_model
        self._measured_tcp_plan_only = measured_tcp_plan_only
        self._flat_grasp_only = flat_grasp_only
        self._ee_to_tcp_offset_m = ee_to_tcp_offset_m
        self._pick_target_x_bias_m = pick_target_x_bias_m
        self._pick_target_z_bias_m = pick_target_z_bias_m
        self._nw_high_target_z_threshold_m = nw_high_target_z_threshold_m
        self._nw_high_target_crane_z_offset_m = nw_high_target_crane_z_offset_m
        self._nw_high_target_descent_extra_below_kp1_m = (
            nw_high_target_descent_extra_below_kp1_m)
        self._nw_high_target_base_y_nudge_m = nw_high_target_base_y_nudge_m
        self._leftmost_extra_advance_request_m = leftmost_extra_advance_request_m
        self._leftmost_wall_safety_margin_m = leftmost_wall_safety_margin_m
        self._leftmost_allow_wall_model_override = leftmost_allow_wall_model_override
        self._allow_unverified_grasp_place = allow_unverified_grasp_place
        self._enable_marker_place = enable_marker_place
        self._execute_marker_place_release = execute_marker_place_release
        self._use_taught_slot0_place_reference = use_taught_slot0_place_reference
        self._hold_after_taught_slot0_place = hold_after_taught_slot0_place

    def _log(self):
        return self.node.get_logger()

    def execute_open_stem_descent_if_needed(self, crane_z_offset_m: float,
                                             straw_z_m: float,
                                             used_grasp_ee_pos,
                                             used_grasp_variant) -> bool:
        if not self._measured_tcp_model or crane_z_offset_m <= 0:
            return True

        open_stem_descent_m, descent_info = compute_open_stem_descent_m(
            crane_z_offset_m,
            float(straw_z_m),
            None if used_grasp_ee_pos is None else float(used_grasp_ee_pos[2]),
            self._nw_high_target_descent_extra_below_kp1_m,
        )
        if descent_info["mode"] == "dynamic":
            self._log().warn(
                "OPEN_DESCENT_DYNAMIC: kp1_z="
                f"{descent_info['target_kp1_z_m']*1000:.0f}mm "
                f"reached_z={descent_info['reached_z_m']*1000:.0f}mm "
                f"overshoot={descent_info['overshoot_above_kp1_m']*1000:.0f}mm "
                f"extra_below_kp1={descent_info['extra_below_kp1_m']*1000:.0f}mm "
                f"-> descent={open_stem_descent_m*1000:.0f}mm")
            self.runtime_log.log(
                "nw_high_target_open_descent_dynamic",
                target_kp1_z_m=descent_info["target_kp1_z_m"],
                reached_z_m=descent_info["reached_z_m"],
                overshoot_above_kp1_m=descent_info["overshoot_above_kp1_m"],
                extra_below_kp1_m=descent_info["extra_below_kp1_m"],
                executed_descent_m=descent_info["executed_descent_m"],
                selected_variant=used_grasp_variant,
            )
        self._log().info(
            f"OPEN_STEM_DESCENT — gripper={GRIPPER_APPROACH_POS}, "
            f"BASE -Z {open_stem_descent_m*1000:.0f}mm to KP1")
        if not self._execute_base_z_relative(
                -open_stem_descent_m, "OPEN_STEM_DESCENT", CRANE_DESCENT_VEL_MM_S):
            self._log().error("ABORT: open stem descent 실패")
            self._abort_pick_with_complete_fn()
            return False
        return True

    def execute_nw_base_y_nudge_if_needed(self, is_nw_high_target: bool,
                                           raw_target_z_m: float) -> bool:
        if not is_nw_high_target or self._nw_high_target_base_y_nudge_m <= 0.0:
            return True
        self._log().warn(
            "NW_HIGH_TARGET_BASE_Y_NUDGE: BASE +Y "
            f"{self._nw_high_target_base_y_nudge_m*1000:.0f}mm before close "
            "(pure depth correction after height alignment)")
        self.runtime_log.log(
            "nw_high_target_base_y_nudge",
            target_z_m=float(raw_target_z_m),
            base_y_nudge_m=self._nw_high_target_base_y_nudge_m,
        )
        if not self._execute_base_relative_line(
                [0.0, self._nw_high_target_base_y_nudge_m, 0.0],
                "NW_HIGH_TARGET_BASE_Y_NUDGE",
                CRANE_DESCENT_VEL_MM_S,
                FINAL_APPROACH_ACC_MM_S2):
            self._log().error(
                "ABORT: NW high target BASE +Y nudge failed")
            self._abort_pick_with_complete_fn()
            return False
        return True

    def handle_gripper_close_failed(self, final_approach_distance: float,
                                     extra_advance_m: float,
                                     tool_finish_executed_m: float,
                                     tool_finish_executed_dir,
                                     used_approach_dir):
        self._log().error(
            "ABORT: gripper close failed twice — skip detach and retreat straight")
        self.runtime_log.log(
            "pick_sequence_stopped",
            result_code="GRIPPER_CLOSE_FAILED",
            action="straight_retreat_without_detach",
        )
        retreat_distance_m = (
            final_approach_distance + extra_advance_m - tool_finish_executed_m)
        retreat_ok = self._execute_retreat_steps_fn(
            build_straight_retreat_steps(
                self._measured_tcp_model,
                retreat_distance_m,
                used_approach_dir,
                tool_finish_executed_m,
                tool_finish_executed_dir,
                "CLOSE_FAIL_RETREAT_BASE",
                "CLOSE_FAIL_RETREAT",
            )
        )
        self._clear_neighbor_obstacles_fn()
        if retreat_ok:
            self._reset_gripper_fn()
            self._publish_pick_complete_fn()
        else:
            self._hold_pick_sequence_fn("gripper_close_failed_retreat_failed")

    def execute_detach_and_retreat(self, final_approach_distance: float,
                                    extra_advance_m: float,
                                    tool_finish_executed_m: float,
                                    tool_finish_executed_dir,
                                    used_approach_dir,
                                    grasp_joints):
        self._log().info(
            f"4 detach pull — BASE -Z {DETACH_PULL_DOWN_MM:.0f}mm "
            f"at {DETACH_PULL_VEL_MM_S:.0f}mm/s")
        self._execute_pitch_detach_fn()  # 실패해도 retreat은 항상 실행

        # 실측 TCP 모델은 cuRobo가 pre-approach까지만 계획하므로 최종 MoveLine
        # 전체를 역진한다. Legacy 모델은 기존 검증 baseline대로 extra advance만
        # 역진하고 이후 joint-space 복귀를 사용한다.
        reverse_distance_m = extra_advance_m
        if self._measured_tcp_model:
            reverse_distance_m += final_approach_distance - tool_finish_executed_m
        reverse_ok = self._execute_retreat_steps_fn(
            build_straight_retreat_steps(
                self._measured_tcp_model,
                reverse_distance_m,
                used_approach_dir,
                tool_finish_executed_m,
                tool_finish_executed_dir,
                "RETREAT_BASE",
                "RETREAT",
            )
        )
        if not reverse_ok:
            self._log().error(
                "ABORT: straight reverse retreat failed — holding current pose")
            self._clear_neighbor_obstacles_fn()
            self._hold_pick_sequence_fn("straight_reverse_retreat_failed")
            return None

        time.sleep(STRAIGHT_RETREAT_SETTLE_SEC)
        return (
            list(self._current_joints())
            if self._current_joints() is not None
            else grasp_joints
        )

    def maybe_execute_place_after_retreat(self, grasp_result: str,
                                           retreat_joints):
        # Place 게이트 기본값은 fail-closed다. 센서 판독이 불가능한 실험에서만
        # allow_unverified_grasp_place를 명시적으로 켜고 사람 관찰 라벨을 남긴다.
        allow_place = allow_place_after_grasp(
            grasp_result,
            self._allow_unverified_grasp_place,
        )
        if not allow_place:
            place_block_reason = place_gate_block_reason(grasp_result)
            self._log().warn(
                f"PLACE_GATE_BLOCKED ({grasp_result}): {place_block_reason}")
            self.runtime_log.log(
                "place_gate_blocked",
                grasp_result=grasp_result,
                reason=place_block_reason,
            )

        return_start_joints = retreat_joints
        if not self._enable_marker_place or not allow_place:
            return False, return_start_joints

        place_status, place_joints = self._tray_place_executor.execute_marker_place_after_retreat(
            retreat_joints)
        if place_status == "success":
            self._increment_marker_place_slot_idx_fn()
        outcome = classify_place_outcome(
            place_status,
            self._use_taught_slot0_place_reference,
            self._hold_after_taught_slot0_place,
            self._marker_place_slot_idx_getter(),
        )
        if outcome["action"] == "continue":
            return False, place_joints
        if (
            outcome["action"] == "hold"
            and outcome["result_code"] == "TAUGHT_TRAY_PLACE_COMPLETE_HOLD"
        ):
            completed_slot_index = outcome["completed_slot_index"]
            self._clear_neighbor_obstacles_fn()
            self.runtime_log.log(
                "pick_sequence_stopped",
                result_code=outcome["result_code"],
                slot_index=completed_slot_index,
                current_joints_rad=self._current_joints(),
            )
            self._log().warn(
                f"TAUGHT_TRAY_SLOT{completed_slot_index}_PLACE_COMPLETE_HOLD: "
                "release complete; "
                "automatic next pick blocked until planner restart")
            self._hold_pick_sequence_fn(outcome["hold_reason"])
            return True, return_start_joints
        if (
            outcome["action"] == "hold"
            and outcome["result_code"] == "TAUGHT_TRAY_FULL"
        ):
            self._clear_neighbor_obstacles_fn()
            self.runtime_log.log(
                "pick_sequence_stopped",
                result_code=outcome["result_code"],
                current_joints_rad=self._current_joints(),
            )
            self._log().warn(
                "TAUGHT_TRAY_FULL: all 15 slots consumed; "
                "automatic next pick blocked until tray reset")
            self._hold_pick_sequence_fn(outcome["hold_reason"])
            return True, return_start_joints
        if outcome["action"] == "skip":
            # tray 없음/stale — place 생략, scan 복귀
            self._log().warn("PLACE_SKIPPED: tray unavailable; returning to scan")
            self.runtime_log.log("place_skipped", reason=outcome["reason"],
                                 grasp_result=grasp_result)
            return False, return_start_joints

        # 로봇이 이미 움직인 뒤 실패 or preview hold → latch
        self._clear_neighbor_obstacles_fn()
        self.runtime_log.log(
            "pick_sequence_stopped",
            result_code=outcome["result_code"],
            place_status=place_status,
            current_joints_rad=self._current_joints(),
        )
        self._log().warn(
            f"PICK_SEQUENCE_HOLD place_status={place_status}; "
            "pick_complete not published, automatic scan paused")
        self._hold_pick_sequence_fn(outcome["hold_reason"])
        return True, return_start_joints

    def return_to_pick_start_and_complete(self, return_start_joints,
                                           pick_start_joints,
                                           grasp_result: str,
                                           detach_result: str) -> bool:
        self._log().info("7 return to pick-start scan pose")
        # 직선 retreat 또는 marker place 완료 후 이번 pick이 시작된 scan pose로
        # 복귀한다. scan_executor는 같은 SW 셀의 다음 target을 이어서 전달한다.
        pick_start_joints_deg = np.rad2deg(pick_start_joints).tolist()
        pick_start_joints_deg = self._nearest_equivalent_joints_fn(pick_start_joints_deg)
        ok, _ = self._plan_to_fixed_joints_pose_fn(
            return_start_joints,
            pick_start_joints_deg,
            "pick-start scan pose after pick/place",
            skip_swing_check=True,
        )
        if not ok:
            self._log().warn(
                "pick-start scan pose after pick/place failed; holding current pose")
            self._clear_neighbor_obstacles_fn()
            self.runtime_log.log(
                "pick_sequence_stopped",
                result_code="RETURN_TO_SCAN_FAILED",
                current_joints_rad=self._current_joints(),
            )
            self._hold_pick_sequence_fn("return_to_scan_failed")
            return False

        self._clear_neighbor_obstacles_fn()
        self._reset_gripper_fn()  # 다음 파지를 위해 approach 위치(600)로 복귀
        self._publish_pick_complete_fn()
        sequence_result_code = pick_sequence_result_code(grasp_result)
        self.runtime_log.log(
            "pick_sequence_complete",
            result_code=sequence_result_code,
            grasp_result=grasp_result,
            detach_result=detach_result,
            return_pose="pick_start_scan_pose",
            marker_place_enabled=self._enable_marker_place,
            marker_place_release_executed=(
                self._enable_marker_place and self._execute_marker_place_release),
            current_joints_rad=self._current_joints(),
        )
        self._log().info(f"=== PICK COMPLETE ({sequence_result_code}) ===")
        return True

    def execute_leftmost_extra_advance_if_needed(self, raw_x_m: float,
                                                  used_grasp_offset: float,
                                                  used_approach_dir,
                                                  used_grasp_ee_pos):
        extra_advance_m = 0.0
        if raw_x_m > 0.25 or self._leftmost_extra_advance_request_m <= 0.0:
            return True, extra_advance_m, used_grasp_ee_pos

        available_extra_m = max(
            0.0, used_grasp_offset - self._leftmost_wall_safety_margin_m)
        extra_advance_m = (
            self._leftmost_extra_advance_request_m
            if self._leftmost_allow_wall_model_override
            else min(self._leftmost_extra_advance_request_m, available_extra_m)
        )
        if self._leftmost_allow_wall_model_override:
            modeled_overtravel_m = max(0.0, extra_advance_m - available_extra_m)
            self._log().error(
                f"LEFTMOST_WALL_MODEL_OVERRIDE: executing "
                f"{extra_advance_m*1000:.0f}mm extra advance; "
                f"modeled wall overtravel={modeled_overtravel_m*1000:.0f}mm. "
                f"Physical clearance and E-stop must be verified.")
            self.runtime_log.log(
                "leftmost_wall_model_override",
                requested_m=self._leftmost_extra_advance_request_m,
                executed_m=extra_advance_m,
                safe_available_m=available_extra_m,
                modeled_wall_overtravel_m=modeled_overtravel_m,
                reason="explicit_ros_parameter_override",
            )
        if extra_advance_m < 0.020:
            self._log().warn(
                f"LEFTMOST_EXTRA_ADVANCE_BLOCKED: request="
                f"{self._leftmost_extra_advance_request_m*1000:.0f}mm, "
                f"safe_available={available_extra_m*1000:.0f}mm, "
                f"wall_margin={self._leftmost_wall_safety_margin_m*1000:.0f}mm")
            self.runtime_log.log(
                "leftmost_extra_advance_blocked",
                requested_m=self._leftmost_extra_advance_request_m,
                safe_available_m=available_extra_m,
                wall_safety_margin_m=self._leftmost_wall_safety_margin_m,
                selected_grasp_offset_m=used_grasp_offset,
            )
            return True, 0.0, used_grasp_ee_pos

        if extra_advance_m < self._leftmost_extra_advance_request_m:
            self._log().warn(
                f"LEFTMOST_EXTRA_ADVANCE_CAPPED: request="
                f"{self._leftmost_extra_advance_request_m*1000:.0f}mm -> "
                f"execute={extra_advance_m*1000:.0f}mm "
                f"(wall margin {self._leftmost_wall_safety_margin_m*1000:.0f}mm)")
        self.runtime_log.log(
            "leftmost_extra_advance",
            requested_m=self._leftmost_extra_advance_request_m,
            executed_m=extra_advance_m,
            wall_safety_margin_m=self._leftmost_wall_safety_margin_m,
            selected_grasp_offset_m=used_grasp_offset,
            validation="wall_distance_gate_only_not_curobo_endpoint",
        )
        if not self._execute_tool_z_line(
            extra_advance_m,
            motion_label="LEFTMOST_EXTRA_ADVANCE",
            vel_mm_s=LEFTMOST_EXTRA_ADVANCE_VEL_MM_S,
            acc_mm_s2=FINAL_APPROACH_ACC_MM_S2,
        ):
            self._log().error(
                "ABORT: leftmost extra advance failed after dispatch — "
                "holding current pose")
            self._clear_neighbor_obstacles_fn()
            self._hold_pick_sequence_fn("leftmost_extra_advance_failed")
            return False, extra_advance_m, used_grasp_ee_pos
        used_grasp_ee_pos = (
            used_grasp_ee_pos + extra_advance_m * used_approach_dir)
        return True, extra_advance_m, used_grasp_ee_pos

    def prepare_pick_target_or_abort(self, p):
        target_info = prepare_pick_target(
            p,
            self._measured_tcp_model,
            self._pick_target_x_bias_m,
            self._pick_target_z_bias_m,
            self._nw_high_target_z_threshold_m,
            self._nw_high_target_crane_z_offset_m,
        )
        detection_raw_y = target_info["detection_raw_y"]
        raw_y = target_info["raw_y"]
        wall_y_clamped = target_info["wall_y_clamped"]
        raw_straw = target_info["raw_straw"]
        straw = target_info["straw"]
        is_nw_high_target = target_info["is_nw_high_target"]
        crane_z_offset_m = target_info["crane_z_offset_m"]

        if wall_y_clamped:
            self._log().warn(
                f"Detection Y={detection_raw_y*1000:.0f}mm > wall surface "
                f"{WALL_SURFACE_Y_M*1000:.0f}mm "
                f"(FK calibration drift) — clamped to {WALL_SURFACE_Y_M*1000:.0f}mm")
        if target_info["y_relax_applied"]:
            before_y = target_info["y_relax_before_m"]
            self._log().warn(
                "NW_HIGH_TARGET_Y_PLANE_RELAX: clamped target Y "
                f"{before_y*1000:.0f}mm -> {straw[1]*1000:.0f}mm "
                "(limited stem-depth correction before planning)")
            self.runtime_log.log(
                "nw_high_target_y_plane_relax",
                raw_detection_y_m=detection_raw_y,
                clamped_wall_y_m=WALL_SURFACE_Y_M,
                before_target_y_m=before_y,
                after_target_y_m=float(straw[1]),
                relax_m=NW_HIGH_TARGET_Y_PLANE_RELAX_M,
            )

        x_min, x_max = target_info["x_range_m"]
        if not target_info["x_guard_ok"]:
            self._log().warn(
                f"ABORT: pick target x={raw_straw[0]*1000:.0f}mm outside "
                f"[{x_min*1000:.0f}, {x_max*1000:.0f}]mm")
            self._publish_pick_complete_fn()
            return None
        if not target_info["z_guard_ok"]:
            self._log().warn(
                f"SKIP: pick target z={raw_straw[2]*1000:.0f}mm > "
                f"{target_info['z_max_m']*1000:.0f}mm "
                "(NW high/leaf candidate guard)")
            self.runtime_log.log(
                "pick_target_skipped",
                reason="target_z_above_measured_tcp_guard",
                raw_target_m=raw_straw,
                z_max_m=target_info["z_max_m"],
                wall_y_clamped=wall_y_clamped,
            )
            self._publish_pick_complete_fn()
            return None
        return target_info

    def search_grasp(self, straw, is_nw_high_target, input_quat_wxyz,
                      grasp_retry_offsets, crane_z_offset_m):
        # 2. Grasp (cuRobo 2-step): 6cm pre-approach → 직선 진입
        # 직전 측방 편차가 줄기 형상/검출점 영향인지 분리하기 위해 6cm를 재검증한다.
        grasp_quat_variants = grasp_quat_variants_for_target(
            self._measured_tcp_model,
            is_nw_high_target,
        )
        if self._flat_grasp_only:
            grasp_quat_variants = [
                v for v in grasp_quat_variants
                if abs(float(v[2])) < 1e-6
            ]
            self._log().warn(
                "FLAT_GRASP_ONLY: using 0deg wall-normal grasp variant "
                "for SW-style horizontal final approach")
        published_roll_variant = self._published_roll_grasp_variant_fn(input_quat_wxyz)
        if published_roll_variant is not None and not self._flat_grasp_only:
            grasp_quat_variants = [published_roll_variant] + grasp_quat_variants

        n_offsets = len(grasp_retry_offsets)
        n_quats   = len(grasp_quat_variants)
        self._log().info(
            f"2 grasp (CuRobo 2-step {PRE_APPROACH_OFFSET*100:.0f}cm pre) — "
            f"trying {n_offsets} offsets × {n_quats} quats "
            f"| target=({straw[0]*1000:.0f},{straw[1]*1000:.0f},{straw[2]*1000:.0f})mm "
            f"| start_J1={np.rad2deg(self._current_joints()[0]):.1f}°")
        if is_nw_high_target:
            self._log().warn(
                "NW_HIGH_TARGET_VARIANT_ORDER: "
                + ", ".join(variant_label(v) for v in grasp_quat_variants)
                + " (prefer flatter branch over +15deg side-drift)")
        grasp_search = GraspSearchResult()
        # 2026-06-20: FINAL_APPROACH_TOOL_FINISH가 틸트 variant에서 horiz_dir로
        # 꺾여 들어가면 전진 경로가 더 이상 단일 직선(used_approach_dir)이 아니다.
        # retreat 쪽이 그 꺾인 다리를 모르면 전체 거리를 한 방향으로만 되돌리려
        # 하면서 실제로 온 길을 못 따라가 관절을 무리하게 꺾는다(J2 한도 초과
        # 실기로 확인). 이 두 변수로 꺾인 다리만 따로 기록해서 retreat에서
        # 분리해 되돌린다. 틸트가 없으면(horiz_dir==used_approach_dir) 0/None
        # 그대로라 기존 단일 직선 retreat과 동일하게 동작한다(SW no-op).
        tool_finish_executed_m = 0.0
        tool_finish_executed_dir = None
        measured_best = None
        # 2026-06-18: depth probing picks the deepest reachable standoff, but
        # multiple grasp_quat_variants often reach the IDENTICAL depth with
        # wildly different elbow health (J3 from ~0deg/near-singular up to
        # ~60deg/healthy) — verified by replaying a real failing run offline
        # bit-for-bit (replay_plan_call_dump.py). The old `depth_m >
        # measured_best_depth_m` strict-greater comparison always kept the
        # FIRST variant tried on a tie, which was consistently the worst one.
        # Track J3 health as a tiebreaker so an equally-deep but healthier
        # elbow from a later variant can replace it.
        for quat_frame, axis, quat_deg in grasp_quat_variants:
            q_retry, approach_dir, ee_pre = grasp_variant_pose(
                (quat_frame, axis, quat_deg),
                straw,
                self._ee_to_tcp_offset_m,
                self._measured_tcp_model,
                crane_z_offset_m,
            )
            r_pre_for_variant = self._plan(
                self._current_joints(), ee_pre.tolist(), q_retry, num_ik_seeds=24
            )
            if r_pre_for_variant is None:
                grasp_search.attempt_count += len(grasp_retry_offsets)
                continue
            pre_joints = r_pre_for_variant[0][-1].tolist()

            if self._measured_tcp_model:
                # Measured TCP에서 pre-approach만 보고 첫 자세를 확정하면,
                # NW처럼 자세가 빡빡한 영역에서 final 접근 IK가 계속 막힌다.
                # 실행 전 각 orientation의 final depth reachability를 probing해
                # 가장 깊게 들어갈 수 있는 자세를 고른다.
                measured_best, should_break_outer = (
                    self._grasp_search_executor.run_measured_tcp_depth_probe(
                        ee_pre,
                        approach_dir,
                        q_retry,
                        pre_joints,
                        r_pre_for_variant,
                        (quat_frame, axis, quat_deg),
                        grasp_search,
                        measured_best,
                        is_nw_high_target,
                    )
                )
                if should_break_outer:
                    break
                continue

            self._grasp_search_executor.try_legacy_grasp_offsets(
                grasp_retry_offsets,
                straw,
                approach_dir,
                q_retry,
                pre_joints,
                r_pre_for_variant,
                (quat_frame, axis, quat_deg),
                ee_pre,
                grasp_search,
            )
            if grasp_search.found:
                break

        if self._measured_tcp_model and measured_best is not None:
            grasp_search.apply_measured_best(measured_best)
            self.runtime_log.log(
                "measured_tcp_final_probe_selected",
                selected_depth_m=grasp_search.measured_best_depth_m,
                grasp_variant=grasp_search.grasp_variant,
                pre_ee_pos_m=grasp_search.pre_ee_pos.tolist(),
                final_ee_pos_m=grasp_search.grasp_ee_pos.tolist(),
            )
        return (
            grasp_search, n_offsets, n_quats,
            tool_finish_executed_m, tool_finish_executed_dir,
        )

    def run(self, msg):
        p = msg.pose.position
        input_quat_wxyz = quat_normalize_wxyz([
            msg.pose.orientation.w,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
        ])
        # 같은 셀의 다음 target을 계속 처리할 수 있도록 이번 pick이 시작된
        # taught scan pose를 저장한다. overview 복귀는 scan_executor가 담당한다.
        pick_start_joints = list(self._current_joints())
        self.runtime_log.log(
            "pick_sequence_start",
            input_frame=msg.header.frame_id,
            input_target_m=[p.x, p.y, p.z],
            input_quat_xyzw=[
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ],
            input_quat_wxyz=input_quat_wxyz,
            start_joints_rad=pick_start_joints,
        )

        target_info = self.prepare_pick_target_or_abort(p)
        if target_info is None:
            return
        detection_raw_y = target_info["detection_raw_y"]
        wall_y_clamped = target_info["wall_y_clamped"]
        raw_straw = target_info["raw_straw"]
        straw = target_info["straw"]
        is_nw_high_target = target_info["is_nw_high_target"]
        crane_z_offset_m = target_info["crane_z_offset_m"]

        grasp_retry_offsets = self._grasp_candidates_for_target_fn(straw)

        self._log().info(
            f"=== PICK 딸기 raw=({raw_straw[0]*1000:.0f},{raw_straw[1]*1000:.0f},{raw_straw[2]*1000:.0f})mm "
            f"grasp=({straw[0]*1000:.0f},{straw[1]*1000:.0f},{straw[2]*1000:.0f})mm "
            f"det_y={detection_raw_y*1000:.0f}mm "
            f"x_bias={self._pick_target_x_bias_m*1000:+.0f}mm "
            f"z_bias={self._pick_target_z_bias_m*1000:+.0f}mm ===")
        self.runtime_log.log(
            "pick_target_prepared",
            detection_raw_y_m=detection_raw_y,
            raw_target_m=raw_straw,
            grasp_target_m=straw,
            grasp_x_bias_m=self._pick_target_x_bias_m,
            grasp_z_bias_m=self._pick_target_z_bias_m,
            wall_y_clamped=wall_y_clamped,
            nw_high_target=is_nw_high_target,
        )

        # 접근 중 잎/과실을 집게로 미는 것을 줄이기 위해 수평 진입 전에
        # 파지 파츠를 600으로 명시적으로 열어 둔다.
        self._gripper_client.open_for_stem_descent()

        self._register_neighbor_obstacles_fn(straw)
        self._motion_gen.detach_object_from_robot()

        if raw_straw[0] < -0.30 and not self._measured_tcp_model:
            straw[0] += LEFTMOST_GRASP_X_CORR_M

        grasp_search, n_offsets, n_quats, tool_finish_executed_m, tool_finish_executed_dir = (
            self.search_grasp(
                straw, is_nw_high_target, input_quat_wxyz,
                grasp_retry_offsets, crane_z_offset_m,
            )
        )

        ret_pre = grasp_search.ret_pre
        ret_grasp = grasp_search.ret_grasp
        used_grasp_offset = grasp_search.grasp_offset_m
        used_grasp_variant = grasp_search.grasp_variant
        used_approach_dir = grasp_search.approach_dir
        used_grasp_quat = grasp_search.grasp_quat
        used_pre_ee_pos = grasp_search.pre_ee_pos
        used_grasp_ee_pos = grasp_search.grasp_ee_pos
        measured_best_depth_m = grasp_search.measured_best_depth_m

        if ret_pre is not None and leftmost_depth_limited(raw_straw[0], used_grasp_offset):
            self._log().warn(
                f"LEFTMOST_DEPTH_LIMITED: deeper 30/35/40/45mm endpoints rejected; "
                f"using {used_grasp_offset*1000:.0f}mm stand-off")
            self.runtime_log.log(
                "leftmost_depth_limited",
                selected_grasp_offset_m=used_grasp_offset,
                attempted_offsets_m=leftmost_rejected_offsets(used_grasp_offset),
                reason="deeper_endpoints_rejected",
            )

        if ret_pre is None:
            self._log().error(
                f"ABORT: grasp 전체 실패 — {grasp_search.attempt_count}개 후보 모두 reject "
                f"(target=({straw[0]*1000:.0f},{straw[1]*1000:.0f},{straw[2]*1000:.0f})mm "
                f"start_J=[{', '.join(f'{np.rad2deg(v):.0f}' for v in self._current_joints())}]°)")
            self._abort_pick_with_complete_fn()
            return

        if self._measured_tcp_model and self._measured_tcp_plan_only:
            self._log().warn(
                "MEASURED_TCP_PLAN_ONLY: valid pre-approach found and guarded "
                f"{(PRE_APPROACH_OFFSET - used_grasp_offset)*1000:.0f}mm final "
                "MoveLine prepared; "
                "no robot motion dispatched. Set measured_tcp_plan_only:=false only "
                "after reviewing the target and keeping E-stop ready.")
            self.runtime_log.log(
                "measured_tcp_plan_only_hold",
                grasp_offset_m=used_grasp_offset,
                grasp_variant=used_grasp_variant,
                approach_dir=used_approach_dir,
                planned_pre_endpoint_rad=ret_pre[0][-1].tolist(),
                planned_grasp_endpoint_rad=ret_grasp[0][-1].tolist(),
                final_standoff_m=used_grasp_offset,
                guarded_final_move_line_m=PRE_APPROACH_OFFSET - used_grasp_offset,
                pick_complete_published=False,
            )
            self._clear_neighbor_obstacles_fn()
            self._log().warn(
                "MEASURED_TCP_PLAN_ONLY_HOLD: /pick_complete was not published, "
                "so the scan executor must not return home or advance automatically.")
            return

        # pre-approach 실행 후 직선 진입
        final_approach_distance = self._compute_final_approach_distance_fn(
            raw_straw,
            straw,
            used_pre_ee_pos,
            used_approach_dir,
            used_grasp_offset,
            is_nw_high_target,
            wall_y_clamped,
        )
        final_state = FinalApproachState(
            final_approach_distance,
            used_grasp_ee_pos,
            tool_finish_executed_m,
            tool_finish_executed_dir,
        )
        if not self._execute_spline(*ret_pre):
            self._log().error("ABORT: pre-approach spline 실패")
            self._abort_pick_with_complete_fn()
            return
        self._log().info(
            f"PRE_APPROACH_REACHED — settling {PRE_APPROACH_SETTLE_SEC:.1f}s "
            f"before {final_approach_distance*1000:.0f}mm straight approach")
        time.sleep(PRE_APPROACH_SETTLE_SEC)

        if not self._execute_final_approach_fn(
            final_state,
            final_approach_distance,
            ret_grasp,
            measured_best_depth_m,
            used_pre_ee_pos,
            used_grasp_quat,
            used_grasp_variant,
            used_approach_dir,
        ):
            return

        # 실기 확인: 모든 벽면 딸기 줄기는 모델 벽 앞면보다 ~30mm 안쪽에 위치.
        # wall_margin=-30mm이면 available = offset+30mm → 80mm extra 자동 실행.
        # rightmost(x>250mm)는 offsets[-0.03, 0.0]으로 이미 깊게 진입하므로 제외.
        extra_ok, extra_advance_m, used_grasp_ee_pos = (
            self.execute_leftmost_extra_advance_if_needed(
                raw_straw[0],
                used_grasp_offset,
                used_approach_dir,
                final_state.grasp_ee_pos,
            )
        )
        final_approach_distance = final_state.distance_m
        tool_finish_executed_m = final_state.tool_finish_executed_m
        tool_finish_executed_dir = final_state.tool_finish_executed_dir
        if not extra_ok:
            return

        grasp_joints = (
            list(self._current_joints())
            if self._current_joints() is not None
            else ret_grasp[0][-1].tolist()
        )
        self._log().info(
            f"GRASP_POSE_REACHED — offset={used_grasp_offset:+.3f}m "
            f"pre={PRE_APPROACH_OFFSET*100:.0f}cm+{final_approach_distance*1000:.0f}mm+{extra_advance_m*1000:.0f}mm "
            f"variant={used_grasp_variant} elevation={np.degrees(np.arcsin(np.clip(used_approach_dir[2], -1.0, 1.0))):+.1f}deg "
            f"(attempt {grasp_search.attempt_count}/{n_offsets * n_quats})")
        self.runtime_log.log(
            "grasp_pose_reached",
            grasp_offset_m=used_grasp_offset,
            grasp_variant=used_grasp_variant,
            approach_dir=used_approach_dir,
            extra_advance_m=extra_advance_m,
            current_joints_rad=self._current_joints(),
        )

        # 수평 진입 완료 후 열린 그리퍼로 줄기를 따라 KP1까지 하강한다.
        if not self.execute_open_stem_descent_if_needed(
                crane_z_offset_m, float(straw[2]), used_grasp_ee_pos, used_grasp_variant):
            return

        if not self.execute_nw_base_y_nudge_if_needed(
                is_nw_high_target, float(raw_straw[2])):
            return

        # 3. 그리퍼 닫기 + 파지 확인
        grasp_result, present_pos, present_current_raw, grasp_reason = (
            self._close_and_verify_grasp_fn())
        if grasp_result == "GRIPPER_CLOSE_FAILED":
            self.handle_gripper_close_failed(
                final_approach_distance,
                extra_advance_m,
                tool_finish_executed_m,
                tool_finish_executed_dir,
                used_approach_dir,
            )
            return
        # 4. BASE -Z 당기기로 줄기 분리 후 직선 역진 retreat
        retreat_joints = self.execute_detach_and_retreat(
            final_approach_distance,
            extra_advance_m,
            tool_finish_executed_m,
            tool_finish_executed_dir,
            used_approach_dir,
            grasp_joints,
        )
        if retreat_joints is None:
            return

        # 4b. VERIFY_DETACH
        detach_result = "DETACH_UNVERIFIED"
        self.runtime_log.log(
            "verify_detach",
            result_code=detach_result,
            grasp_result=grasp_result,
            retreat_policy="pitch_detach_then_straight_reverse",
            reason="no sensor; pitch detach executed",
        )

        place_handled, return_start_joints = self.maybe_execute_place_after_retreat(
            grasp_result,
            retreat_joints,
        )
        if place_handled:
            return

        self.return_to_pick_start_and_complete(
            return_start_joints,
            pick_start_joints,
            grasp_result,
            detach_result,
        )
