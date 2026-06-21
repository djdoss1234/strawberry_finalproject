#!/usr/bin/env python3
"""Tray place execution for the harvest planner.

This wraps the two post-retreat place sequences (taught Slot0 grid reference,
and marker/tray-localization based place) that must call the live cuRobo
plan()/execute_spline() and the Doosan motion services. Kept as a thin
node-dependent class (like HarvestGripperClient / GraspSearchExecutor) rather
than pure functions, since both sequences dispatch real robot motion.

Row2 (every third slot) has a separate, currently-unresolved accuracy issue
(see project memory) — this module moves that logic unchanged; it does not
attempt to fix it.
"""

import numpy as np
from scipy.spatial.transform import Rotation as SciR

from harvest_motion_params import (
    GRIPPER_PLACE_RELEASE_POS,
    MAX_TAUGHT_PLACE_TRANSFER_JOINT_DELTA_DEG,
    TAUGHT_SLOT0_PLACE_REFERENCE_JOINTS_DEG,
    TAUGHT_SLOT0_PLACE_REFERENCE_POSX_MM_DEG,
    TAUGHT_SLOT0_VERTICAL_VEL_MM_S,
    TAUGHT_TRAY_SLOT_COUNT,
    TRAY_VIEW_JOINTS_DEG,
)
from marker_place_orientation_policy import (
    doosan_zyz_to_wxyz,
    marker_place_candidate_target,
    marker_place_orientation_candidates,
    unique_clearance_candidates,
)
from row2_place_policy import row2_line_check_result


class TrayPlaceExecutor:
    """Runs the taught-slot0-grid and marker-based place sequences."""

    def __init__(self, node, runtime_log, plan_fn, execute_spline_fn,
                 execute_base_z_relative_fn, plan_to_fixed_joints_pose_fn,
                 overview_joints_near_current_fn, nearest_equivalent_joints_fn,
                 curobo_fk_ee_pose_fn, trajectory_line_deviation_fn,
                 ensure_operation_speed_fn, set_gripper_position_fn,
                 current_joints_getter, slot_idx_getter, tray_place_policy,
                 use_taught_slot0_place_reference: bool,
                 execute_marker_place_release: bool,
                 allow_generated_tray_slot_release: bool,
                 measured_tcp_model: bool,
                 marker_place_above_clearance_m: float,
                 taught_slot_above_clearance_m: float,
                 row2_place_pitch_tilt_deg: float,
                 row2_release_correction_mm,
                 row2_max_line_deviation_mm: float):
        self.node = node
        self.runtime_log = runtime_log
        self._plan = plan_fn
        self._execute_spline = execute_spline_fn
        self._execute_base_z_relative = execute_base_z_relative_fn
        self._plan_to_fixed_joints_pose = plan_to_fixed_joints_pose_fn
        self._overview_joints_near_current = overview_joints_near_current_fn
        self._nearest_equivalent_joints = nearest_equivalent_joints_fn
        self._curobo_fk_ee_pose = curobo_fk_ee_pose_fn
        self._trajectory_line_deviation_mm = trajectory_line_deviation_fn
        self._ensure_operation_speed = ensure_operation_speed_fn
        self._set_gripper_position = set_gripper_position_fn
        self._current_joints_getter = current_joints_getter
        self._slot_idx_getter = slot_idx_getter
        self.tray_place_policy = tray_place_policy
        self.use_taught_slot0_place_reference = use_taught_slot0_place_reference
        self.execute_marker_place_release = execute_marker_place_release
        self.allow_generated_tray_slot_release = allow_generated_tray_slot_release
        self.measured_tcp_model = measured_tcp_model
        self.marker_place_above_clearance_m = marker_place_above_clearance_m
        self.taught_slot_above_clearance_m = taught_slot_above_clearance_m
        self.row2_place_pitch_tilt_deg = row2_place_pitch_tilt_deg
        self.row2_release_correction_mm = row2_release_correction_mm
        self.row2_max_line_deviation_mm = row2_max_line_deviation_mm

    def _log(self):
        return self.node.get_logger()

    @property
    def _current_joints(self):
        return self._current_joints_getter()

    def execute_marker_place_after_retreat(self, retreat_joints):
        """Marker-derived place. Release 승인 전에는 above에서 정지한다.

        Caller is responsible for advancing the slot index on a "success"
        result (the increment used to live here; lifted out so this class
        only reads the index via slot_idx_getter).
        """
        if self.use_taught_slot0_place_reference:
            return self.execute_taught_slot0_place_reference_after_retreat(
                retreat_joints)

        target = self.tray_place_policy.load_marker_place_target(
            self._slot_idx_getter())
        if target is None:
            return "skip", retreat_joints   # tray 없음/stale → soft skip, hold 없음

        self._log().info(
            f"5 marker place slot={target['slot_index']} via overview/tray-view "
            f"source={target['target_source']}")
        overview_deg = self._overview_joints_near_current()
        ok, overview_joints = self._plan_to_fixed_joints_pose(
            retreat_joints, overview_deg, "marker place transfer overview",
            skip_swing_check=True)
        if not ok:
            self._log().error(
                "MARKER_PLACE_BLOCKED: transfer overview plan failed; holding fruit")
            return "failed", retreat_joints

        tray_view_deg = self._nearest_equivalent_joints(TRAY_VIEW_JOINTS_DEG)
        ok, tray_view_joints = self._plan_to_fixed_joints_pose(
            overview_joints, tray_view_deg, "marker place tray view",
            skip_swing_check=True)
        if not ok:
            self._log().error(
                "MARKER_PLACE_BLOCKED: tray-view plan failed; holding fruit")
            return "failed", overview_joints

        # Tray localization은 Doosan controller TCP orientation을 저장하지만 cuRobo
        # measured grasp_tcp_link와 convention/model 차이로 그대로는 IK_FAIL이 난다.
        # 도달 가능한 place orientation을 preview ABOVE에서 먼저 선택한다.
        tray_view_fk_pos, tray_view_quat = self._curobo_fk_ee_pose(tray_view_joints)
        json_place_quat = doosan_zyz_to_wxyz(*target["above"][3:])
        quat_dot = min(1.0, abs(float(np.dot(tray_view_quat, json_place_quat))))
        quat_delta_deg = float(np.rad2deg(2.0 * np.arccos(quat_dot)))
        self._log().info(
            f"MARKER_PLACE_ORIENTATION_SEARCH tray-view/json delta="
            f"{quat_delta_deg:.1f}deg")

        # ABOVE: 하향 place 자세 후보 중 도달 가능한 첫 경로를 선택한다. Measured
        # TCP는 orientation마다 파츠 끝->파지 중심 10mm 방향이 달라지므로 후보별로
        # release/above 위치를 contact point에서 다시 계산한다. Tray contact가 이미
        # 면에서 60mm 위이므로, 100mm ABOVE가 작업반경 경계에서 IK_FAIL이면 더 낮은
        # clearance도 안전 후보로 탐색한다.
        default_above_pos_m = [v / 1000.0 for v in target["above"][:3]]
        self._log().info(
            f"MARKER_PLACE_ABOVE cuRobo "
            f"xyz={[round(v, 1) for v in target['above'][:3]]}mm "
            f"abc={[round(v, 1) for v in target['above'][3:]]}deg")
        above_plan = None
        selected_orientation_name = None
        above_quat = None
        selected_release_pos_m = None
        selected_above_pos_m = None
        requested_clearance = self.marker_place_above_clearance_m
        clearance_candidates = unique_clearance_candidates(requested_clearance)

        for clearance_m in clearance_candidates:
            for orientation_name, candidate_quat in marker_place_orientation_candidates(
                    tray_view_quat, default_above_pos_m):
                candidate = marker_place_candidate_target(
                    target,
                    candidate_quat,
                    clearance_m,
                    requested_clearance,
                    self.measured_tcp_model,
                )
                self._log().info(
                    f"MARKER_PLACE_ABOVE trying clearance={clearance_m*1000:.0f}mm "
                    f"orientation={orientation_name} "
                    f"tcp_r={candidate['tcp_radius_m']:.3f}m "
                    f"flange_r={candidate['flange_radius_m']:.3f}m "
                    f"goal_mm={[round(v * 1000, 1) for v in candidate['above_pos_m']]}")
                candidate_plan = self._plan(
                    tray_view_joints, candidate["above_pos_m"], candidate_quat,
                    num_ik_seeds=64, max_attempts=3, timeout_sec=2.0)
                if candidate_plan is not None:
                    above_plan = candidate_plan
                    above_quat = candidate_quat
                    selected_orientation_name = orientation_name
                    selected_release_pos_m = candidate["release_pos_m"]
                    selected_above_pos_m = candidate["above_pos_m"]
                    selected_clearance_m = clearance_m
                    break
            if above_plan is not None:
                break
        if above_plan is None:
            self._log().error(
                "MARKER_PLACE_BLOCKED: all above orientation candidates failed; "
                "holding fruit")
            return "failed", tray_view_joints
        self._log().info(
            f"MARKER_PLACE_ORIENTATION selected={selected_orientation_name} "
            f"clearance={selected_clearance_m*1000:.0f}mm")
        self.runtime_log.log(
            "marker_place_orientation_selected",
            source=selected_orientation_name,
            tray_view_fk_pos_m=tray_view_fk_pos,
            selected_quat_wxyz=above_quat,
            selected_above_pos_m=selected_above_pos_m,
            selected_release_pos_m=selected_release_pos_m,
            selected_clearance_m=selected_clearance_m,
            tray_view_quat_wxyz=tray_view_quat,
            json_quat_wxyz=json_place_quat,
            tray_view_json_angular_delta_deg=quat_delta_deg,
        )
        ok_above = self._execute_spline(*above_plan)
        above_joints = list(
            above_plan[0][-1].tolist() if ok_above else tray_view_joints)
        if not ok_above:
            self._log().error("MARKER_PLACE_BLOCKED: above spline exec failed; holding fruit")
            return "failed", tray_view_joints

        if not self.execute_marker_place_release:
            self._log().warn(
                "MARKER_PLACE_PREVIEW_HOLD: above reached; release disabled. "
                "Inspect clearance before enabling execute_marker_place_release.")
            return "preview_hold", list(self._current_joints or above_joints)

        # RELEASE: cuRobo Cartesian plan — avoids kinematic flip caused by BASE ABS
        release_pos_m = selected_release_pos_m
        release_quat = above_quat
        self._log().info(
            f"MARKER_PLACE_RELEASE_DESCEND cuRobo "
            f"xyz={[round(v, 1) for v in target['release'][:3]]}mm "
            f"abc={[round(v, 1) for v in target['release'][3:]]}deg")
        release_plan = self._plan(above_joints, release_pos_m, release_quat)
        if release_plan is None:
            self._log().error(
                "MARKER_PLACE_BLOCKED: release cuRobo plan failed; holding fruit")
            return "failed", list(self._current_joints or above_joints)
        ok_release = self._execute_spline(*release_plan)
        if not ok_release:
            self._log().error(
                "MARKER_PLACE_BLOCKED: release spline exec failed; holding fruit")
            return "failed", list(self._current_joints or above_joints)
        release_joints = list(release_plan[0][-1].tolist())

        self._log().info(
            f"6 marker place release gripper position_cmd={GRIPPER_PLACE_RELEASE_POS}")
        self.runtime_log.log(
            "gripper_command", command="set_position",
            position=GRIPPER_PLACE_RELEASE_POS,
            slot_index=target["slot_index"])
        self._set_gripper_position(GRIPPER_PLACE_RELEASE_POS, timeout_sec=3.0)

        # RETREAT: release pose에서 먼저 above로 상승한 뒤 tray-view로 복귀한다.
        # release에서 tray-view 관절 자세로 바로 이동하면 tray body를 가로지를 수 있다.
        above_retreat_plan = self._plan(
            release_joints, selected_above_pos_m, above_quat)
        if above_retreat_plan is None or not self._execute_spline(*above_retreat_plan):
            self._log().error(
                "MARKER_PLACE_RELEASED_BUT_ABOVE_RETREAT_FAILED: holding position")
            return "failed_after_release", list(self._current_joints or release_joints)
        above_retreat_joints = list(above_retreat_plan[0][-1].tolist())

        # Known tray-view configuration으로 collision-aware joint-space 복귀.
        tray_view_deg_retreat = self._nearest_equivalent_joints(TRAY_VIEW_JOINTS_DEG)
        ok_retreat, _ = self._plan_to_fixed_joints_pose(
            above_retreat_joints, tray_view_deg_retreat,
            "MARKER_PLACE_TRAY_VIEW_RETURN")
        if not ok_retreat:
            self._log().error(
                "MARKER_PLACE_RELEASED_BUT_RETREAT_FAILED: holding position")
            return "failed_after_release", list(
                self._current_joints or above_retreat_joints)

        self.runtime_log.log(
            "marker_place_complete",
            result_code="PLACE_SEQUENCE_COMPLETE_UNVERIFIED",
            slot_index=target["slot_index"],
            tray_cells_json=target["path"],
        )
        return "success", list(self._current_joints or tray_view_joints)

    def execute_taught_slot0_place_reference_after_retreat(self, retreat_joints):
        """Slot0 FK와 실측 격자 벡터로 생성한 슬롯에 수직 Place한다."""
        slot_index = self._slot_idx_getter()
        if not 0 <= slot_index < TAUGHT_TRAY_SLOT_COUNT:
            self._log().error(
                f"TAUGHT_TRAY_PLACE_COMPLETE: slot index {slot_index} out of range")
            return "tray_complete", retreat_joints
        self._log().warn(
            f"TAUGHT_TRAY_GRID_PLACE active: slot={slot_index}; fixed tray pose only; "
            "marker localization is bypassed")

        reference_deg = self._nearest_equivalent_joints(
            TAUGHT_SLOT0_PLACE_REFERENCE_JOINTS_DEG)
        reference_rad = np.deg2rad(reference_deg).tolist()
        release_fk_pos_m, release_fk_quat = self._curobo_fk_ee_pose(reference_rad)
        is_row2 = (slot_index % 3 == 2)
        if is_row2 and self.row2_place_pitch_tilt_deg != 0.0:
            w, x, y, z = release_fk_quat
            base_rot = SciR.from_quat([x, y, z, w])
            tilt_rot = SciR.from_euler('y', self.row2_place_pitch_tilt_deg, degrees=True)
            tilted = tilt_rot * base_rot
            q = tilted.as_quat()
            release_fk_quat = [float(q[3]), float(q[0]), float(q[1]), float(q[2])]
            self._log().info(
                f"ROW2_PLACE_TILT: {self.row2_place_pitch_tilt_deg:.1f}deg pitch "
                f"quat_wxyz={[round(v, 4) for v in release_fk_quat]}")
        slot_offset_m = self.tray_place_policy.taught_grid_slot_offset_m(slot_index)
        release_pos_m = (
            np.array(release_fk_pos_m, dtype=float)
            + np.array(slot_offset_m, dtype=float)
        ).tolist()
        if is_row2 and any(v != 0.0 for v in self.row2_release_correction_mm):
            corr_m = np.array(self.row2_release_correction_mm, dtype=float) / 1000.0
            release_pos_m = (np.array(release_pos_m, dtype=float) + corr_m).tolist()
            self._log().info(
                f"ROW2_RELEASE_CORRECTION: {self.row2_release_correction_mm}mm "
                f"→ release_pos={[round(v*1000,1) for v in release_pos_m]}mm")
        above_pos_m = list(release_pos_m)
        clearance_m = self.taught_slot_above_clearance_m
        above_pos_m[2] += clearance_m
        self._log().info(
            f"TAUGHT_TRAY_SLOT{slot_index}_ABOVE generated from Slot0 FK + grid offset: "
            f"clearance={clearance_m*1000:.0f}mm "
            f"goal_mm={[round(v * 1000, 1) for v in above_pos_m]}")
        self._ensure_operation_speed(30)
        above_plan = self._plan(
            retreat_joints, above_pos_m, release_fk_quat,
            num_ik_seeds=64, max_attempts=3, timeout_sec=2.0,
            max_joint_delta_deg=MAX_TAUGHT_PLACE_TRANSFER_JOINT_DELTA_DEG)
        if above_plan is None or not self._execute_spline(*above_plan):
            self._log().error(
                f"TAUGHT_TRAY_SLOT{slot_index}_PLACE_BLOCKED: "
                "above plan failed; holding fruit")
            return "failed", retreat_joints
        above_joints = list(above_plan[0][-1].tolist())

        row2_descent_plan = None
        if is_row2:
            # Preview에서도 release 경로와 측방 편차를 계산해 실제 하강 전에
            # 안전성을 확인할 수 있게 한다.
            self._log().info(
                "TAUGHT_SLOT0_RELEASE_DESCEND cuRobo continuous (row2): "
                "plan once and validate Cartesian line deviation")
            row2_descent_plan = self._plan(
                above_joints, release_pos_m, release_fk_quat,
                num_ik_seeds=64, max_attempts=3, timeout_sec=2.0,
                max_joint_delta_deg=MAX_TAUGHT_PLACE_TRANSFER_JOINT_DELTA_DEG)
            if row2_descent_plan is None:
                self._log().error(
                    f"TAUGHT_TRAY_SLOT{slot_index}_PLACE_BLOCKED: "
                    "release descent plan failed; holding fruit")
                return "failed", list(self._current_joints or above_joints)
            row2_traj, _ = row2_descent_plan
            line_deviation_mm, deviation_index = self._trajectory_line_deviation_mm(
                row2_traj, above_pos_m, release_pos_m)
            line_check = row2_line_check_result(
                "descent",
                slot_index,
                line_deviation_mm,
                self.row2_max_line_deviation_mm,
                deviation_index,
            )
            self._log().info(
                f"ROW2_DESCENT_LINE_CHECK "
                f"max_deviation={line_check['max_deviation_mm']:.1f}mm "
                f"limit={line_check['limit_mm']:.1f}mm "
                f"waypoint={line_check['max_deviation_waypoint']}")
            self.runtime_log.log(
                "row2_cartesian_line_check",
                **line_check,
            )
            if not line_check["ok"]:
                self._log().error(
                    f"TAUGHT_TRAY_SLOT{slot_index}_PLACE_BLOCKED: row2 descent "
                    f"deviates {line_check['max_deviation_mm']:.1f}mm from Cartesian "
                    f"line (limit {line_check['limit_mm']:.1f}mm)")
                return "failed", list(self._current_joints or above_joints)

        self.runtime_log.log(
            "taught_slot0_place_above_reached",
            slot_index=slot_index,
            release_enabled=self.execute_marker_place_release,
            reference_joints_deg=reference_deg,
            reference_posx_mm_deg=TAUGHT_SLOT0_PLACE_REFERENCE_POSX_MM_DEG,
            slot_offset_m=slot_offset_m,
            generated_release_pos_m=release_pos_m,
            above_pos_m=above_pos_m,
            above_clearance_m=clearance_m,
        )
        if not self.execute_marker_place_release:
            self._log().warn(
                f"TAUGHT_TRAY_SLOT{slot_index}_PLACE_PREVIEW_HOLD: "
                "above reached; release disabled")
            return "preview_hold", list(self._current_joints or above_joints)

        if slot_index != 0 and not self.allow_generated_tray_slot_release:
            self._log().error(
                f"TAUGHT_TRAY_SLOT{slot_index}_RELEASE_BLOCKED: slot pose is generated "
                "from the Slot0/1/3 grid and has not been physically taught/verified. "
                "Holding at Above; teach the actual slot release pose before enabling.")
            self.runtime_log.log(
                "generated_tray_slot_release_blocked",
                slot_index=slot_index,
                reason="generated_slot_not_physically_verified",
                above_pos_m=above_pos_m,
                release_pos_m=release_pos_m,
            )
            return "preview_hold", list(self._current_joints or above_joints)

        if is_row2:
            # Preview에서 검증한 동일 궤적을 정지 없이 단일 spline으로 실행한다.
            # 독립 hop/분할 실행은 J4 branch 전환과 구간별 정지를 유발하므로
            # 사용하지 않는다.
            self._log().info(
                "TAUGHT_SLOT0_RELEASE_DESCEND cuRobo continuous (row2): "
                "executing validated one-spline trajectory")
            full_traj, full_time = row2_descent_plan
            if not self._execute_spline(full_traj, full_time):
                self._log().error(
                    f"TAUGHT_TRAY_SLOT{slot_index}_PLACE_BLOCKED: "
                    "continuous descent spline failed; holding fruit")
                return "failed", list(self._current_joints or above_joints)
            release_joints = list(full_traj[-1].tolist())
        else:
            self._log().info(
                f"TAUGHT_SLOT0_RELEASE_DESCEND BASE -Z {round(clearance_m*1000)}mm")
            if not self._execute_base_z_relative(
                    -clearance_m,
                    "TAUGHT_SLOT0_RELEASE_DESCEND",
                    TAUGHT_SLOT0_VERTICAL_VEL_MM_S):
                self._log().error(
                    f"TAUGHT_TRAY_SLOT{slot_index}_PLACE_BLOCKED: "
                    "vertical release descend failed; holding fruit")
                return "failed", list(self._current_joints or above_joints)
            release_joints = list(self._current_joints or above_joints)

        self._log().warn(
            f"TAUGHT_TRAY_SLOT{slot_index}_PLACE_RELEASE: "
            f"position_cmd={GRIPPER_PLACE_RELEASE_POS}")
        self.runtime_log.log(
            "gripper_command", command="set_position",
            position=GRIPPER_PLACE_RELEASE_POS, slot_index=slot_index,
            source="taught_slot0_grid_reference")
        self._set_gripper_position(GRIPPER_PLACE_RELEASE_POS, timeout_sec=3.0)

        if is_row2:
            self._log().info(
                "TAUGHT_SLOT0_RELEASE_ASCEND cuRobo continuous (row2): "
                "plan once, validate Cartesian line deviation, execute one spline")
            ascent_plan = self._plan(
                release_joints, above_pos_m, release_fk_quat,
                num_ik_seeds=64, max_attempts=3, timeout_sec=2.0,
                max_joint_delta_deg=MAX_TAUGHT_PLACE_TRANSFER_JOINT_DELTA_DEG)
            if ascent_plan is None:
                self._log().error(
                    f"TAUGHT_TRAY_SLOT{slot_index}_RELEASED_BUT_ASCEND_FAILED: "
                    "ascent plan failed; holding position")
                return "failed_after_release", list(self._current_joints or release_joints)
            asc_traj, asc_time = ascent_plan
            line_deviation_mm, deviation_index = self._trajectory_line_deviation_mm(
                asc_traj, release_pos_m, above_pos_m)
            line_check = row2_line_check_result(
                "ascent",
                slot_index,
                line_deviation_mm,
                self.row2_max_line_deviation_mm,
                deviation_index,
            )
            self._log().info(
                f"ROW2_ASCENT_LINE_CHECK "
                f"max_deviation={line_check['max_deviation_mm']:.1f}mm "
                f"limit={line_check['limit_mm']:.1f}mm "
                f"waypoint={line_check['max_deviation_waypoint']}")
            self.runtime_log.log(
                "row2_cartesian_line_check",
                **line_check,
            )
            if not line_check["ok"]:
                self._log().error(
                    f"TAUGHT_TRAY_SLOT{slot_index}_RELEASED_BUT_ASCEND_FAILED: "
                    f"row2 ascent deviates {line_check['max_deviation_mm']:.1f}mm "
                    f"from Cartesian line (limit {line_check['limit_mm']:.1f}mm)")
                return "failed_after_release", list(self._current_joints or release_joints)
            if not self._execute_spline(asc_traj, asc_time):
                self._log().error(
                    f"TAUGHT_TRAY_SLOT{slot_index}_RELEASED_BUT_ASCEND_FAILED: "
                    "continuous ascent spline failed; holding position")
                return "failed_after_release", list(self._current_joints or release_joints)
        else:
            if not self._execute_base_z_relative(
                    clearance_m,
                    "TAUGHT_SLOT0_RELEASE_ASCEND",
                    TAUGHT_SLOT0_VERTICAL_VEL_MM_S):
                self._log().error(
                    f"TAUGHT_TRAY_SLOT{slot_index}_RELEASED_BUT_ASCEND_FAILED: "
                    "holding position")
                return "failed_after_release", list(self._current_joints or above_joints)

        self.runtime_log.log(
            "marker_place_complete",
            result_code="PLACE_SEQUENCE_COMPLETE_UNVERIFIED",
            slot_index=slot_index,
            source="taught_slot0_grid_reference",
        )
        return "success", list(self._current_joints or above_joints)
