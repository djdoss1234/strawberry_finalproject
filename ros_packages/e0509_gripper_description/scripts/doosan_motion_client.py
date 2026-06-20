#!/usr/bin/env python3
"""Thin Doosan motion-service wrapper used by the harvest planner.

This module intentionally keeps the existing service contracts and timing
guards unchanged. It only moves controller I/O out of curobo_planner_node.py so
the planner node can focus on target selection and task logic.
"""

import time

import numpy as np
from dsr_msgs2.srv import MoveJoint, MoveLine, MoveSplineJoint
from std_msgs.msg import Float64MultiArray as F64MA

from harvest_motion_params import (
    DETACH_PULL_DOWN_MM,
    DETACH_PULL_VEL_MM_S,
    FINAL_APPROACH_ACC_MM_S2,
    FINAL_APPROACH_VEL_MM_S,
    MAX_SPLINE_POINTS,
    SPLINE_ACC_DEG_S2,
    SPLINE_MIN_TIME,
    SPLINE_TIME_SCALE,
    SPLINE_VEL_DEG_S,
)


class DoosanMotionClient:
    """Synchronous helpers around Doosan MoveSplineJoint/MoveJoint/MoveLine."""

    def __init__(self, node, runtime_log, cli_spline, cli_movej, cli_movel,
                 current_joints_getter):
        self.node = node
        self.runtime_log = runtime_log
        self.cli_spline = cli_spline
        self.cli_movej = cli_movej
        self.cli_movel = cli_movel
        self.current_joints_getter = current_joints_getter

    def _log(self):
        return self.node.get_logger()

    def _current_joints(self):
        return self.current_joints_getter()

    def execute_spline(self, traj_rad, motion_time: float) -> bool:
        if not self.cli_spline.wait_for_service(timeout_sec=3.0):
            self._log().error("MoveSplineJoint not available")
            return False
        traj_deg = np.rad2deg(traj_rad)
        n = traj_deg.shape[0]
        if n > MAX_SPLINE_POINTS:
            idx = np.linspace(0, n - 1, MAX_SPLINE_POINTS, dtype=int)
            traj_deg = traj_deg[idx]
            n = MAX_SPLINE_POINTS

        req = MoveSplineJoint.Request()
        req.pos_cnt = n
        for row in traj_deg:
            pt = F64MA()
            pt.data = row.tolist()
            req.pos.append(pt)
        req.vel = [SPLINE_VEL_DEG_S] * 6
        req.acc = [SPLINE_ACC_DEG_S2] * 6
        req.time = max(float(motion_time) * SPLINE_TIME_SCALE, SPLINE_MIN_TIME)
        req.mode = 0
        req.sync_type = 0

        self._log().info(
            f"Spline {n}pts plan={motion_time:.2f}s exec={req.time:.2f}s "
            f"-> end={[f'{v:.1f}' for v in traj_deg[-1]]}deg")
        self.runtime_log.log(
            "motion_command",
            controller="doosan_move_spline_joint",
            service="/dsr01/motion/move_spline_joint",
            trajectory_deg=traj_deg,
            planned_motion_time_sec=motion_time,
            requested_time_sec=req.time,
            velocity_deg_s=req.vel,
            acceleration_deg_s2=req.acc,
        )
        future = self.cli_spline.call_async(req)
        t0 = time.time()
        while not future.done() and (time.time() - t0) < 60.0:
            time.sleep(0.05)

        ok = future.done() and future.result() and future.result().success
        if not ok:
            self._log().error("Spline failed/timeout")
        self.runtime_log.log(
            "motion_result",
            controller="doosan_move_spline_joint",
            success=bool(ok),
            current_joints_rad=self._current_joints(),
        )
        return ok

    def execute_tool_z_line(self, distance_m: float,
                            motion_label="FINAL_APPROACH_STRAIGHT",
                            vel_mm_s: float = None,
                            acc_mm_s2: float = None,
                            min_distance_m: float = 0.02) -> bool:
        """Move straight along current TOOL Z while preserving TCP orientation."""
        if not min_distance_m <= abs(distance_m) <= 0.25:
            self._log().error(
                f"MoveLine rejected: {motion_label} distance={distance_m*1000:.1f}mm "
                f"allowed={min_distance_m*1000:.1f}..250.0mm")
            return False
        if not self.cli_movel.wait_for_service(timeout_sec=3.0):
            self._log().error("MoveLine not available")
            return False

        vel = vel_mm_s if vel_mm_s is not None else FINAL_APPROACH_VEL_MM_S
        acc = acc_mm_s2 if acc_mm_s2 is not None else FINAL_APPROACH_ACC_MM_S2

        req = MoveLine.Request()
        req.pos = [0.0, 0.0, float(distance_m * 1000.0), 0.0, 0.0, 0.0]
        req.vel = [vel, 10.0]
        req.acc = [acc, 20.0]
        req.time = 0.0
        req.radius = 0.0
        req.ref = 1
        req.mode = 1
        req.blend_type = 0
        req.sync_type = 0

        self._log().info(
            f"{motion_label} TOOL {'+Z' if distance_m > 0 else '-Z'} "
            f"{abs(distance_m)*1000:.1f}mm vel={vel:.1f}mm/s")
        self.runtime_log.log(
            "motion_command",
            controller="doosan_move_line",
            label=motion_label,
            service="/dsr01/motion/move_line",
            reference_frame="tool",
            relative_pose_mm_deg=req.pos,
            velocity=req.vel,
            acceleration=req.acc,
        )
        start_joints = self._current_joints_array()
        future = self.cli_movel.call_async(req)
        t0 = time.time()
        nominal_motion_sec = abs(distance_m * 1000.0) / max(float(vel), 1.0)
        timeout_sec = max(30.0, nominal_motion_sec * 10.0 + 10.0)
        while not future.done() and (time.time() - t0) < timeout_sec:
            time.sleep(0.05)
        ok = future.done() and future.result() and future.result().success
        elapsed_sec = time.time() - t0
        min_expected_sec = max(0.5, nominal_motion_sec * 0.6)
        if ok and elapsed_sec < min_expected_sec:
            remaining_sec = min_expected_sec - elapsed_sec
            self._log().warn(
                f"{motion_label} MoveLine returned early "
                f"({elapsed_sec:.2f}s < expected {min_expected_sec:.2f}s); "
                f"waiting {remaining_sec:.2f}s before continuing")
            time.sleep(remaining_sec)
        if ok and start_joints is not None and abs(distance_m) > 0.05:
            ok = self._reject_no_motion_if_needed(
                start_joints, motion_label, "MoveLine", min_delta_deg=0.5)
        if not ok:
            self._log().error(
                f"{motion_label} MoveLine failed/timeout>{timeout_sec:.1f}s")
        self.runtime_log.log(
            "motion_result",
            controller="doosan_move_line",
            label=motion_label,
            success=bool(ok),
            timeout_sec=timeout_sec,
            elapsed_sec=elapsed_sec,
            current_joints_rad=self._current_joints(),
        )
        return ok

    def execute_pitch_detach(self) -> bool:
        """Pull TCP down in BASE -Z after grasp to detach the stem."""
        if not self.cli_movel.wait_for_service(timeout_sec=3.0):
            self._log().warn("DETACH_PULL: MoveLine service unavailable")
            return False
        req = MoveLine.Request()
        req.pos = [0.0, 0.0, -float(DETACH_PULL_DOWN_MM), 0.0, 0.0, 0.0]
        req.vel = [float(DETACH_PULL_VEL_MM_S), 10.0]
        req.acc = [30.0, 20.0]
        req.time = 0.0
        req.radius = 0.0
        req.ref = 0
        req.mode = 1
        req.blend_type = 0
        req.sync_type = 0
        future = self.cli_movel.call_async(req)
        t0 = time.time()
        while not future.done() and (time.time() - t0) < 30.0:
            time.sleep(0.05)
        ok = future.done() and bool(future.result() and future.result().success)
        self._log().info(
            f"DETACH_PULL_DOWN: BASE -Z {DETACH_PULL_DOWN_MM:.0f}mm "
            f"-> {'OK' if ok else 'FAIL'}")
        self.runtime_log.log(
            "detach_pull_down",
            pull_mm=DETACH_PULL_DOWN_MM,
            vel_mm_s=DETACH_PULL_VEL_MM_S,
            success=ok,
        )
        return ok

    def execute_base_z_relative(self, distance_m: float, motion_label: str,
                                vel_mm_s: float = 30.0) -> bool:
        return self.execute_base_relative_line(
            [0.0, 0.0, float(distance_m)], motion_label, vel_mm_s)

    def execute_base_relative_line(self, delta_m, motion_label: str,
                                   vel_mm_s: float = 30.0,
                                   acc_mm_s2: float = 30.0) -> bool:
        """Move straight in BASE-relative XYZ."""
        if not self.cli_movel.wait_for_service(timeout_sec=3.0):
            self._log().error(f"{motion_label}: MoveLine service unavailable")
            return False
        delta_m = np.array(delta_m, dtype=float)
        distance_m = float(np.linalg.norm(delta_m))
        if not 0.005 <= distance_m <= 0.30:
            self._log().error(
                f"{motion_label}: BASE relative distance "
                f"{distance_m*1000:.1f}mm outside 5..300mm")
            return False
        req = MoveLine.Request()
        req.pos = [
            float(delta_m[0] * 1000.0),
            float(delta_m[1] * 1000.0),
            float(delta_m[2] * 1000.0),
            0.0, 0.0, 0.0,
        ]
        req.vel = [float(vel_mm_s), 10.0]
        req.acc = [float(acc_mm_s2), 20.0]
        req.time = 0.0
        req.radius = 0.0
        req.ref = 0
        req.mode = 1
        req.blend_type = 0
        req.sync_type = 0
        self._log().info(
            f"{motion_label} BASE REL "
            f"xyz={[round(v * 1000.0, 1) for v in delta_m]}mm "
            f"dist={distance_m*1000:.1f}mm vel={vel_mm_s:.1f}mm/s")
        self.runtime_log.log(
            "motion_command",
            controller="doosan_move_line",
            label=motion_label,
            service="/dsr01/motion/move_line",
            reference_frame="base",
            relative_pose_mm_deg=req.pos,
            velocity=req.vel,
        )
        start_joints = self._current_joints_array()
        future = self.cli_movel.call_async(req)
        t0 = time.time()
        nominal_motion_sec = distance_m * 1000.0 / max(float(vel_mm_s), 1.0)
        timeout_sec = max(10.0, nominal_motion_sec * 3.0 + 5.0)
        while not future.done() and (time.time() - t0) < timeout_sec:
            time.sleep(0.05)
        ok = future.done() and bool(future.result() and future.result().success)
        elapsed_sec = time.time() - t0
        min_expected_sec = max(0.5, nominal_motion_sec * 0.6)
        if ok and elapsed_sec < min_expected_sec:
            remaining_sec = min_expected_sec - elapsed_sec
            self._log().warn(
                f"{motion_label} BASE MoveLine returned early "
                f"({elapsed_sec:.2f}s < expected {min_expected_sec:.2f}s); "
                f"waiting {remaining_sec:.2f}s before continuing")
            time.sleep(remaining_sec)
        if ok and start_joints is not None and distance_m > 0.05:
            ok = self._reject_no_motion_if_needed(
                start_joints, motion_label, "BASE MoveLine", min_delta_deg=0.5)
        self.runtime_log.log(
            "motion_result",
            controller="doosan_move_line",
            label=motion_label,
            success=bool(ok),
            timeout_sec=timeout_sec,
            elapsed_sec=elapsed_sec,
        )
        if not ok:
            self._log().error(f"{motion_label} BASE relative failed/timeout")
        return ok

    def execute_base_line(self, posx_mm_deg, motion_label,
                          vel_mm_s=20.0) -> bool:
        """Move TCP to an absolute BASE posx. Used by marker/taught place."""
        if len(posx_mm_deg) != 6:
            self._log().error(f"{motion_label}: expected 6D posx")
            return False
        if not self.cli_movel.wait_for_service(timeout_sec=3.0):
            self._log().error("MoveLine not available")
            return False

        req = MoveLine.Request()
        req.pos = [float(v) for v in posx_mm_deg]
        req.vel = [float(vel_mm_s), 10.0]
        req.acc = [30.0, 20.0]
        req.time = 0.0
        req.radius = 0.0
        req.ref = 0
        req.mode = 0
        req.blend_type = 0
        req.sync_type = 0

        self._log().info(
            f"{motion_label} BASE ABS "
            f"xyz={[round(v, 1) for v in req.pos[:3]]}mm "
            f"abc={[round(v, 1) for v in req.pos[3:]]}deg")
        self.runtime_log.log(
            "motion_command",
            controller="doosan_move_line",
            label=motion_label,
            service="/dsr01/motion/move_line",
            reference_frame="base",
            absolute_pose_mm_deg=req.pos,
            velocity=req.vel,
            acceleration=req.acc,
        )
        future = self.cli_movel.call_async(req)
        t0 = time.time()
        while not future.done() and (time.time() - t0) < 60.0:
            time.sleep(0.05)
        ok = future.done() and future.result() and future.result().success
        if not ok:
            self._log().error(f"{motion_label} MoveLine failed/timeout")
        self.runtime_log.log(
            "motion_result",
            controller="doosan_move_line",
            label=motion_label,
            success=bool(ok),
            current_joints_rad=self._current_joints(),
        )
        return ok

    def movej_direct(self, joints_deg, vel=40.0, acc=60.0):
        """Direct Doosan MoveJoint fallback."""
        if not self.cli_movej.wait_for_service(timeout_sec=3.0):
            self._log().error("MoveJoint service not available")
            return False
        req = MoveJoint.Request()
        req.pos = [float(v) for v in joints_deg]
        req.vel = float(vel)
        req.acc = float(acc)
        req.time = 0.0
        req.radius = 0.0
        req.mode = 0
        req.blend_type = 0
        req.sync_type = 0
        self._log().warn(
            f"MoveJoint direct -> {[round(v, 1) for v in joints_deg]}deg vel={vel}")
        future = self.cli_movej.call_async(req)
        t0 = time.time()
        while not future.done() and (time.time() - t0) < 90.0:
            time.sleep(0.05)
        ok = future.done() and future.result() and future.result().success
        if not ok:
            self._log().error("MoveJoint direct failed/timeout")
        return ok

    def _current_joints_array(self):
        joints = self._current_joints()
        if joints is None:
            return None
        return np.array(joints, dtype=float)

    def _reject_no_motion_if_needed(self, start_joints, motion_label,
                                    command_name, min_delta_deg: float) -> bool:
        end_joints = self._current_joints_array()
        if end_joints is None:
            end_joints = start_joints
        max_delta_deg = float(np.max(np.degrees(np.abs(end_joints - start_joints))))
        if max_delta_deg >= min_delta_deg:
            return True
        self._log().error(
            f"{motion_label} {command_name} reported success but joints barely moved "
            f"(max_delta={max_delta_deg:.2f}deg); treating as failed")
        return False
