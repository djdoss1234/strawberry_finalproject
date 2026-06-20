#!/usr/bin/env python3
"""Trajectory validation and joint-equivalent normalization helpers."""

import numpy as np

from harvest_motion_params import (
    MAX_HARVEST_JOINT_DELTA_DEG,
    OPERATIONAL_JOINT_LIMITS_DEG,
    WRAP_EQUIVALENT_JOINT_IDX,
)


class TrajectoryGuards:
    """Safety checks applied before dispatching cuRobo trajectories."""

    def __init__(self, logger, joint_names):
        self.logger = logger
        self.joint_names = joint_names

    def in_operational_limits(self, traj_rad, label):
        traj_deg = np.rad2deg(traj_rad)
        for joint_idx, (lo, hi) in enumerate(OPERATIONAL_JOINT_LIMITS_DEG):
            vals = traj_deg[:, joint_idx]
            if np.any(vals < lo) or np.any(vals > hi):
                self.logger.warn(
                    f"{label} rejected: J{joint_idx+1} out of [{lo:.0f}deg, {hi:.0f}deg] "
                    f"(range {vals.min():.1f}deg..{vals.max():.1f}deg)")
                return False
        return True

    def has_reasonable_swing(self, traj_rad, start_joints, label,
                             max_joint_delta_deg=None):
        traj_deg = np.rad2deg(traj_rad)
        start_deg = np.rad2deg(start_joints)
        limits = max_joint_delta_deg or MAX_HARVEST_JOINT_DELTA_DEG
        if isinstance(limits, (int, float)):
            limits = [float(limits)] * len(self.joint_names)
        for joint_idx, max_delta in enumerate(limits):
            vals = traj_deg[:, joint_idx]
            if joint_idx in WRAP_EQUIVALENT_JOINT_IDX:
                delta_vals = np.abs(vals - float(vals[0]))
            else:
                delta_vals = np.abs(vals - start_deg[joint_idx])
            delta = float(np.max(delta_vals))
            if delta > max_delta:
                end_deg = traj_deg[-1, joint_idx]
                self.logger.warn(
                    f"{label} rejected: J{joint_idx+1} swing {delta:.1f}deg "
                    f"> {max_delta:.1f}deg "
                    f"(start={start_deg[joint_idx]:.1f}deg -> end={end_deg:.1f}deg)")
                return False
        return True

    def normalize_equivalents(self, traj_rad, label, robot_start_joints_rad=None):
        traj_deg = np.rad2deg(traj_rad).astype(float)
        robot_start_deg = (
            np.rad2deg(robot_start_joints_rad).tolist()
            if robot_start_joints_rad is not None
            else None
        )
        rewritten = []
        for joint_idx in WRAP_EQUIVALENT_JOINT_IDX:
            lo, hi = OPERATIONAL_JOINT_LIMITS_DEG[joint_idx]
            original = traj_deg[:, joint_idx].copy()
            prev = float(robot_start_deg[joint_idx]) if robot_start_deg is not None else None
            for row_idx, value in enumerate(original):
                candidates = [value + 360.0 * k for k in range(-2, 3)]
                valid = [c for c in candidates if lo <= c <= hi]
                if not valid:
                    continue
                reference = prev if prev is not None else value
                best = min(valid, key=lambda c: abs(c - reference))
                traj_deg[row_idx, joint_idx] = best
                prev = best
            if np.max(np.abs(traj_deg[:, joint_idx] - original)) > 1e-6:
                rewritten.append(
                    f"J{joint_idx+1} "
                    f"{float(np.min(original)):.1f}~{float(np.max(original)):.1f}"
                    f" -> {float(np.min(traj_deg[:, joint_idx])):.1f}~"
                    f"{float(np.max(traj_deg[:, joint_idx])):.1f}"
                )
        if rewritten:
            self.logger.info(f"{label} joint equivalent rewrite: " + "; ".join(rewritten))
        return np.deg2rad(traj_deg)

    def nearest_equivalent_joints(self, base_joints_deg, current_joints_rad):
        """Choose wrap-equivalent joints closest to the current robot joints."""
        if current_joints_rad is None:
            return base_joints_deg
        current_deg = np.rad2deg(current_joints_rad)
        joints = list(base_joints_deg)
        for joint_idx in WRAP_EQUIVALENT_JOINT_IDX:
            lo, hi = OPERATIONAL_JOINT_LIMITS_DEG[joint_idx]
            candidates = [joints[joint_idx] + 360.0 * k for k in range(-2, 3)]
            valid = [c for c in candidates if lo <= c <= hi]
            if valid:
                joints[joint_idx] = min(
                    valid, key=lambda c: abs(c - current_deg[joint_idx]))
        return joints

    def has_no_spline_jumps(self, traj_rad, label, max_jump_deg=270.0):
        traj_deg = np.rad2deg(traj_rad)
        for joint_idx in WRAP_EQUIVALENT_JOINT_IDX:
            diffs = np.abs(np.diff(traj_deg[:, joint_idx]))
            if len(diffs) == 0:
                continue
            max_diff = float(np.max(diffs))
            if max_diff > max_jump_deg:
                bad_idx = int(np.argmax(diffs))
                self.logger.warn(
                    f"{label} rejected: J{joint_idx+1} spline jump "
                    f"{max_diff:.1f}deg > {max_jump_deg:.1f}deg at waypoint {bad_idx} "
                    "(limit boundary crossing - normalize discontinuity)")
                return False
        return True
