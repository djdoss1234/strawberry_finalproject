#!/usr/bin/env python3
"""Small cuRobo kinematics helpers for FK and Cartesian path diagnostics."""

import numpy as np
import torch


class CuroboKinematicsAdapter:
    """Read-only FK helpers around `motion_gen.kinematics`."""

    def __init__(self, motion_gen, device: str = "cuda:0"):
        self.motion_gen = motion_gen
        self.device = device

    def ee_pose(self, joints_rad):
        """Return `(position_m, quat_wxyz)` for cuRobo's configured ee_link."""
        q = torch.tensor([joints_rad], device=self.device, dtype=torch.float32)
        state = self.motion_gen.kinematics.get_state(q)
        return (
            state.ee_position[0].detach().cpu().numpy().tolist(),
            state.ee_quaternion[0].detach().cpu().numpy().tolist(),
        )

    def trajectory_line_deviation_mm(self, traj_rad, start_pos_m, end_pos_m):
        """Return maximum FK trajectory deviation from a Cartesian line segment."""
        q = torch.tensor(traj_rad, device=self.device, dtype=torch.float32)
        state = self.motion_gen.kinematics.get_state(q)
        points = state.ee_position.detach().cpu().numpy()
        start = np.array(start_pos_m, dtype=float)
        end = np.array(end_pos_m, dtype=float)
        line = end - start
        line_norm_sq = float(np.dot(line, line))
        if line_norm_sq < 1e-12:
            return float("inf"), -1
        fractions = np.clip(((points - start) @ line) / line_norm_sq, 0.0, 1.0)
        projected = start + fractions[:, None] * line
        deviations_mm = np.linalg.norm(points - projected, axis=1) * 1000.0
        max_index = int(np.argmax(deviations_mm))
        return float(deviations_mm[max_index]), max_index
