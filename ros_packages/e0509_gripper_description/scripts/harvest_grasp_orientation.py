#!/usr/bin/env python3
"""Per-target grasp orientation helpers.

The planner's wall approach is intentionally conservative: TOOL Z should keep
pointing into the whiteboard. A published stem quaternion from perception is
therefore converted into a roll-only correction around that wall-normal
approach, instead of being used as the full end-effector orientation.
"""

from dataclasses import dataclass

import numpy as np

from harvest_math import (
    normalized_vec,
    quat_from_axis_angle,
    quat_multiply_wxyz,
    quat_normalize_wxyz,
    quat_rotate_vec,
)


@dataclass
class PublishedRollCandidate:
    accepted: bool
    reason: str
    candidate_quat_wxyz: list | None = None
    roll_deg: float | None = None
    raw_roll_deg: float | None = None
    stem_dir_base: list | None = None
    wall_approach_dir: list | None = None
    approach_error_deg: float | None = None


def published_roll_grasp_candidate(
    input_quat_wxyz,
    wall_quat_wxyz,
    *,
    align_axis="x",
    max_abs_roll_deg=75.0,
):
    """Build a wall-normal, stem-roll-aligned grasp candidate.

    Perception publishes a quaternion whose TOOL Z follows the local stem
    direction. The gripper opening is axial, so +stem and -stem are equivalent;
    roll is folded by 180 degrees to pick the smallest physical correction.
    """
    q_in = quat_normalize_wxyz(input_quat_wxyz)
    if q_in is None:
        return PublishedRollCandidate(False, "invalid_input_quaternion")

    wall_q = quat_normalize_wxyz(wall_quat_wxyz)
    wall_approach = normalized_vec(quat_rotate_vec(wall_q, [0.0, 0.0, 1.0]))
    stem_dir = normalized_vec(quat_rotate_vec(q_in, [0.0, 0.0, 1.0]))
    if wall_approach is None or stem_dir is None:
        return PublishedRollCandidate(False, "invalid_rotated_axis")

    align_axis_local = [1.0, 0.0, 0.0] if align_axis == "x" else [0.0, 1.0, 0.0]
    ref_axis = np.array(quat_rotate_vec(wall_q, align_axis_local), dtype=float)
    ref_axis = ref_axis - np.dot(ref_axis, wall_approach) * wall_approach
    stem_axis = stem_dir - np.dot(stem_dir, wall_approach) * wall_approach
    ref_axis = normalized_vec(ref_axis)
    stem_axis = normalized_vec(stem_axis)
    if ref_axis is None or stem_axis is None:
        return PublishedRollCandidate(
            False,
            "stem_projection_too_small",
            stem_dir_base=stem_dir.tolist(),
            wall_approach_dir=wall_approach.tolist(),
        )

    sin_v = float(np.dot(wall_approach, np.cross(ref_axis, stem_axis)))
    cos_v = float(np.dot(ref_axis, stem_axis))
    raw_roll_deg = float(np.degrees(np.arctan2(sin_v, cos_v)))
    roll_deg = raw_roll_deg
    if roll_deg > 90.0:
        roll_deg -= 180.0
    elif roll_deg < -90.0:
        roll_deg += 180.0

    if abs(roll_deg) > max_abs_roll_deg:
        return PublishedRollCandidate(
            False,
            "roll_exceeds_limit",
            roll_deg=roll_deg,
            raw_roll_deg=raw_roll_deg,
            stem_dir_base=stem_dir.tolist(),
            wall_approach_dir=wall_approach.tolist(),
        )

    q_roll = quat_from_axis_angle(wall_approach, np.deg2rad(roll_deg))
    q_candidate = quat_normalize_wxyz(quat_multiply_wxyz(q_roll, wall_q))
    if q_candidate is None:
        return PublishedRollCandidate(
            False,
            "candidate_quaternion_invalid",
            roll_deg=roll_deg,
            raw_roll_deg=raw_roll_deg,
        )

    candidate_approach = normalized_vec(quat_rotate_vec(q_candidate, [0.0, 0.0, 1.0]))
    approach_error_deg = 0.0
    if candidate_approach is not None:
        approach_error_deg = float(np.degrees(np.arccos(np.clip(
            np.dot(candidate_approach, wall_approach), -1.0, 1.0))))

    return PublishedRollCandidate(
        True,
        "ok",
        candidate_quat_wxyz=q_candidate,
        roll_deg=roll_deg,
        raw_roll_deg=raw_roll_deg,
        stem_dir_base=stem_dir.tolist(),
        wall_approach_dir=wall_approach.tolist(),
        approach_error_deg=approach_error_deg,
    )
