#!/usr/bin/env python3
"""Retreat step calculation after final approach/grasp.

The planner executes the returned steps; this module only computes the exact
reverse path. Keeping it pure makes the measured-TCP two-stage retreat easier
to audit after the J2 over-extension incident.
"""

from dataclasses import dataclass
from typing import Any, Dict, List

import numpy as np


@dataclass(frozen=True)
class ApproachDistanceResult:
    target_plane_dist_m: float
    uncapped_distance_m: float
    final_distance_m: float


@dataclass(frozen=True)
class ToolFinishDirection:
    use_base_line: bool
    is_published_roll: bool
    direction: np.ndarray


def tool_finish_base_direction(grasp_variant, approach_dir,
                               tilt_threshold: float = 1e-3):
    """Return whether final tool-finish should use horizontal BASE motion."""
    is_published_roll = (
        grasp_variant is not None
        and grasp_variant[0] == "published_roll"
    )
    approach_dir = np.array(approach_dir, dtype=float)
    use_base_line = (
        is_published_roll
        or abs(float(approach_dir[2])) > float(tilt_threshold)
    )
    if not use_base_line:
        return ToolFinishDirection(False, is_published_roll, approach_dir)

    horiz_dir = approach_dir.copy()
    horiz_dir[2] = 0.0
    horiz_norm = float(np.linalg.norm(horiz_dir))
    if horiz_norm > 1e-6:
        horiz_dir = horiz_dir / horiz_norm
    else:
        horiz_dir = approach_dir
    return ToolFinishDirection(True, is_published_roll, horiz_dir)


def measured_tcp_approach_distance(raw_y_m: float,
                                   straw,
                                   used_pre_ee_pos,
                                   used_approach_dir,
                                   max_approach_m: float,
                                   y_detection_bias_m: float,
                                   pre_approach_offset_m: float,
                                   final_standoff_m: float,
                                   wall_surface_y_m: float,
                                   ceiling_m: float = 0.260):
    """Compute measured-TCP final approach distance before optional extras."""
    baseline_approach = float(pre_approach_offset_m) - float(final_standoff_m)
    pre_approach_y_m = float(wall_surface_y_m) - float(pre_approach_offset_m)
    adaptive_dist = (float(raw_y_m) - float(y_detection_bias_m)) - pre_approach_y_m
    target_plane_dist = float(np.dot(
        np.array(straw, dtype=float) - np.array(used_pre_ee_pos, dtype=float),
        np.array(used_approach_dir, dtype=float),
    ))
    uncapped_distance = max(baseline_approach, min(adaptive_dist, float(ceiling_m)))
    final_distance = max(0.0, min(uncapped_distance, float(max_approach_m)))
    return ApproachDistanceResult(
        target_plane_dist_m=target_plane_dist,
        uncapped_distance_m=uncapped_distance,
        final_distance_m=final_distance,
    )


def build_straight_retreat_steps(measured_tcp_model: bool,
                                 retreat_distance_m: float,
                                 used_approach_dir,
                                 tool_finish_executed_m: float,
                                 tool_finish_executed_dir,
                                 base_label: str,
                                 tool_label: str) -> List[Dict[str, Any]]:
    """Return ordered retreat steps without executing robot motion.

    For measured-TCP, a horizontal/tool-finish leg may have been executed in a
    direction different from the main approach direction. That leg must be
    undone first, then the remaining approach distance is reversed along the
    selected approach direction. For the legacy model, the retreat is a single
    TOOL -Z MoveLine as before.
    """
    retreat_distance_m = float(retreat_distance_m)
    if retreat_distance_m <= 0.0:
        return []

    if not measured_tcp_model:
        return [{
            "frame": "tool",
            "label": tool_label,
            "distance_m": -retreat_distance_m,
        }]

    steps = []
    if tool_finish_executed_m > 0.0 and tool_finish_executed_dir is not None:
        undo_label_base = (
            base_label[:-5] if base_label.endswith("_BASE") else base_label)
        steps.append({
            "frame": "base",
            "label": f"{undo_label_base}_TOOL_FINISH_UNDO",
            "delta_m": -float(tool_finish_executed_m) * np.array(
                tool_finish_executed_dir, dtype=float),
        })

    if retreat_distance_m > 0.0:
        steps.append({
            "frame": "base",
            "label": base_label,
            "delta_m": -retreat_distance_m * np.array(
                used_approach_dir, dtype=float),
        })
    return steps
