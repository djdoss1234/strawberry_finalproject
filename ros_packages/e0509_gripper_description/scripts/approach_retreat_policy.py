#!/usr/bin/env python3
"""Retreat step calculation after final approach/grasp.

The planner executes the returned steps; this module only computes the exact
reverse path. Keeping it pure makes the measured-TCP two-stage retreat easier
to audit after the J2 over-extension incident.
"""

from typing import Any, Dict, List

import numpy as np


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
