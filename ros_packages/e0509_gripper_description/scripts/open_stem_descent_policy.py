#!/usr/bin/env python3
"""Open-gripper stem descent distance policy.

After horizontal approach, measured-TCP mode descends with the gripper open so
the stem reaches the KP1 grasp zone before closing. This module keeps that
distance calculation explicit and testable.
"""


def compute_open_stem_descent_m(default_descent_m: float,
                                target_kp1_z_m: float,
                                reached_z_m,
                                extra_below_kp1_m: float):
    """Return `(descent_m, details)` for open-stem descent.

    If the reached TCP Z is known, use the actual overshoot above KP1 plus a
    configured extra-below-KP1 margin. If not, preserve the legacy/default
    descent distance.
    """
    if reached_z_m is None:
        descent_m = float(default_descent_m)
        return descent_m, {
            "mode": "default",
            "target_kp1_z_m": float(target_kp1_z_m),
            "reached_z_m": None,
            "overshoot_above_kp1_m": None,
            "extra_below_kp1_m": float(extra_below_kp1_m),
            "executed_descent_m": descent_m,
        }

    target_kp1_z_m = float(target_kp1_z_m)
    reached_z_m = float(reached_z_m)
    overshoot_above_kp1_m = max(0.0, reached_z_m - target_kp1_z_m)
    descent_m = overshoot_above_kp1_m + float(extra_below_kp1_m)
    return descent_m, {
        "mode": "dynamic",
        "target_kp1_z_m": target_kp1_z_m,
        "reached_z_m": reached_z_m,
        "overshoot_above_kp1_m": overshoot_above_kp1_m,
        "extra_below_kp1_m": float(extra_below_kp1_m),
        "executed_descent_m": descent_m,
    }
