#!/usr/bin/env python3
"""Pick target preparation and guard policy."""

import numpy as np

from harvest_motion_params import (
    CRANE_Z_OFFSET_M,
    DIRECT_GRASP_TARGET_X_RANGE_M,
    MEASURED_TCP_TARGET_Z_MAX_M,
    NW_HIGH_TARGET_Y_PLANE_RELAX_M,
    WALL_SURFACE_Y_M,
)


def prepare_pick_target(position,
                        measured_tcp_model: bool,
                        pick_target_x_bias_m: float,
                        pick_target_z_bias_m: float,
                        nw_high_target_z_threshold_m: float,
                        nw_high_target_crane_z_offset_m: float):
    """Prepare raw/grasp target and static guard decisions for a pick pose."""
    detection_raw_y = float(position.y)
    raw_y = detection_raw_y
    wall_y_clamped = raw_y > WALL_SURFACE_Y_M
    if wall_y_clamped:
        raw_y = WALL_SURFACE_Y_M

    raw_straw = np.array([position.x, raw_y, max(position.z, 0.05)])
    straw = raw_straw + np.array([
        float(pick_target_x_bias_m),
        0.0,
        float(pick_target_z_bias_m),
    ])
    straw[2] = max(straw[2], 0.05)

    is_nw_high_target = (
        bool(measured_tcp_model)
        and float(raw_straw[2]) >= float(nw_high_target_z_threshold_m)
    )
    crane_z_offset_m = (
        float(nw_high_target_crane_z_offset_m)
        if is_nw_high_target else CRANE_Z_OFFSET_M
    )

    y_relax_applied = False
    y_relax_before_m = float(straw[1])
    if (
        is_nw_high_target
        and wall_y_clamped
        and NW_HIGH_TARGET_Y_PLANE_RELAX_M > 0.0
    ):
        straw[1] += NW_HIGH_TARGET_Y_PLANE_RELAX_M
        y_relax_applied = True

    x_min, x_max = DIRECT_GRASP_TARGET_X_RANGE_M
    x_guard_ok = x_min <= float(raw_straw[0]) <= x_max
    z_guard_ok = not (
        bool(measured_tcp_model)
        and float(raw_straw[2]) > MEASURED_TCP_TARGET_Z_MAX_M
    )

    return {
        "detection_raw_y": detection_raw_y,
        "raw_y": raw_y,
        "wall_y_clamped": wall_y_clamped,
        "raw_straw": raw_straw,
        "straw": straw,
        "is_nw_high_target": is_nw_high_target,
        "crane_z_offset_m": crane_z_offset_m,
        "y_relax_applied": y_relax_applied,
        "y_relax_before_m": y_relax_before_m,
        "x_guard_ok": x_guard_ok,
        "x_range_m": (x_min, x_max),
        "z_guard_ok": z_guard_ok,
        "z_max_m": MEASURED_TCP_TARGET_Z_MAX_M,
    }
