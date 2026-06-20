#!/usr/bin/env python3
"""Row2 tray-place Cartesian line-check policy."""


def row2_line_check_result(phase: str,
                           slot_index: int,
                           max_deviation_mm: float,
                           limit_mm: float,
                           max_deviation_waypoint: int):
    """Return normalized row2 line-check information."""
    max_deviation_mm = float(max_deviation_mm)
    limit_mm = float(limit_mm)
    return {
        "phase": phase,
        "slot_index": int(slot_index),
        "max_deviation_mm": max_deviation_mm,
        "limit_mm": limit_mm,
        "max_deviation_waypoint": int(max_deviation_waypoint),
        "ok": max_deviation_mm <= limit_mm,
    }
