#!/usr/bin/env python3
"""Task-level place status handling policy.

The place executor returns compact statuses such as success/preview_hold/failed.
This module translates those statuses into the next state-machine action without
performing any robot I/O.
"""


def classify_place_outcome(place_status: str,
                           use_taught_slot0_place_reference: bool,
                           hold_after_taught_slot0_place: bool,
                           marker_place_slot_idx: int):
    """Return a normalized outcome dict for `_pick()` place handling."""
    if place_status == "success":
        if use_taught_slot0_place_reference and hold_after_taught_slot0_place:
            completed_slot_index = marker_place_slot_idx - 1
            return {
                "action": "hold",
                "result_code": "TAUGHT_TRAY_PLACE_COMPLETE_HOLD",
                "hold_reason": "taught_tray_place_complete",
                "completed_slot_index": completed_slot_index,
            }
        return {"action": "continue"}

    if place_status == "tray_complete":
        return {
            "action": "hold",
            "result_code": "TAUGHT_TRAY_FULL",
            "hold_reason": "taught_tray_full",
        }

    if place_status == "skip":
        return {
            "action": "skip",
            "reason": "tray_unavailable",
        }

    result_code = (
        "MARKER_PLACE_PREVIEW_HOLD"
        if place_status == "preview_hold"
        else "MARKER_PLACE_FAILED"
    )
    return {
        "action": "hold",
        "result_code": result_code,
        "hold_reason": f"marker_place_{place_status}",
    }
