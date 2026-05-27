"""Pure safety checks shared by the workspace scan executor and tests."""

from __future__ import annotations

from typing import Iterable, Tuple


def motion_start_allowed(
    *,
    execute_motion: bool,
    candidate_authorized: bool,
    has_joint_state: bool,
) -> Tuple[bool, str]:
    """Return whether an explicit scan request may reach motion code."""
    if not execute_motion:
        return False, "execute_motion parameter is false"
    if not candidate_authorized:
        return False, "scan candidates are not authorized for automated motion"
    if not has_joint_state:
        return False, "no current joint state received"
    return True, "explicit scan request accepted"


def joints_within_tolerance_deg(
    actual_rad: Iterable[float],
    expected_deg: Iterable[float],
    tolerance_deg: float,
) -> bool:
    """Check that a measured joint state matches a taught reference pose."""
    actual = list(actual_rad)
    expected = list(expected_deg)
    if len(actual) != len(expected):
        return False
    rad_to_deg = 180.0 / 3.141592653589793
    return all(
        abs(measured * rad_to_deg - taught) <= tolerance_deg
        for measured, taught in zip(actual, expected)
    )
