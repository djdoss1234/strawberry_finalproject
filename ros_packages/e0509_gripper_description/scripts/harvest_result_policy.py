#!/usr/bin/env python3
"""Result-code and gate policies for harvest task execution.

These helpers are intentionally pure functions. The planner node still owns
motion execution and logging; this module only answers task-policy questions
such as whether a grasp result may proceed to place.
"""


def allow_place_after_grasp(grasp_result: str,
                            allow_unverified_grasp_place: bool) -> bool:
    """Return whether the current grasp result is allowed to enter place."""
    return (
        grasp_result == "GRASP_CONTACT_DETECTED"
        or (
            grasp_result == "GRASP_UNVERIFIED"
            and allow_unverified_grasp_place
        )
    )


def place_gate_block_reason(grasp_result: str) -> str:
    """Human-readable reason when place is blocked after grasp verification."""
    if grasp_result == "GRASP_EMPTY":
        return "GRASP_EMPTY: jaw fully closed, nothing grabbed"
    return "GRASP_UNVERIFIED: enable explicit override only after visual check"


def pick_sequence_result_code(grasp_result: str) -> str:
    """Normalize the final pick sequence code after detach/retreat."""
    if grasp_result == "GRASP_CONTACT_DETECTED":
        return "DETACH_SUCCESS_UNVERIFIED"
    return grasp_result
