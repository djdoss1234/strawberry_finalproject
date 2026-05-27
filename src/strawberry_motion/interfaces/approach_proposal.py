"""
VLA ↔ motion layer interface for approach direction proposals.

The VLA (or rule layer) calls validate_approach_proposal() with an
ApproachProposal.  The function returns a MotionValidationResult WITHOUT
executing any robot motion.  The caller decides whether to proceed.

Execution is only permitted when MotionValidationStatus.VALID is returned
AND the caller explicitly invokes the executor.
"""

from __future__ import annotations

import dataclasses
import enum
from typing import List, Optional, Tuple

import numpy as np


class ApproachDirection(enum.Enum):
    """Semantic directions a VLA or rule layer may propose for a target."""

    FRONT = "FRONT"              # camera z-axis approach (normal to panel)
    LEFT = "LEFT"                # approach from the robot's left
    RIGHT = "RIGHT"              # approach from the robot's right
    UPPER_LEFT = "UPPER_LEFT"    # diagonal: up + left
    UPPER_RIGHT = "UPPER_RIGHT"  # diagonal: up + right
    REOBSERVE = "REOBSERVE"      # re-scan current cell before committing
    SKIP = "SKIP"                # skip this target; move to next cell
    RECOVER_HOME = "RECOVER_HOME"  # return to safe home state


class MotionValidationStatus(enum.Enum):
    """Result of motion-layer validation for a proposed approach."""

    VALID = "VALID"
    IK_FAIL = "IK_FAIL"
    JOINT_LIMIT_REJECT = "JOINT_LIMIT_REJECT"
    COLLISION_RISK = "COLLISION_RISK"
    START_STATE_COLLISION = "START_STATE_COLLISION"
    PATH_MARGIN_REJECT = "PATH_MARGIN_REJECT"
    INVALID_TARGET = "INVALID_TARGET"
    NOT_VALIDATED = "NOT_VALIDATED"   # cuRobo offline check not performed yet
    REQUIRES_ALTERNATIVE = "REQUIRES_ALTERNATIVE"  # no known reachable pose


@dataclasses.dataclass(frozen=True)
class ApproachProposal:
    """Structured proposal from VLA or rule layer to motion layer.

    Fields:
        cell_id       - quadtree cell identifier (e.g. "root/nw")
        direction     - semantic approach direction
        target_pos_m  - optional target position in base frame (m)
        confidence    - optional VLA confidence score [0.0, 1.0]
        annotation    - optional human-readable note from VLA
    """

    cell_id: str
    direction: ApproachDirection
    target_pos_m: Optional[Tuple[float, float, float]] = None
    confidence: Optional[float] = None
    annotation: Optional[str] = None

    def __post_init__(self):
        if self.confidence is not None and not (0.0 <= self.confidence <= 1.0):
            raise ValueError(f"confidence must be in [0, 1]; got {self.confidence}")


@dataclasses.dataclass(frozen=True)
class MotionValidationResult:
    """Validation result returned to VLA or rule layer.

    No robot motion has occurred.  VALID status means the trajectory
    planner found a collision-free path; the caller must explicitly
    trigger execution.

    Fields:
        proposal         - the original proposal that was validated
        status           - outcome of the validation
        validated_pos_m  - position that was actually validated (may differ
                           from proposal if the direction was adjusted)
        joint_solution   - joint angles (deg) of the IK solution, or None
        planner_note     - diagnostic message from the motion layer
    """

    proposal: ApproachProposal
    status: MotionValidationStatus
    validated_pos_m: Optional[Tuple[float, float, float]] = None
    joint_solution: Optional[Tuple[float, ...]] = None
    planner_note: Optional[str] = None

    @property
    def is_executable(self) -> bool:
        return self.status == MotionValidationStatus.VALID


# ── Pre-loaded offline validation table ────────────────────────────────────────
# Populated from scan_pose_candidates.yaml dry-run results (v2, standoff 0.65 m).
# Key: (cell_id, ApproachDirection) -> MotionValidationStatus
# This is a static lookup; runtime cuRobo validation will supersede these.

_OFFLINE_TABLE: dict = {
    ("root/nw", ApproachDirection.FRONT): MotionValidationStatus.VALID,
    ("root/ne", ApproachDirection.FRONT): MotionValidationStatus.VALID,
    ("root/sw", ApproachDirection.FRONT): MotionValidationStatus.REQUIRES_ALTERNATIVE,
    ("root/se", ApproachDirection.FRONT): MotionValidationStatus.VALID,
}

_OFFLINE_NOTES: dict = {
    ("root/sw", ApproachDirection.FRONT): (
        "IK_FAIL at all tested standoffs (0.35–0.921 m) with panel-facing orientation. "
        "z ≈ 0.335 m + x ≈ -0.335 m falls outside E0509 workspace at this orientation. "
        "Consider observing SW from NW scan position or repositioning robot base."
    ),
}


def validate_approach_proposal(
    proposal: ApproachProposal,
    *,
    offline_only: bool = True,
) -> MotionValidationResult:
    """Validate an ApproachProposal without executing robot motion.

    Args:
        proposal:     the VLA or rule-layer proposal to validate
        offline_only: if True, use only the pre-loaded offline table;
                      runtime cuRobo call is not yet implemented.

    Returns:
        MotionValidationResult with status and optional diagnostic info.
        Status VALID does NOT authorise execution; the caller must
        explicitly invoke the robot executor.
    """
    direction = proposal.direction

    # Non-motion actions: pass through without physical validation
    if direction in (ApproachDirection.REOBSERVE, ApproachDirection.SKIP):
        return MotionValidationResult(
            proposal=proposal,
            status=MotionValidationStatus.VALID,
            planner_note=f"Non-motion action {direction.value}; no IK check required.",
        )

    if direction == ApproachDirection.RECOVER_HOME:
        # HOME pose is pre-validated as safe; a real implementation would
        # verify start-state collision before accepting.
        return MotionValidationResult(
            proposal=proposal,
            status=MotionValidationStatus.VALID,
            planner_note=(
                "RECOVER_HOME accepted by motion layer. "
                "Actual path safety must be verified before execution."
            ),
        )

    if offline_only:
        key = (proposal.cell_id, direction)
        status = _OFFLINE_TABLE.get(key, MotionValidationStatus.NOT_VALIDATED)
        note = _OFFLINE_NOTES.get(key)
        if status == MotionValidationStatus.NOT_VALIDATED:
            note = (
                f"No offline record for ({proposal.cell_id}, {direction.value}). "
                "Run cuRobo dry-run to obtain a validation result."
            )
        return MotionValidationResult(
            proposal=proposal,
            status=status,
            planner_note=note,
        )

    # Runtime cuRobo path (not yet implemented; placeholder).
    raise NotImplementedError(
        "Runtime cuRobo validation is not yet implemented. "
        "Use offline_only=True or add a cuRobo backend here."
    )
