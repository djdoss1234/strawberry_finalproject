"""Unit tests for ApproachProposal / MotionValidationResult interface."""

import pytest

from strawberry_motion.interfaces import (
    ApproachDirection,
    ApproachProposal,
    MotionValidationResult,
    MotionValidationStatus,
    validate_approach_proposal,
)


class TestApproachProposal:
    def test_basic_construction(self):
        p = ApproachProposal(cell_id="root/ne", direction=ApproachDirection.FRONT)
        assert p.cell_id == "root/ne"
        assert p.direction == ApproachDirection.FRONT
        assert p.confidence is None

    def test_with_confidence(self):
        p = ApproachProposal(
            cell_id="root/nw",
            direction=ApproachDirection.LEFT,
            confidence=0.85,
        )
        assert p.confidence == 0.85

    def test_invalid_confidence_raises(self):
        with pytest.raises(ValueError):
            ApproachProposal(
                cell_id="root/nw",
                direction=ApproachDirection.FRONT,
                confidence=1.5,
            )

    def test_all_directions_constructible(self):
        for d in ApproachDirection:
            p = ApproachProposal(cell_id="root/ne", direction=d)
            assert p.direction == d


class TestMotionValidationResult:
    def test_valid_is_executable(self):
        proposal = ApproachProposal(cell_id="root/ne", direction=ApproachDirection.FRONT)
        result = MotionValidationResult(
            proposal=proposal,
            status=MotionValidationStatus.VALID,
        )
        assert result.is_executable is True

    def test_ik_fail_not_executable(self):
        proposal = ApproachProposal(cell_id="root/sw", direction=ApproachDirection.FRONT)
        result = MotionValidationResult(
            proposal=proposal,
            status=MotionValidationStatus.IK_FAIL,
        )
        assert result.is_executable is False

    def test_requires_alternative_not_executable(self):
        proposal = ApproachProposal(cell_id="root/sw", direction=ApproachDirection.FRONT)
        result = MotionValidationResult(
            proposal=proposal,
            status=MotionValidationStatus.REQUIRES_ALTERNATIVE,
        )
        assert result.is_executable is False


class TestValidateApproachProposal:
    """Tests for the offline validation lookup."""

    def test_ne_front_is_valid(self):
        p = ApproachProposal(cell_id="root/ne", direction=ApproachDirection.FRONT)
        result = validate_approach_proposal(p)
        assert result.status == MotionValidationStatus.VALID
        assert result.is_executable is True

    def test_nw_front_is_valid(self):
        p = ApproachProposal(cell_id="root/nw", direction=ApproachDirection.FRONT)
        result = validate_approach_proposal(p)
        assert result.status == MotionValidationStatus.VALID

    def test_se_front_is_valid(self):
        p = ApproachProposal(cell_id="root/se", direction=ApproachDirection.FRONT)
        result = validate_approach_proposal(p)
        assert result.status == MotionValidationStatus.VALID

    def test_sw_front_requires_alternative(self):
        p = ApproachProposal(cell_id="root/sw", direction=ApproachDirection.FRONT)
        result = validate_approach_proposal(p)
        assert result.status == MotionValidationStatus.REQUIRES_ALTERNATIVE
        assert result.is_executable is False
        assert result.planner_note is not None

    def test_sw_front_note_is_informative(self):
        p = ApproachProposal(cell_id="root/sw", direction=ApproachDirection.FRONT)
        result = validate_approach_proposal(p)
        assert "IK_FAIL" in (result.planner_note or "")

    def test_unknown_cell_returns_not_validated(self):
        p = ApproachProposal(cell_id="root/unknown", direction=ApproachDirection.FRONT)
        result = validate_approach_proposal(p)
        assert result.status == MotionValidationStatus.NOT_VALIDATED

    def test_reobserve_is_valid_without_ik_check(self):
        p = ApproachProposal(cell_id="root/sw", direction=ApproachDirection.REOBSERVE)
        result = validate_approach_proposal(p)
        assert result.status == MotionValidationStatus.VALID
        assert "Non-motion" in (result.planner_note or "")

    def test_skip_is_valid_without_ik_check(self):
        p = ApproachProposal(cell_id="root/nw", direction=ApproachDirection.SKIP)
        result = validate_approach_proposal(p)
        assert result.status == MotionValidationStatus.VALID

    def test_recover_home_is_valid(self):
        p = ApproachProposal(cell_id="root/sw", direction=ApproachDirection.RECOVER_HOME)
        result = validate_approach_proposal(p)
        assert result.status == MotionValidationStatus.VALID

    def test_recover_home_note_mentions_safety(self):
        p = ApproachProposal(cell_id="root/sw", direction=ApproachDirection.RECOVER_HOME)
        result = validate_approach_proposal(p)
        assert "safety" in (result.planner_note or "").lower() or "safe" in (result.planner_note or "").lower()

    def test_runtime_not_implemented(self):
        p = ApproachProposal(cell_id="root/ne", direction=ApproachDirection.FRONT)
        with pytest.raises(NotImplementedError):
            validate_approach_proposal(p, offline_only=False)

    def test_result_preserves_proposal(self):
        p = ApproachProposal(
            cell_id="root/ne",
            direction=ApproachDirection.FRONT,
            confidence=0.9,
            annotation="ripe strawberry",
        )
        result = validate_approach_proposal(p)
        assert result.proposal is p
        assert result.proposal.annotation == "ripe strawberry"
