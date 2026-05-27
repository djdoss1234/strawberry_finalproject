"""Tests for the fail-closed workspace scan motion gate."""

import math

from strawberry_motion.execution.scan_safety import (
    joints_within_tolerance_deg,
    motion_start_allowed,
    single_cell_request_allowed,
)


def test_preview_mode_never_allows_motion():
    allowed, reason = motion_start_allowed(
        execute_motion=False, candidate_authorized=True, has_joint_state=True
    )
    assert allowed is False
    assert "execute_motion" in reason


def test_unauthorized_candidates_block_motion():
    allowed, reason = motion_start_allowed(
        execute_motion=True, candidate_authorized=False, has_joint_state=True
    )
    assert allowed is False
    assert "not authorized" in reason


def test_joint_state_is_required_for_motion():
    allowed, reason = motion_start_allowed(
        execute_motion=True, candidate_authorized=True, has_joint_state=False
    )
    assert allowed is False
    assert "joint state" in reason


def test_all_start_gates_must_be_open():
    allowed, _ = motion_start_allowed(
        execute_motion=True, candidate_authorized=True, has_joint_state=True
    )
    assert allowed is True


def test_overview_pose_tolerance_check():
    expected = [97.84, -94.40, 65.95]
    actual = [math.radians(v) for v in [98.0, -94.0, 66.2]]
    far = [math.radians(v) for v in [98.0, -91.0, 66.2]]
    assert joints_within_tolerance_deg(actual, expected, 1.0) is True
    assert joints_within_tolerance_deg(far, expected, 1.0) is False


def test_single_cell_validation_requires_nw_or_ne():
    assert single_cell_request_allowed("root/nw", ["root/nw", "root/ne"])[0] is True
    assert single_cell_request_allowed("", ["root/nw", "root/ne"])[0] is False
    assert single_cell_request_allowed("root/se", ["root/nw", "root/ne"])[0] is False
