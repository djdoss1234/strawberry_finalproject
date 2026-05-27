import numpy as np

from strawberry_motion.registration.panel_registration_validator import (
    DEFAULT_LANDMARKS_PANEL_M,
    evaluate_landmarks,
    predicted_base_point,
)


def test_predicts_panel_point_in_base_frame():
    transform = np.eye(4)
    transform[:3, 3] = [1.0, 2.0, 3.0]
    np.testing.assert_allclose(
        predicted_base_point(transform, [0.5, -0.5, 0.0]),
        [1.5, 1.5, 3.0],
    )


def test_five_precise_landmarks_pass_measurement_only():
    transform = np.eye(4)
    observations = {
        key: {"point_panel_m": point, "observed_base_m": point}
        for key, point in DEFAULT_LANDMARKS_PANEL_M.items()
    }
    result = evaluate_landmarks(transform, observations)
    assert result["status"] == "MEASURED_PASS_PENDING_MOTION_MARGIN"
    assert result["max_error_mm"] == 0.0
    assert result["use_for_automated_motion"] is False


def test_large_or_incomplete_measurement_requires_recapture():
    result = evaluate_landmarks(
        np.eye(4),
        {"origin_crossing": {"point_panel_m": [0.0, 0.0, 0.0],
                             "observed_base_m": [0.020, 0.0, 0.0]}},
    )
    assert result["status"] == "MEASUREMENT_INSUFFICIENT_OR_REQUIRES_RECAPTURE"
    assert result["max_error_mm"] == 20.0
