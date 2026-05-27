"""Evaluate measured cultivation-panel landmarks against a registration pose."""

from __future__ import annotations

from typing import Mapping

import numpy as np


OUTER_TAPE_LANDMARKS_PANEL_M = {
    "origin_crossing": [0.0, 0.0, 0.0],
    "outer_nw": [-0.545, 0.395, 0.0],
    "outer_ne": [0.555, 0.395, 0.0],
    "outer_sw": [-0.545, -0.405, 0.0],
    "outer_se": [0.555, -0.405, 0.0],
}

# Measured points are clicked on white paper approximately one 20 mm tape width
# inside the outer black tape boundary. Refine these coordinates if the exact
# paper-side inset is measured later.
DEFAULT_LANDMARKS_PANEL_M = {
    "origin_crossing": [0.0, 0.0, 0.0],
    "paper_inner_nw": [-0.525, 0.375, 0.0],
    "paper_inner_ne": [0.535, 0.375, 0.0],
    "paper_inner_sw": [-0.525, -0.385, 0.0],
    "paper_inner_se": [0.535, -0.385, 0.0],
}


def predicted_base_point(transform_matrix, point_panel_m):
    transform = np.asarray(transform_matrix, dtype=float)
    point = np.asarray([*point_panel_m, 1.0], dtype=float)
    return (transform @ point)[:3]


def fit_panel_transform(observations: Mapping[str, Mapping]) -> np.ndarray:
    """Fit a rigid panel-to-base transform from matched landmark observations."""
    source = np.asarray([item["point_panel_m"] for item in observations.values()], dtype=float)
    target = np.asarray([item["observed_base_m"] for item in observations.values()], dtype=float)
    if len(source) < 3:
        raise ValueError("At least three landmark observations are required for fitting.")
    source_center = source.mean(axis=0)
    target_center = target.mean(axis=0)
    left, _, right_t = np.linalg.svd(
        (source - source_center).T @ (target - target_center)
    )
    rotation = right_t.T @ left.T
    if np.linalg.det(rotation) < 0:
        right_t[-1] *= -1
        rotation = right_t.T @ left.T
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = target_center - rotation @ source_center
    return transform


def evaluate_landmarks(transform_matrix, observations: Mapping[str, Mapping]) -> dict:
    """Calculate per-landmark and aggregate registration error in millimeters."""
    inverse_transform = np.linalg.inv(np.asarray(transform_matrix, dtype=float))
    results = {}
    errors_mm = []
    plane_offsets_mm = []
    for landmark_id, observation in observations.items():
        expected = predicted_base_point(
            transform_matrix, observation["point_panel_m"]
        )
        measured = np.asarray(observation["observed_base_m"], dtype=float)
        vector_mm = (measured - expected) * 1000.0
        norm_mm = float(np.linalg.norm(vector_mm))
        measured_panel = (
            inverse_transform @ np.asarray([*measured.tolist(), 1.0], dtype=float)
        )[:3]
        plane_offset_mm = float(measured_panel[2] * 1000.0)
        results[landmark_id] = {
            "expected_base_m": [round(value, 6) for value in expected.tolist()],
            "observed_base_m": [round(value, 6) for value in measured.tolist()],
            "error_vector_mm": [round(value, 3) for value in vector_mm.tolist()],
            "error_norm_mm": round(norm_mm, 3),
            "plane_offset_mm": round(plane_offset_mm, 3),
        }
        errors_mm.append(norm_mm)
        plane_offsets_mm.append(abs(plane_offset_mm))
    if not errors_mm:
        raise ValueError("At least one panel landmark observation is required.")
    rms_mm = float(np.sqrt(np.mean(np.square(errors_mm))))
    max_mm = float(np.max(errors_mm))
    max_abs_plane_offset_mm = float(np.max(plane_offsets_mm))
    status = (
        "MEASURED_PASS_PENDING_MOTION_MARGIN"
        if (
            len(errors_mm) == len(DEFAULT_LANDMARKS_PANEL_M)
            and rms_mm <= 10.0
            and max_mm <= 15.0
            and max_abs_plane_offset_mm <= 10.0
        )
        else "MEASUREMENT_INSUFFICIENT_OR_REQUIRES_RECAPTURE"
    )
    return {
        "landmark_count": len(errors_mm),
        "rms_error_mm": round(rms_mm, 3),
        "max_error_mm": round(max_mm, 3),
        "max_abs_plane_offset_mm": round(max_abs_plane_offset_mm, 3),
        "status": status,
        "use_for_automated_motion": False,
        "landmarks": results,
    }
