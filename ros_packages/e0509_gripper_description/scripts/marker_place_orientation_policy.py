#!/usr/bin/env python3
"""Marker-tray place orientation and candidate target generation."""

import numpy as np
from scipy.spatial.transform import Rotation as SciR

from harvest_motion_params import MEASURED_FLANGE_TO_GRASP_CENTER_M


def doosan_zyz_to_wxyz(rx_deg: float, ry_deg: float, rz_deg: float):
    """Doosan ZYZ Euler (deg) -> quaternion [w, x, y, z] for cuRobo."""
    r = SciR.from_euler("ZYZ", [rx_deg, ry_deg, rz_deg], degrees=True)
    xyzw = r.as_quat()
    return [float(xyzw[3]), float(xyzw[0]), float(xyzw[1]), float(xyzw[2])]


def quat_from_tool_z(tool_z, roll_deg=0.0):
    """Build quaternion [w, x, y, z] from desired world TOOL +Z and roll."""
    z_axis = np.array(tool_z, dtype=float)
    z_axis /= np.linalg.norm(z_axis)
    reference = np.array([0.0, 0.0, 1.0])
    if abs(float(np.dot(reference, z_axis))) > 0.95:
        reference = np.array([1.0, 0.0, 0.0])
    x_axis = np.cross(reference, z_axis)
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)
    rotation = SciR.from_matrix(np.column_stack((x_axis, y_axis, z_axis)))
    rotation = rotation * SciR.from_euler("Z", roll_deg, degrees=True)
    xyzw = rotation.as_quat()
    return [float(xyzw[3]), float(xyzw[0]), float(xyzw[1]), float(xyzw[2])]


def marker_place_orientation_candidates(tray_view_quat, target_pos_m):
    """Return place orientation candidates for tray marker flow."""
    candidates = [("tray_view_fk", tray_view_quat)]

    target = np.array(target_pos_m, dtype=float)
    radial_xy = np.array([target[0], target[1], 0.0])
    radial_norm = float(np.linalg.norm(radial_xy))
    if radial_norm > 1e-9:
        radial_xy /= radial_norm
        for down_component in (-0.25, -0.50, -0.75):
            tool_z = radial_xy + np.array([0.0, 0.0, down_component])
            tool_z /= np.linalg.norm(tool_z)
            for roll_deg in (0.0, 90.0, -90.0, 180.0):
                candidates.append((
                    f"inclined_down_{abs(down_component):.2f}_roll_{roll_deg:+.0f}",
                    quat_from_tool_z(tool_z, roll_deg),
                ))

    for yaw_deg in (0.0, 45.0, -45.0, 90.0, -90.0, 180.0):
        rotation = (
            SciR.from_euler("Z", yaw_deg, degrees=True)
            * SciR.from_euler("X", 180.0, degrees=True)
        )
        xyzw = rotation.as_quat()
        candidates.append((
            f"top_down_yaw_{yaw_deg:+.0f}",
            [float(xyzw[3]), float(xyzw[0]), float(xyzw[1]), float(xyzw[2])],
        ))
    return candidates


def unique_clearance_candidates(requested_clearance_m: float):
    candidates = []
    for clearance_m in (requested_clearance_m, 0.070, 0.050, 0.030):
        if clearance_m > 0.0 and not any(
                abs(clearance_m - existing) < 1e-6
                for existing in candidates):
            candidates.append(clearance_m)
    return candidates


def marker_place_candidate_target(target, candidate_quat,
                                  clearance_m: float,
                                  requested_clearance_m: float,
                                  measured_tcp_model: bool):
    """Return release/above positions and diagnostic radii for one candidate."""
    default_above_pos_m = [v / 1000.0 for v in target["above"][:3]]
    candidate_release_pos_m = [v / 1000.0 for v in target["release"][:3]]
    candidate_above_pos_m = list(default_above_pos_m)

    candidate_xyzw = [
        candidate_quat[1], candidate_quat[2],
        candidate_quat[3], candidate_quat[0],
    ]
    candidate_tool_z = SciR.from_quat(candidate_xyzw).apply([0.0, 0.0, 1.0])

    if measured_tcp_model and target.get("contact_mm"):
        contact_m = np.array([
            float(target["contact_mm"][axis]) / 1000.0
            for axis in ("x", "y", "z")
        ])
        candidate_release_pos_m = (contact_m - 0.010 * candidate_tool_z).tolist()
        candidate_above_pos_m = list(candidate_release_pos_m)
        candidate_above_pos_m[2] += clearance_m
    else:
        candidate_above_pos_m[2] += clearance_m - requested_clearance_m

    radial_distance_m = float(np.linalg.norm(candidate_above_pos_m))
    implied_flange_pos_m = (
        np.array(candidate_above_pos_m)
        - MEASURED_FLANGE_TO_GRASP_CENTER_M * candidate_tool_z
    )
    implied_flange_distance_m = float(np.linalg.norm(implied_flange_pos_m))
    return {
        "release_pos_m": candidate_release_pos_m,
        "above_pos_m": candidate_above_pos_m,
        "tcp_radius_m": radial_distance_m,
        "flange_radius_m": implied_flange_distance_m,
    }
