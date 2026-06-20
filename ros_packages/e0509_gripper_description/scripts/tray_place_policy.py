#!/usr/bin/env python3
"""Tray slot target generation for marker/taught place flows."""

import glob
import json
import os
import time

import numpy as np
from scipy.spatial.transform import Rotation as SciR

from harvest_motion_params import (
    DEFAULT_TRAY_CELLS_GLOB,
    TAUGHT_SLOT0_PLACE_REFERENCE_POSX_MM_DEG,
    TAUGHT_SLOT1_PLACE_REFERENCE_POSX_MM_DEG,
    TAUGHT_SLOT3_PLACE_REFERENCE_POSX_MM_DEG,
)


class TrayPlacePolicy:
    """Compute tray-cell and taught-grid place targets without robot I/O."""

    def __init__(self, node, runtime_log, tray_cells_json: str,
                 marker_place_max_age_sec: float,
                 marker_place_above_clearance_m: float,
                 measured_tcp_model: bool):
        self.node = node
        self.runtime_log = runtime_log
        self.tray_cells_json = tray_cells_json
        self.marker_place_max_age_sec = marker_place_max_age_sec
        self.marker_place_above_clearance_m = marker_place_above_clearance_m
        self.measured_tcp_model = measured_tcp_model

    def _log(self):
        return self.node.get_logger()

    def latest_tray_cells_json(self):
        if self.tray_cells_json:
            return self.tray_cells_json
        files = sorted(
            glob.glob(DEFAULT_TRAY_CELLS_GLOB),
            key=os.path.getmtime,
            reverse=True,
        )
        return files[0] if files else None

    def marker_cell_with_taught_grid_pitch(self, cells, slot_index):
        """Apply taught Slot0/1/3 pitch to marker-localized tray cells."""
        by_index = {int(cell.get("index", idx)): cell for idx, cell in enumerate(cells)}
        if not all(index in by_index for index in (0, 1, 3, slot_index)):
            raise ValueError("marker JSON missing required slot0/1/3/target cells")

        cell0 = by_index[0]
        selected = dict(by_index[slot_index])
        contact0 = self._contact_xyz(cell0)
        contact1 = self._contact_xyz(by_index[1])
        contact3 = self._contact_xyz(by_index[3])
        marker_vertical = contact1 - contact0
        marker_horizontal = contact3 - contact0
        marker_vertical_norm = float(np.linalg.norm(marker_vertical))
        marker_horizontal_norm = float(np.linalg.norm(marker_horizontal))
        if marker_vertical_norm < 1e-6 or marker_horizontal_norm < 1e-6:
            raise ValueError("invalid marker tray grid axis")

        taught_slot0 = np.array(TAUGHT_SLOT0_PLACE_REFERENCE_POSX_MM_DEG[:3])
        taught_vertical_pitch = float(np.linalg.norm(
            np.array(TAUGHT_SLOT1_PLACE_REFERENCE_POSX_MM_DEG[:3]) - taught_slot0))
        taught_horizontal_pitch = float(np.linalg.norm(
            np.array(TAUGHT_SLOT3_PLACE_REFERENCE_POSX_MM_DEG[:3]) - taught_slot0))
        horizontal_idx, vertical_idx = divmod(slot_index, 3)
        calibrated_contact = (
            contact0
            + vertical_idx * taught_vertical_pitch * marker_vertical / marker_vertical_norm
            + horizontal_idx * taught_horizontal_pitch * marker_horizontal / marker_horizontal_norm
        )

        original_contact = self._contact_xyz(selected)
        correction = calibrated_contact - original_contact
        selected["position_contact_mm"] = self._xyz_dict(calibrated_contact)
        if "position_tcp_mm" in selected:
            selected["position_tcp_mm"] = {
                axis: float(selected["position_tcp_mm"][axis]) + float(correction[idx])
                for idx, axis in enumerate(("x", "y", "z"))
            }
        return selected, {
            "marker_vertical_pitch_mm": marker_vertical_norm,
            "marker_horizontal_pitch_mm": marker_horizontal_norm,
            "taught_vertical_pitch_mm": taught_vertical_pitch,
            "taught_horizontal_pitch_mm": taught_horizontal_pitch,
            "position_correction_mm": correction.tolist(),
        }

    def load_marker_place_target(self, slot_index: int):
        path = self.latest_tray_cells_json()
        if not path or not os.path.isfile(path):
            self._log().error("MARKER_PLACE_BLOCKED: tray cells JSON not found")
            return None
        age_sec = time.time() - os.path.getmtime(path)
        if age_sec > self.marker_place_max_age_sec:
            self._log().error(
                f"MARKER_PLACE_BLOCKED: tray localization stale "
                f"age={age_sec:.0f}s > {self.marker_place_max_age_sec:.0f}s")
            return None
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            cells = data.get("cells", [])
            if not cells:
                raise ValueError("no cells")
            slot_index = slot_index % len(cells)
            cell, grid_calibration = self.marker_cell_with_taught_grid_pitch(
                cells, slot_index)
            orient = cell["task_orientation_deg"]
            if self.measured_tcp_model and "position_contact_mm" in cell:
                tcp = self._measured_tcp_from_contact(cell, orient)
                target_source = "measured_grasp_center_from_contact_minus_10mm"
            else:
                tcp = cell["position_tcp_mm"]
                target_source = "legacy_position_tcp_mm"
            release = [
                float(tcp["x"]), float(tcp["y"]), float(tcp["z"]),
                float(orient["rx"]), float(orient["ry"]), float(orient["rz"]),
            ]
            if not (
                -800.0 <= release[0] <= 800.0
                and -800.0 <= release[1] <= 800.0
                and 250.0 <= release[2] <= 1200.0
            ):
                raise ValueError(f"target outside guarded workspace: {release[:3]}")
        except Exception as exc:
            self._log().error(f"MARKER_PLACE_BLOCKED: invalid tray JSON ({exc})")
            return None

        above = list(release)
        above[2] += self.marker_place_above_clearance_m * 1000.0
        gripper_offset = data.get("gripper_offset") or {}
        self.runtime_log.log(
            "marker_place_target_loaded",
            path=path,
            age_sec=age_sec,
            slot_index=cell.get("index"),
            row=cell.get("row"),
            col=cell.get("col"),
            release_posx_mm_deg=release,
            above_posx_mm_deg=above,
            source_standoff_mm=gripper_offset.get("fingertip_standoff_mm"),
            target_source=target_source,
            source_contact_mm=cell.get("position_contact_mm"),
            source_legacy_tcp_mm=cell.get("position_tcp_mm"),
            grid_calibration=grid_calibration,
        )
        self._log().info(
            f"MARKER_TRAY_GRID slot={cell.get('index')} "
            f"marker_pitch(vertical/horizontal)="
            f"{grid_calibration['marker_vertical_pitch_mm']:.1f}/"
            f"{grid_calibration['marker_horizontal_pitch_mm']:.1f}mm -> "
            f"taught_pitch="
            f"{grid_calibration['taught_vertical_pitch_mm']:.1f}/"
            f"{grid_calibration['taught_horizontal_pitch_mm']:.1f}mm "
            f"correction={np.round(grid_calibration['position_correction_mm'], 1).tolist()}mm")
        return {
            "path": path,
            "slot_index": int(cell.get("index", slot_index)),
            "release": release,
            "above": above,
            "target_source": target_source,
            "contact_mm": cell.get("position_contact_mm"),
        }

    def taught_grid_slot_offset_m(self, slot_index: int):
        """Compute BASE offset from measured Slot0/1/3 taught positions."""
        slot0 = np.array(TAUGHT_SLOT0_PLACE_REFERENCE_POSX_MM_DEG[:3], dtype=float)
        slot1 = np.array(TAUGHT_SLOT1_PLACE_REFERENCE_POSX_MM_DEG[:3], dtype=float)
        slot3 = np.array(TAUGHT_SLOT3_PLACE_REFERENCE_POSX_MM_DEG[:3], dtype=float)
        horizontal_idx, vertical_idx = divmod(slot_index, 3)
        offset_mm = horizontal_idx * (slot3 - slot0) + vertical_idx * (slot1 - slot0)
        return (offset_mm / 1000.0).tolist()

    @staticmethod
    def _contact_xyz(cell):
        return np.array([
            float(cell["position_contact_mm"][axis])
            for axis in ("x", "y", "z")
        ])

    @staticmethod
    def _xyz_dict(values):
        return {
            axis: float(values[idx])
            for idx, axis in enumerate(("x", "y", "z"))
        }

    @staticmethod
    def _measured_tcp_from_contact(cell, orient):
        contact = cell["position_contact_mm"]
        rotation = SciR.from_euler(
            "ZYZ",
            [float(orient["rx"]), float(orient["ry"]), float(orient["rz"])],
            degrees=True,
        )
        tool_z = rotation.apply([0.0, 0.0, 1.0])
        return {
            axis: float(contact[axis]) - 10.0 * float(tool_z[idx])
            for idx, axis in enumerate(("x", "y", "z"))
        }
