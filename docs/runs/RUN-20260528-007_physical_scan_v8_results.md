# RUN-20260528-007 — Physical Scan Run: v8 camera-centered results

## Run context

- Executor: `scan_executor_node.py` (513c4f1) — YAML endpoint_joints_deg via direct MoveJoint
- Poses: `scan_pose_candidates_refit_candidate.yaml` v8, all cells 0.35m camera-centered standoff
- Scan order: NW → NE → SE → SW

## Observed results (photos saved separately — see below)

| # | Cell | Description |
|---|------|-------------|
| 1 | NW (or NE) | Panel visible, multiple strawberries detected. Cross center shifted SW relative to cell center. Too close — FOV too narrow. |
| 2 | NE (or NW) | Panel close-up, strawberries in upper portion. Shifted. |
| 3 | SE | Camera pointing at blank wall. Panel barely visible at very bottom of frame. |
| 4 | SW | Same as above — blank wall, panel at extreme bottom edge. |

## Photo storage

Physical scan photos → `docs/runs/photos/RUN-20260528-007/`  
Naming convention: `<cell_id>_scan.jpg` (e.g. `nw_scan.jpg`, `se_scan.jpg`)

## Root cause analysis

**Two separate problems:**

### 1. Too close (affects all cells)
`cam_dir ≈ [0.064, 0.922, 0.381]` → Y component 0.922 means 0.35m standoff places the
camera only ~0.32m from the panel face. The cell does not fill 1-cell worth of FOV.
Fix: increase standoff to 0.45–0.55m.

### 2. Systematic SW shift + SE/SW pointing at blank wall
The `fk_gripper_rh` custom Python FK used in `search_all_cells_camera_centered.py` does
not perfectly match the actual robot kinematics. The TCP orientation R_tcp computed at
the overview joints (`fk_gripper_rh(OVERVIEW_DEG)`) may differ from the actual TCP
orientation. Since all 4 camera-centered poses use this R_tcp as the fixed orientation:
- cuRobo IK finds joints that put TCP at the planned position/orientation
- But if fk_gripper_rh gives a wrong R_tcp, the planned TCP targets are systematically off
- Lower cells (SE/SW) have the largest joint deltas from overview → largest accumulated FK error

**Alternative cause**: `t_cam_in_rh` calibration error (rotation matrix M in
`_cam_offset_in_rh()`) causes the camera offset to be applied in the wrong direction,
shifting all cam_pos values systematically.

## Decision

Abandon the purely analytical camera-centered approach. Switch to **physical teaching**:
1. Use `joint_jog_control.py` to drive to each cell manually
2. Verify via YOLO viewer that one cell fills the frame with crosshair centered on cell
3. Record joint angles
4. Write final scan pose YAML directly from measured joints

This eliminates all FK / calibration / panel registration errors.

## Status

- [x] Executor fixed: YAML endpoint_joints_deg used for all cells (IK non-determinism eliminated)
- [ ] Scan poses still wrong due to FK/calibration errors in search script
- [ ] Manual teaching required for all 4 cells
- [ ] Photos to be placed in docs/runs/photos/RUN-20260528-007/
