# RUN-20260601-001 — Gripper-Centered Scan Pose Reteach

## Context

The previous scan pose set prioritized camera-centered framing: each cell was
aligned so the camera crosshair and optical center saw the cell center well.
That was good for detection, but it did not guarantee that the gripper approach
axis was positioned conveniently for harvesting.

On 2026-06-01, the four scan poses were retaught so each cell is framed with
gripper access in mind. The practical criterion was:

- the target cell remains visible enough for detection,
- the gripper fingers are horizontally aligned with the cell working area,
- the robot has a reachable approach posture for subsequent grasp motion.

This is a planning/task-frame correction, not a model-training change.

## New Taught Poses

All values are from Doosan DART. Joint angles are degrees. TCP pose is
`x y z rx ry rz` in `base_link`, with translation in millimeters and rotation in
degrees.

| Cell | Joint Pose | TCP Pose |
| --- | --- | --- |
| root/nw | `[144.09, 22.90, -1.00, -238.52, -75.31, 108.68]` | `[-225.46, 338.93, 902.31, 88.42, 87.31, -89.88]` |
| root/ne | `[18.91, 25.97, 1.00, 74.58, 78.17, -115.61]` | `[314.90, 279.89, 883.40, 89.90, 86.29, -89.62]` |
| root/se | `[22.71, -4.60, 103.56, 97.69, 68.16, -190.82]` | `[312.61, 302.83, 529.32, 89.90, 86.29, -89.62]` |
| root/sw | `[150.27, -11.92, 109.11, -97.81, 63.33, 10.46]` | `[-247.70, 317.34, 533.88, 87.75, 86.31, -89.49]` |

Screenshot evidence order supplied by the operator:

1. root/nw
2. root/ne
3. root/se
4. root/sw

## Config Update

Updated:

- `config/scan_pose_candidates_refit_candidate.yaml`
  - `version: v12_gripper_centered_manual_teach`
  - `status: gripper_centered_scan_poses_taught_pending_single_cell_ros_validation`
  - `endpoint_joints_deg` and `dart_tcp_pose_base_mm_deg` for all four cells

The installed package config is symlinked to the source file, so this update is
visible to ROS launches without a package rebuild.

## Safety Notes

- The new poses are DART-taught physical poses, but each cell still needs ROS
  single-cell validation before full unattended traversal.
- Some DART joint values cross `+/-180` representation boundaries. Doosan may
  report equivalent wrapped joint branches; execution checks must compare joint
  arrival in a wrap-aware way if needed.
- The temporary seg/pose YOLO models are draft reference models. Use them first
  for live detection/fusion validation, not immediately as an unattended harvest
  trigger.

## Next Validation Steps

1. Run single-cell scan validation for `root/nw`, `root/ne`, `root/se`, `root/sw`.
2. Run seg+pose fusion viewer using the draft `.pt` files.
3. Confirm only matched `ripe` detections produce pick candidates.
4. Test one manually selected target at low speed.
5. Enable full scan-to-pick traversal only after the above passes.
