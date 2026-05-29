# RUN-20260529-002: Manual Scan Pose Teaching After Workspace Reset

## Purpose

After physically realigning the panel so the paper center is closer to the
robot `base_link` centerline, all four scan poses are being retaught manually
with Doosan DART. This run records the remaining `root/ne`, `root/se`, and
`root/sw` poses.

Automated execution is still locked. These poses are configuration candidates
until low-speed single-cell execution validates actual joint arrival and camera
view.

## Start Reference

Overview pose:

```text
joints_deg = [87.98, -94.92, 129.89, 175.94, -31.34, 93.42]
task_pose  = x=6.35mm y=-0.25mm z=547.58mm rx=85.67 ry=66.27 rz=-89.12
```

Previously recorded NW pose:

```text
joints_deg = [139.76, 75.04, -79.09, 128.00, -69.04, 87.75]
task_pose  = x=-245.34mm y=373.90mm z=741.39mm rx=86.46 ry=66.60 rz=-88.80
```

## Newly Taught Poses

### root/ne

```text
joints_deg = [34.82, 77.37, -81.88, 229.61, -67.97, 95.20]
task_pose  = x=285.75mm y=347.14mm z=730.58mm rx=85.94 ry=65.09 rz=-88.59
```

Evidence:

- `docs/runs/photos/RUN-20260529-002/ne_detection.png`
- `docs/runs/photos/RUN-20260529-002/ne_landmark.png`

### root/se

```text
joints_deg = [-145.15, -16.58, -125.59, 135.27, -90.14, -60.53]
task_pose  = x=284.91mm y=346.35mm z=342.92mm rx=86.37 ry=64.04 rz=-89.22
```

Evidence:

- `docs/runs/photos/RUN-20260529-002/se_detection.png`
- `docs/runs/photos/RUN-20260529-002/se_landmark.png`

### root/sw

```text
joints_deg = [-39.25, -15.20, -125.63, 227.28, -89.79, -119.01]
task_pose  = x=-248.59mm y=366.73mm z=348.70mm rx=86.47 ry=64.81 rz=-88.15
```

Evidence:

- `docs/runs/photos/RUN-20260529-002/sw_detection.png`
- `docs/runs/photos/RUN-20260529-002/sw_landmark.png`

## Configuration Changes

- Updated `config/scan_pose_candidates_refit_candidate.yaml`.
- Updated `endpoint_joints_deg` for `root/ne`, `root/se`, and `root/sw`.
- Updated TCP translation entries to the DART task pose positions in meters.
- Kept global `use_for_automated_motion=false`.

## Remaining Validation

Before enabling automated traversal:

1. Run each cell as a low-speed single-cell execution.
2. Confirm actual joint arrival from `/dsr01/joint_states`.
3. Confirm camera view at the scan pose.
4. Return to overview after each cell.
5. Only then consider enabling automated traversal.

