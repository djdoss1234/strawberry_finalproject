# RUN-20260602-002 — RViz/MoveIt Gripper-Frame Visualization Update

## Context

The workspace was physically re-centered so the `base_link` relationship to the
paper center is no longer the old camera-centered setup. The scan poses were
also retaught in v12 so the gripper, not the camera optical center alone, is
parallel/aligned with each cell working area.

Because of that, the RViz preview needed to stop showing old camera-centered
generated markers and instead show the active taught TCP/gripper frame and
camera optical axis.

## Changes

### Active v12 Scan Pose Preview

Updated:

- `src/strawberry_motion/visualization/scan_pose_tcp_preview_node.py`
- `src/strawberry_motion/visualization/workspace_marker_node.py`
- `rviz/workspace_exploration.rviz`

Before:

- loaded old camera-centered/generated preview assumptions
- showed multiple TCP axes, TCP-to-cell lines, coordinate/status labels, and
  extra marker text that made the validation view hard to read

After:

- loads active `config/scan_pose_candidates_refit_candidate.yaml` v12 taught poses
- shows only the markers needed for scan-pose validation:
  - yellow dot: cell center
  - `NW` / `NE` / `SE` / `SW`: cell name below each center
  - green dot: taught TCP position executed by MoveJoint
  - green arrow: gripper/tool forward axis
  - cyan dot/arrow: camera center and camera optical direction from eye-in-hand calibration
  - `base_link` label near the base axes
- removes TCP projection dot, TCP coordinate labels, marker legend text, and stale
  generated camera preview markers

This makes RViz a clean reference view for the current gripper-centered scan
geometry. It is a diagnostic visualization, not a motion authorization signal.

### Panel TF Visual Fit

Updated:

- `config/panel_registration.yaml`
- `config/scan_collision_world.yaml`

A temporary `base_link` lateral-X centering override made the cell centers look
centered globally, but it did not match the four physically taught scan TCP
poses. The active panel transform was therefore re-fit using the v12 taught TCP
positions projected onto the paper plane.

After the fit, the taught TCP projections are within roughly 1-3cm of the four
cell centers in the panel frame. Robot execution still uses the taught
`endpoint_joints_deg`; this registration update is for RViz/offline world
consistency.

### Old Generated Camera Preview Disabled

Updated:

- `config/workspace.yaml`

Changed:

```yaml
scan_pose:
  preview_standoff_m: null
  preview_source: disabled_after_v12_gripper_centered_scan_pose_reteach
```

This prevents `workspace_marker_node` from publishing stale generated camera
standoff markers that no longer represent the active scan poses.

### MoveIt Parallel Launch Option

Updated:

- `launch/workspace_scan.launch.py`
- `launch/workspace_rviz.launch.py`

Added launch arguments:

```text
enable_moveit:=false
moveit_rviz:=false
moveit_environment:=false
```

When `enable_moveit:=true`, the launch includes:

```text
e0509_gripper_moveit_config/launch/demo.launch.py
```

with MoveIt's own RViz disabled by default so the existing workspace RViz remains
the primary visualizer.

## Intended Role Of MoveIt

MoveIt is not replacing the current scan executor yet. Its immediate role is:

- planning scene and robot state visualization,
- trajectory sanity checking,
- future comparison baseline against cuRobo and Doosan native MoveJoint,
- eventual shared scene source with cuRobo collision world.

The current scan traversal remains:

```text
YAML v12 endpoint_joints_deg
 -> scan_executor_node
 -> Doosan MoveJoint
```

with J1/J4/J6 equivalent-branch rewriting at dispatch time.

## Safety Notes

- The approximately 15.8cm 3D-printed gripper attachment means tool collision
  geometry must be reviewed before trusting either cuRobo or MoveIt collision
  clearance.
- RViz marker correctness is visual/diagnostic evidence, not motion
  authorization.
- MoveIt launch is optional and defaults off to avoid changing the already
  validated scan execution path.
