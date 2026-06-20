# HANDOFF 2026-06-20 — NW high motion regression after depth-fix experiments

## 0. Read this first

Official repo:

```bash
~/doosan_ws/src/strawberry_finalproject
```

Package path:

```bash
~/doosan_ws/src/e0509_gripper_description
```

Commit only to `strawberry_finalproject`.

Do not touch:

```bash
scripts/측정.py
```

Persistent untracked file:

```bash
config/scan_pose_candidates_depth2.yaml
```

Leave it alone unless Minseok explicitly asks.

## 1. Current problem

NW high target motion regressed after a sequence of fixes.

User observation:

- Earlier run had **straight approach and depth roughly correct**.
- Remaining issue then was only: gripper closed loosely because another stem overlapped.
- User asked to lower the approach point slightly, not to redesign the whole approach.
- Subsequent changes caused:
  - side approach,
  - KP1 close point too high,
  - repeated `MoveLine success but joints barely moved`,
  - straight approach failure.

Current latest failed log:

```text
logs/runtime/2026-06-20/curobo_planner_node_20260620T172703-1b8ca736.jsonl
```

Important lines:

```text
NW_HIGH_TARGET_VARIANT_ORDER: +0deg, +180deg, -180deg
MEASURED_TCP_FINAL_PROBE_BEST depth=70mm J3=34.9deg J5pre=86.4deg align=0.0deg variant=('base', [1,0,0], 0.0)
Cartesian plan rejected: J6 spline jump 355.8deg ...
FINAL_APPROACH_PRECOMPUTED_CUROBO depth=70mm
FINAL_APPROACH_TOOL_FINISH TOOL +Z 110.0mm
FINAL_APPROACH_TOOL_FINISH MoveLine reported success but joints barely moved
ABORT: 직선 진입 실패
```

This means the latest code still fell back to:

```text
pre -> cuRobo 70mm spline -> TOOL +Z 110mm
```

instead of reliably doing the intended SW-like:

```text
pre -> TOOL +Z 180mm
```

## 2. Last visually useful state

The useful state was around:

```text
commit f2ec778 fix: bias NW high target plane before planning
log: curobo_planner_node_20260620T160902-deba2eb6.jsonl
```

Observed by user:

```text
"더 깊이 들어갔다. 깊이는 괜찮은데 옆에 다른 줄기가 같이 겹침"
```

At that point:

- target Y relax was active:

```text
NW_HIGH_TARGET_Y_PLANE_RELAX_M = 0.010
```

- depth looked acceptable.
- calculation was somewhat faster.
- remaining issue was **not depth**, but open descent / close geometry around KP1.

## 3. Experimental commits after the useful state

These commits are experimental and should be treated as regression candidates:

```text
6766cef fix: shorten NW high open descent
733f827 fix: lower NW high approach endpoint
1b0755b fix: prefer SW-like NW high approach
c1d2218 fix: choose healthier SW-like NW branch
3b67f35 fix: force horizontal NW high approach
e20b6c0 fix: use SW-style final approach for NW high
e31abae fix: avoid NW high wrist singularity
```

Recommended recovery:

Do not keep stacking fixes on top of the current state. First restore a clean
known-good base.

Option A, safest for 실기 debugging:

```bash
git -C ~/doosan_ws/src/strawberry_finalproject checkout -b debug/nw-return-to-depth-good f2ec778
colcon build --packages-select e0509_gripper_description --allow-overriding e0509_gripper_description
```

Then test the exact known-good depth state again before applying new changes.

Option B, if staying on `main`, revert the experimental commits in reverse order:

```bash
git -C ~/doosan_ws/src/strawberry_finalproject revert e31abae e20b6c0 3b67f35 c1d2218 1b0755b 733f827 6766cef
```

Only do this if Minseok agrees, because it rewrites behavior through revert commits.

## 4. What NOT to do next

Do not:

- keep adding orientation candidates blindly,
- use +5/+10/+15 pitch as a fallback if the user says it visually goes sideways,
- treat "J3 healthy" as success if the physical approach line is wrong,
- keep mixing cuRobo partial final approach with TOOL finish without proving it matches SW motion,
- hardcode more offsets without making them runtime parameters.

The user wants:

```text
SW-like horizontal approach
-> close near KP1
-> not a side approach
-> only slightly lower approach/close point
```

## 5. Concrete next fix after restoring f2ec778

Make the KP1 approach height tunable instead of hardcoded.

Current global:

```python
CRANE_Z_OFFSET_M = 0.030
```

Do not immediately hardcode 15mm. That was too aggressive and got mixed with
other branch-selection changes.

Add a ROS parameter, e.g.

```text
nw_high_target_crane_z_offset_m
```

Recommended default after restore:

```text
0.030  # preserve last visually useful behavior
```

Then test in small steps:

```text
30mm -> 25mm -> 20mm
```

The intended change is:

```text
pre-approach endpoint lowered by the same amount as open descent
open descent distance changed by the same amount
```

So the close point remains geometrically consistent. Do not reduce only descent
without moving pre-approach; that was the first wrong implementation.

Expected log after parameterization:

```text
NW_HIGH_TARGET_KP1_OFFSET: pre-approach and open descent 30mm -> 25mm
OPEN_STEM_DESCENT BASE REL ... -25mm
```

## 6. Bug candidate in latest main

If continuing from latest `main` instead of reverting, inspect this code path:

```python
force_sw_style_tool_line = (
    self._measured_tcp_model
    and is_nw_high_target
    and used_variant_tilt_deg_for_approach <= 1e-6
)
```

This tolerance is probably too strict. `approach_dir[2]` may be tiny but nonzero,
so the SW-style direct TOOL line branch may not activate. That explains why the
latest log still used:

```text
FINAL_APPROACH_PRECOMPUTED_CUROBO depth=70mm
FINAL_APPROACH_TOOL_FINISH 110mm
```

even though `e20b6c0` was supposed to force:

```text
FINAL_APPROACH_SW_STYLE_TOOL_LINE 180mm
```

If not reverting, change the condition to something like:

```python
used_variant_tilt_deg_for_approach <= 1.0
```

or use the existing:

```python
NW_HIGH_TARGET_STOP_ALIGNMENT_DEG
```

But the recommended path is still: restore the useful state first, then reapply
one small parameterized change.

## 7. Test command used during regression

Planner:

```bash
source ~/doosan_ws/install/setup.bash
ros2 run e0509_gripper_description curobo_planner_node.py --ros-args \
  -p measured_tcp_plan_only:=false \
  -p direct_curobo_final_approach_for_measured_tcp:=true \
  -p measured_tcp_max_approach_m:=0.200 \
  -p measured_tcp_tool_line_after_curobo_fallback:=true \
  -p debug_dump_plan_calls:=true
```

Scan:

```bash
source ~/doosan_ws/install/setup.bash
ros2 launch strawberry_motion workspace_scan.launch.py \
  enable_robot_execution:=true \
  target_cell:=root/nw \
  enable_fusion_detection:=true \
  enable_pick_integration:=true \
  collect_then_pick:=true \
  collect_pick_ready_cell:=root/nw \
  max_total_picks:=1 \
  scan_movej_vel_deg_s:=5.0 \
  scan_movej_acc_deg_s2:=10.0 \
  overview_return_vel_deg_s:=5.0 \
  overview_return_acc_deg_s2:=10.0
```

Trigger:

```bash
ros2 service call /strawberry/scan/start std_srvs/srv/Trigger "{}"
```

## 8. Immediate instruction for Claude Code

Start by reading:

```bash
docs/HANDOFF_20260620_NW_REGRESSION_FOR_CLAUDE.md
docs/HANDOFF_20260618_NW_SESSION_HANDOFF_FOR_CODEX.md
```

Then do one of:

1. Create debug branch at `f2ec778`, rebuild, and re-test known-good depth.
2. If Minseok wants mainline only, revert the experimental commits listed above.

After that, implement only:

```text
parameterized NW high KP1 offset: 30mm default, test 25mm/20mm
```

Do not reintroduce side-pitch candidates until the horizontal baseline is stable.
