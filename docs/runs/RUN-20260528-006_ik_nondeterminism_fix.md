# RUN-20260528-006 — cuRobo IK Non-Determinism Root Cause & Fix

## Summary

Today's 4-cell scan traversal produced wrong camera views despite
geometrically correct TCP targets. Root cause: cuRobo picks a different
IK solution at runtime than the one found during the offline search,
sending the robot to the wrong joint configuration.

## What Happened

### Observed failures (NW/NE scan, v8 camera-centered poses)

| Cell | Search endpoint (J1) | Runtime endpoint (J1) | Result |
|------|---------------------|-----------------------|--------|
| NW   | 129.6°              | −50.4°                | Wrong camera area, J4=137.9° near limit |
| NE   | 36.1°               | ~216°                 | J4=232.1° → traj rejected, 5/5 retries fail |
| SE   | 35.8°               | −248.9° J6 wind-up    | EXEC_FAIL, aborted |
| SW   | 128.9°              | (was already using direct MoveJoint) | OK |

Camera photos (NW/NE after v8 pose switch): field of view was worse than
before — camera pointed at wrong region because robot went to mirrored
IK solution.

### Root cause

`_scan_sequence` called cuRobo `_plan()` at runtime for NW/NE/SE.
cuRobo uses stochastic seeds for IK — same TCP target, different run,
different joint solution. The 5-retry loop didn't help because cuRobo
consistently converged to the same wrong solution in each session.

Example (NW, v8 run):
```
root/nw plan endpoint_deg=[-50.4 -94.2 100.2 137.9 72.0 -93.5]  attempt=1
root/nw J1 swing 148° — using reduced spline vel 60°/s
```
vs. search result: `[129.6, -6.0, 100.2, 56.0, -50.2, -152.6]` (J1 swing 32°)

J1 flipped sign: cuRobo found the elbow-up/elbow-down mirror solution.
Camera ended up pointing ~200° off from the intended cell center.

## Fix (committed in this session)

**`scan_executor_node.py`**: Removed cuRobo `_plan()` from `_scan_sequence`.
All 4 cells now use `_movej(endpoint_joints_deg, ...)` directly, reading
`endpoint_joints_deg` from the YAML candidates file.

```
Before: _scan_sequence → _plan() [cuRobo, non-deterministic] → endpoint_rad → _movej/spline
After:  _scan_sequence → YAML endpoint_joints_deg → _movej (deterministic)
```

- `_plan()`, `_exec_spline()`, `_spline_vel_for_j1_swing()` remain defined
  (available for pick planning) but are no longer called during scanning.
- `_SCAN_MOVEJ_VEL_DEG_S = 20.0`, `_SCAN_MOVEJ_ACC_DEG_S2 = 30.0` for all cells.
- cuRobo `_init_motion_gen()` is no longer called in `_scan_sequence`.

## YAML endpoint_joints_deg values (v8, all cells)

| Cell | endpoint_joints_deg | J1 swing from overview |
|------|---------------------|------------------------|
| NW   | [129.6, -6.0, 100.2, 56.0, -50.2, -152.6] | 31.8° |
| NE   | [36.1, 6.0, 87.2, -59.7, -55.1, -20.3]   | 61.7° |
| SW   | [128.9, 37.3, 126.4, -140.4, 100.1, 79.7] | 31.1° |
| SE   | [35.8, 41.7, 111.6, -45.5, -94.3, -68.9]  | 62.0° |

Overview: [97.84, -94.40, 65.95, -10.93, 95.49, -94.79]

## Pending verification

- [ ] Run 4-cell traversal and confirm all 4 cells arrive at correct joints
- [ ] Check camera images: should show single cell filling FOV with crosshair centered
- [ ] If FOV still too wide/narrow at 0.35m standoff → re-run
      `search_all_cells_camera_centered.py` with adjusted standoff range
- [ ] SE J6 behaviour at runtime with direct MoveJoint (was winding up with cuRobo)
