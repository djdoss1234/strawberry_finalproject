# RUN-20260529-001: SW Workspace Alignment Troubleshooting

## Summary

`root/sw` scan pose failed repeatedly before the workspace reset even though the
cell was physically close to the robot. The failure was not a simple reach
distance issue. It was a combined task-space and joint-space feasibility issue:
the eye-in-hand camera view constrained TCP position and orientation at the same
time, pushing the robot toward poor joint branches and joint-limit margins.

## Observed Symptoms

- `MoveSplineJoint` returned `success=True`, but the robot did not move.
- Direct `MoveJoint` to the endpoint also returned `success=True`, but the robot
  did not arrive within the timeout.
- Staged `MoveJoint` diagnostics reached intermediate stages but repeatedly
  failed near the final SW pose.
- Increasing the number of stages reduced jump size but did not remove the
  terminal failure region.

## Root Cause Interpretation

For a scan pose, the robot is not only asked to reach a 3D point. It must reach
that point while keeping the camera looking at the paper cell. This couples:

- TCP position
- camera optical direction
- wrist orientation
- J1 branch
- shoulder/elbow reach
- J4/J5/J6 wrist limits

The previous physical board placement made the panel center offset from the
robot `base_link` centerline. In that geometry, the SW cell was close in a
Cartesian sense but required an awkward low and side-biased configuration with
large wrist/arm changes. This is why "near the robot" did not mean "easy for the
planner and controller."

## Why More Waypoints Were Not Enough

Waypoint subdivision helps when the problem is a large command jump. It does
not solve a target pose that lies in an unfavorable or limit-adjacent joint
branch. The repeated failures around the late staged waypoints indicated that
the final SW camera pose itself was poorly conditioned for the current physical
layout.

## Corrective Action

The panel was physically moved so the paper center and `base_link` centerline
were better aligned. After this reset:

- a new `base_link -> cultivation_panel` registration was captured,
- a new overview pose was recorded,
- previous scan candidates were marked as superseded,
- scan poses are being retaught manually from the new geometry.

## Interview Explanation

The important lesson is that motion planning is not only a software planner
problem. If the task fixture is placed in a poor region of the robot workspace,
the planner may find brittle IK branches or controller-rejected motions even
when the target appears close. I debugged this by comparing planner success,
service response, actual joint arrival, staged waypoint behavior, and camera
view. The resolution was to redesign the physical workspace alignment so the
robot operates in a better manipulability region before retuning scan poses.

