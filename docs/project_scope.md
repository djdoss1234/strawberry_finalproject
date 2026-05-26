# Project Scope And Collaboration Plan

## 1. Objective

Build a strawberry harvesting robot system that can perform repeatable
pick-and-place motions for realistic strawberry mockups and later operate with
high-level VLA judgments in more complex plant environments.

The first engineering priority is not semantic complexity. It is establishing a
motion system that can be measured, debugged, and safely integrated with the
VLA workstream.

## 2. Workstreams

### Motion Workstream: djdoss1234

Responsibilities:

- robot motion architecture and implementation
- target-pose validation for executable harvest actions
- motion planning, trajectory execution, and planner evaluation
- grasp/retreat/transfer/place sequence reliability
- tray detection integration and automatic slot placement
- collision environment maintenance
- execution logging, failure taxonomy, and recovery policy

Expected outputs:

- modular motion runtime code
- reusable motion/planner interfaces
- tray localization and placement demo
- benchmark results and experiment logs
- integrated end-to-end motion demonstrations

### VLA Workstream: Collaborating Team Member

Responsibilities:

- complex-scene visual-language reasoning
- candidate prioritization in occlusion or clutter
- harvestability and next-action judgments
- semantic explanation or metadata useful for recovery

Expected integration outputs:

- structured target/action proposal
- confidence and risk annotation
- reproducible VLA evaluation cases

## 3. Interface Principle

The integration boundary should separate reasoning from hardware motion:

```text
VLA / perception decision
  -> target proposal
  -> motion-side geometric and safety validation
  -> planner and executor
  -> structured execution result
  -> next decision
```

The motion system has final authority over whether a requested motion is
geometrically valid and safe to execute.

## 4. Motion Baseline Development

Reference repository:

- https://github.com/djdoss1234/strawberry_miniproject

The baseline supplies proven starting material for:

- strawberry target pipeline
- robot frame transformation
- cuRobo planning integration
- Doosan trajectory execution
- gripper operation
- taught placement workflow

For the final project, stable capabilities will be migrated into a modular
architecture with experiment-grade logging and explicit interfaces.

## 5. First Functional Milestone

### Movable Tray Automatic Placement

Goal:

> After the harvesting tray is manually moved, the robot recognizes the new
> tray pose and places a harvested mock strawberry into a generated empty slot.

Implementation outline:

1. Attach an AprilTag or ArUco marker to a rigid tray frame.
2. Detect tray pose using the RGB-D camera.
3. Transform tray pose into `base_link`.
4. Generate slot `above` and `release` poses from tray geometry.
5. Check slot availability using RGB-D observations.
6. Execute placement through the motion pipeline.
7. Record pose error, placement result, and failure reason.

Why this milestone comes first:

- removes the current fixed teaching dependency
- provides a measurable environment-change test
- exercises perception-to-motion integration without depending on VLA
- supplies a clear portfolio-level comparison against the mini-project

## 6. Measurement Plan

Motion metrics:

| Metric | Description |
| --- | --- |
| planning success rate | Valid trajectory generation per eligible target |
| execution success rate | Completed motion without abort/collision stop |
| grasp success rate | Fruit retained after grasp and retreat |
| place success rate | Fruit placed into intended slot |
| tray relocation success | Placement success after tray movement |
| cycle time | Target acceptance to completed placement |
| failure distribution | Count grouped by defined failure code |

Initial failure codes:

```text
INVALID_TARGET
TF_UNAVAILABLE
TRAY_NOT_FOUND
TRAY_POSE_UNCERTAIN
SLOT_OCCUPIED
IK_FAIL
PLANNING_FAIL
COLLISION_RISK
EXECUTION_ABORT
GRASP_FAIL
PLACE_FAIL
SUCCESS
```

## 7. Initial Sprint

1. Photograph and dimension the realistic mockup workspace and movable tray.
2. Decide the marker type, tray coordinate frame, slot pitch, and slot count.
3. Define ROS topics/actions and the experiment log schema.
4. Establish a package layout for motion, tray, planning, and diagnostics.
5. Implement tray marker detection and slot-pose generation.
6. Test placement with the tray at three measured positions.
7. Capture `rqt_graph`, TF tree, RViz scene, and labeled trial results.

## 8. Later Integration

After the tray and motion baselines are measurable:

- compare planner behavior on identical target/scene inputs
- add realistic obstacle and collision models
- introduce recovery/retry policies
- receive VLA target proposals through a versioned interface
- test complex-scene harvest decisions with deterministic motion validation
