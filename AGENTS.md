# Strawberry Final Project Agent Guide

## Mission

Develop the motion-system side of a realistic strawberry harvesting robot while
supporting clean integration with a teammate's VLA-based complex-scene decision
module.

## Ownership Boundary

The repository owner's primary responsibility is:

- robot harvest motion behavior
- planning and execution integration
- approach, grasp, retreat, transfer, and placement sequences
- tray localization and place target generation
- collision scene, retry handling, diagnostics, and motion evaluation

The collaborator's primary responsibility is:

- VLA-based reasoning in complex harvest scenes
- semantic target selection and high-level action recommendations

Treat the VLA output as a proposal. Motion-side validation must confirm target
pose validity, workspace/collision constraints, execution readiness, and safe
recovery behavior before commanding the real robot.

## Baseline Reference

Read the mini-project repository before migrating runtime code:

- https://github.com/djdoss1234/strawberry_miniproject

It contains the previous RealSense/YOLO/curobo/Doosan prototype, system
architecture notes, experiment result summary, and future roadmap.

## Engineering Priorities

1. Preserve an executable baseline before major refactoring.
2. Make robot behavior measurable with explicit success and failure labels.
3. Separate target decisions from motion validation and execution.
4. Keep hardware configuration, calibration, and experiment artifacts out of
   public commits unless explicitly approved.
5. Treat rqt graph, TF frames, RViz scenes, and experiment logs as required
   debugging evidence for integration work.

## Planned Motion Architecture

```text
perception_or_vla_target
  -> target_validation
  -> pick_place_state_machine
  -> planner_backend
  -> collision_and_safety_check
  -> robot_executor
  -> result_logger
```

Expected packages/modules as implementation begins:

```text
perception/
planning/
task/
tray/
interfaces/
diagnostics/
config/
launch/
docs/
```

## Integration Contract Direction

The VLA workstream should eventually submit structured proposals containing:

- target identifier
- target pose or target region reference
- harvestability/confidence judgment
- occlusion or risk annotations
- requested action such as `PICK`, `REOBSERVE`, or `SKIP`

The motion workstream returns structured execution results such as:

- `SUCCESS`
- `INVALID_TARGET`
- `IK_FAIL`
- `PLANNING_FAIL`
- `COLLISION_RISK`
- `GRASP_FAIL`
- `PLACE_FAIL`
- `REOBSERVE_REQUIRED`

Do not finalize message types or topics without reviewing both workstreams'
runtime needs.

## First Implementation Order

1. Define metrics, hardware scene, interface topics/actions, and log schema.
2. Implement marker-based tray localization and slot generation.
3. Migrate stable pick-place motion components from the mini-project.
4. Refactor motion sequence into explicit state-machine and planner modules.
5. Run repeated realistic-mockup motion experiments.
6. Add the VLA proposal/result integration boundary.
