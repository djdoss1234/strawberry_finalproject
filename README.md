# Strawberry Final Project

Realistic strawberry harvesting robot project based on the validated mini-project
prototype.

This repository is the main project workspace for extending the existing
RealSense, YOLO, cuRobo, and Doosan E0509 pick-and-place pipeline into a
repeatable harvesting system for realistic strawberry mockups and changing
workspace conditions.

## Project Focus

The project is developed collaboratively with separated responsibilities:

| Area | Primary owner | Scope |
| --- | --- | --- |
| Harvest motion system | djdoss1234 | approach, grasp, retreat, transfer, tray placement, planning/execution integration, failure recovery, evaluation |
| VLA-based complex-scene harvesting | Team collaborator | semantic target selection, occlusion/complex-scene reasoning, high-level harvest decisions |
| Integration | Team | agreed target/action interfaces, experiment protocol, end-to-end demonstrations |

The motion system remains deterministic and safety-checkable. A VLA component
may propose targets or task decisions, but it does not directly send unchecked
robot trajectories to hardware.

## Baseline

The prior mini-project repository is the validated starting reference:

- [strawberry_miniproject](https://github.com/djdoss1234/strawberry_miniproject)

Baseline capabilities already demonstrated or implemented include:

- RGB-D strawberry candidate detection and 3D target generation
- eye-in-hand coordinate transformation
- cuRobo-based approach, grasp, and retreat planning
- Doosan robot execution and gripper control
- taught tray-slot placement
- documented experimental evidence and known limitations

This final project will migrate only stable baseline components as they are
needed, rather than mixing exploratory prototype code directly into the new
architecture.

## Motion-System Goals

The motion workstream will implement and evaluate:

- reliable motion execution for realistic strawberry mockups
- modular planning and task-sequencing architecture
- tray localization and automatic placement target generation
- collision-world management for plant, tray, and placed fruit constraints
- retry and failure-classification policies
- planner comparison and trajectory-quality analysis
- integration interface for targets selected by the VLA workstream

## Initial Roadmap

1. Freeze hardware setup, task definition, metrics, and ROS interfaces.
2. Build a realistic mockup testbed with movable tray and marker-based localization.
3. Migrate/refactor the stable motion baseline into reusable modules.
4. Implement automatic tray-frame slot generation and placement evaluation.
5. Establish motion-planner benchmarking and failure logging.
6. Integrate VLA-generated high-level harvest proposals through a validated interface.

Detailed ownership, architecture, interfaces, and first sprint tasks are in
[docs/project_scope.md](docs/project_scope.md).

## Repository Status

This repository has been initialized for the final project. Runtime robot code
will be brought in after the motion baseline boundaries and experimental setup
are finalized.

## Data And Safety Policy

Do not commit:

- robot/camera calibration files containing local setup values
- trained weights unless distribution is explicitly intended
- raw camera logs or experiment videos
- credentials, tokens, or machine-specific ROS/network settings

Robot execution changes must be tested with reduced speed and a documented
collision scene before full pick-and-place trials.
