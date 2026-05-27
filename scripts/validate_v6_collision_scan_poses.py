#!/usr/bin/env python3
"""Collision-aware offline check for the v6 scan pose candidates.

No ROS service or robot command is used. The check loads the mini-project
robot/tool collision sphere baseline and the registered-whiteboard world in
``config/scan_collision_world.yaml``.
"""

import argparse
from copy import deepcopy
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation
import torch
import yaml


PROJECT_ROOT = Path(__file__).resolve().parent.parent
BASELINE_CUROBO_DIR = Path(
    "/home/user/doosan_ws/src/e0509_gripper_description/config/curobo"
)
ROBOT_YML_PATH = BASELINE_CUROBO_DIR / "e0509_gripper.yml"
URDF_PATH = BASELINE_CUROBO_DIR / "e0509_gripper.urdf"
SPHERES_PATH = BASELINE_CUROBO_DIR / "e0509_spheres.yml"
DEFAULT_CANDIDATES = PROJECT_ROOT / "config" / "scan_pose_candidates.yaml"
DEFAULT_WORLD = PROJECT_ROOT / "config" / "scan_collision_world.yaml"
OVERVIEW_JOINTS_DEG = [97.84, -94.40, 65.95, -10.93, 95.49, -94.79]
JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
OP_LIMITS_DEG = [
    (-225.0, 225.0),
    (-95.0, 95.0),
    (-155.0, 155.0),
    (-170.0, 170.0),
    (-130.0, 130.0),
    (-225.0, 225.0),
]


def _load_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def _display_path(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(PROJECT_ROOT))
    except ValueError:
        return str(path)


def load_inputs(candidates_path: Path, world_path: Path):
    candidate_cfg = _load_yaml(candidates_path)["scan_pose_candidates"]
    world_cfg = _load_yaml(world_path)["scan_collision_world"]
    return candidate_cfg, world_cfg


def _mat4_to_pose(mat4):
    matrix = np.asarray(mat4, dtype=float)
    q_xyzw = Rotation.from_matrix(matrix[:3, :3]).as_quat()
    return matrix[:3, 3].tolist(), [
        float(q_xyzw[3]), float(q_xyzw[0]), float(q_xyzw[1]), float(q_xyzw[2])
    ]


def _classify(status_name: str, success: bool) -> str:
    if success:
        return "PLAN_VALID"
    status = status_name.upper()
    if "IK_FAIL" in status:
        return "IK_FAIL"
    if "COLLISION" in status or "START_STATE" in status:
        return "COLLISION_REJECT"
    if "JOINT_LIMIT" in status:
        return "JOINT_LIMIT_REJECT"
    return "PLAN_FAIL"


def _joint_range_result(traj_rad: np.ndarray):
    trajectory_deg = np.rad2deg(traj_rad)
    ranges = {}
    accepted = True
    for idx, (lower, upper) in enumerate(OP_LIMITS_DEG):
        minimum = float(np.min(trajectory_deg[:, idx]))
        maximum = float(np.max(trajectory_deg[:, idx]))
        ranges[f"J{idx + 1}"] = [round(minimum, 2), round(maximum, 2)]
        if minimum < lower or maximum > upper:
            accepted = False
    return accepted, ranges


def run_validation(candidate_cfg: dict, world_metadata: dict) -> dict:
    from curobo.geom.types import Cuboid, WorldConfig
    from curobo.types.base import TensorDeviceType
    from curobo.types.math import Pose
    from curobo.types.robot import JointState as CuroboJointState, RobotConfig
    from curobo.wrap.reacher.motion_gen import (
        MotionGen,
        MotionGenConfig,
        MotionGenPlanConfig,
    )

    tensor_args = TensorDeviceType(device=torch.device("cuda:0"))
    robot_cfg_data = deepcopy(_load_yaml(ROBOT_YML_PATH))
    robot_kinematics = robot_cfg_data["robot_cfg"]["kinematics"]
    robot_kinematics["urdf_path"] = str(URDF_PATH)
    robot_kinematics["collision_spheres"] = str(SPHERES_PATH)
    robot_cfg = RobotConfig.from_dict(robot_cfg_data, tensor_args=tensor_args)

    cuboids = [
        Cuboid(
            name=obj["name"],
            pose=[float(value) for value in obj["pose_wxyz"]],
            dims=[float(value) for value in obj["dims_m"]],
        )
        for obj in world_metadata["objects"]
        if obj.get("enabled", True) and obj.get("type") == "cuboid"
    ]
    world_cfg = WorldConfig(cuboid=cuboids)

    motion_cfg = MotionGenConfig.load_from_robot_config(
        robot_cfg,
        world_cfg,
        tensor_args=tensor_args,
        num_trajopt_seeds=16,
        num_graph_seeds=16,
        collision_cache={"obb": 30, "mesh": 10},
        use_cuda_graph=False,
        self_collision_check=world_metadata["self_collision_check_enabled"],
        self_collision_opt=world_metadata["self_collision_check_enabled"],
    )
    motion_gen = MotionGen(motion_cfg)
    motion_gen.warmup(warmup_js_trajopt=False)
    motion_gen.detach_object_from_robot()

    start_state = CuroboJointState.from_position(
        position=torch.tensor(
            [np.deg2rad(OVERVIEW_JOINTS_DEG).tolist()],
            dtype=torch.float32,
            device=tensor_args.device,
        ),
        joint_names=JOINT_NAMES,
    )
    plan_config = MotionGenPlanConfig(enable_graph=True, max_attempts=4)
    results = {}

    for target in candidate_cfg["targets"]:
        position, quaternion = _mat4_to_pose(target["tcp_transform_base"])
        goal = Pose(
            position=torch.tensor([position], dtype=torch.float32, device=tensor_args.device),
            quaternion=torch.tensor([quaternion], dtype=torch.float32, device=tensor_args.device),
        )
        started = time.time()
        result = motion_gen.plan_single(start_state, goal, plan_config)
        elapsed = time.time() - started
        success = bool(result.success.item())
        raw_status = result.status.name if hasattr(result.status, "name") else str(result.status)
        if success and raw_status in ("None", "NoneType"):
            raw_status = "SUCCESS"
        label = _classify(raw_status, success)
        joint_ranges = None
        if success:
            within_limits, joint_ranges = _joint_range_result(
                result.get_interpolated_plan().position.cpu().numpy()
            )
            if not within_limits:
                success = False
                label = "JOINT_LIMIT_REJECT"
        results[target["cell_id"]] = {
            "approach": target["approach"],
            "standoff_m": target["standoff_m"],
            "label": label,
            "success": success,
            "curobo_status": raw_status,
            "plan_time_s": round(elapsed, 3),
            "target_pos_m": [round(value, 6) for value in position],
            "joint_range_deg": joint_ranges,
        }
        print(f"  {target['cell_id']:8s} {label:20s} {elapsed:.2f}s")
    return results


def main(argv=None):
    parser = argparse.ArgumentParser(description="Offline v6 collision-aware scan validation")
    parser.add_argument("--candidates", type=Path, default=DEFAULT_CANDIDATES)
    parser.add_argument("--world", type=Path, default=DEFAULT_WORLD)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args(argv)
    candidate_cfg, world_cfg = load_inputs(args.candidates, args.world)

    print("=== v6 registered-whiteboard collision dry-run; no robot command ===")
    print(f"torch.cuda.is_available() = {torch.cuda.is_available()}")
    print(f"robot spheres             = {SPHERES_PATH}")
    print(f"world                     = {args.world}")
    print(f"scope                     = {world_cfg['validation_scope']}")
    if not torch.cuda.is_available():
        raise SystemExit(
            "CUDA GPU is not available in this process; collision dry-run was not executed."
        )
    started = time.time()
    results = run_validation(candidate_cfg, world_cfg)
    elapsed = time.time() - started
    output = {
        "scan_pose_collision_validation": {
            "run_date": "2026-05-27",
            "method": "curobo_motion_gen_registered_whiteboard_dry_run",
            "candidate_version": candidate_cfg["version"],
            "robot_geometry_source": str(BASELINE_CUROBO_DIR),
            "robot_collision_spheres_enabled": True,
            "self_collision_check_enabled": world_cfg["self_collision_check_enabled"],
            "collision_world_config": _display_path(args.world),
            "active_obstacles": [
                obj["name"] for obj in world_cfg["objects"] if obj.get("enabled", True)
            ],
            "validation_scope": world_cfg["validation_scope"],
            "start_state": "overview_pose_operational_limit",
            "start_joints_deg": OVERVIEW_JOINTS_DEG,
            "total_elapsed_s": round(elapsed, 2),
            "use_for_automated_motion": False,
            "results": results,
        }
    }
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(output, stream, sort_keys=False, allow_unicode=True)
        print(f"Saved: {args.output}")
    return output


if __name__ == "__main__":
    main()
