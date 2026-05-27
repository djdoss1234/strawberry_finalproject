#!/usr/bin/env python3
"""
Dry-run cuRobo IK + motion planning validation for scan pose candidates.
NO robot motion is executed. Reads config files only.

Each candidate is classified as one of:
  PLAN_VALID              - cuRobo found a collision-free trajectory
  IK_FAIL                 - IK solver found no solution
  JOINT_LIMIT_REJECT      - start state or IK result violates joint limits
  START_STATE_COLLISION   - start state is in collision (world or self)
  PATH_MARGIN_REJECT      - IK succeeded but trajectory optimisation failed

Usage (run from project root):
  python3 scripts/validate_scan_poses.py
  python3 scripts/validate_scan_poses.py --output docs/runs/RUN-20260527-004_curobo_dry_run.yaml
"""

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import torch
import yaml

CUROBO_ROBOT_DIR = Path(
    "/home/user/doosan_ws/src/e0509_gripper_description/config/curobo"
)
URDF_PATH = CUROBO_ROBOT_DIR / "e0509_gripper.urdf"
ROBOT_YML_PATH = CUROBO_ROBOT_DIR / "e0509_gripper.yml"

# Read-only overview pose (operational-limit version from recorded_poses.yaml)
OVERVIEW_JOINTS_DEG = [97.84, -94.40, 65.95, -10.93, 95.49, -94.79]


def _mat4_to_pos_quat_wxyz(mat4: np.ndarray):
    """Return (position_xyz, quaternion_wxyz) from a 4x4 homogeneous matrix."""
    try:
        from scipy.spatial.transform import Rotation
    except ImportError:
        raise ImportError("scipy is required: pip install scipy")
    pos = mat4[:3, 3].tolist()
    q_xyzw = Rotation.from_matrix(mat4[:3, :3]).as_quat()  # xyzw
    q_wxyz = [float(q_xyzw[3]), float(q_xyzw[0]), float(q_xyzw[1]), float(q_xyzw[2])]
    return pos, q_wxyz


def _classify(status_name: str, success: bool) -> str:
    if success:
        return "PLAN_VALID"
    s = status_name.upper()
    if "IK_FAIL" in s:
        return "IK_FAIL"
    if "JOINT_LIMIT" in s:
        return "JOINT_LIMIT_REJECT"
    if "WORLD_COLLISION" in s or "SELF_COLLISION" in s or "START_STATE" in s:
        return "START_STATE_COLLISION"
    return "PATH_MARGIN_REJECT"


def run_validation(candidates_path: Path) -> dict:
    from curobo.types.base import TensorDeviceType
    from curobo.types.robot import JointState as CuroboJointState, RobotConfig
    from curobo.types.math import Pose
    from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig
    from curobo.geom.types import WorldConfig, Cuboid

    tensor_args = TensorDeviceType()

    # -- Load candidates -------------------------------------------------------
    with candidates_path.open() as fh:
        data = yaml.safe_load(fh)
    targets = data["scan_pose_candidates"]["targets"]
    standoff_m = data["scan_pose_candidates"]["standoff_m"]

    # -- Robot config (from_basic; collision spheres not required for IK check) -
    robot_cfg = RobotConfig.from_basic(
        urdf_path=str(URDF_PATH),
        base_link="base_link",
        ee_link="gripper_rh_p12_rn_base",
        tensor_args=tensor_args,
    )

    # -- World: floor only (no panel geometry yet) -----------------------------
    world_cfg = WorldConfig(cuboid=[
        Cuboid(name="floor", pose=[0.0, 0.0, -0.02, 1, 0, 0, 0], dims=[2.0, 2.0, 0.04]),
    ])

    # -- MotionGen setup -------------------------------------------------------
    print("  Loading cuRobo MotionGen … (first call compiles CUDA kernels)")
    t_load = time.time()
    motion_gen_cfg = MotionGenConfig.load_from_robot_config(
        robot_cfg, world_cfg, tensor_args=tensor_args,
        num_trajopt_seeds=16, num_graph_seeds=16,
        collision_cache={"obb": 30, "mesh": 10},
        use_cuda_graph=False,
    )
    motion_gen = MotionGen(motion_gen_cfg)
    motion_gen.warmup()
    print(f"  MotionGen ready in {time.time() - t_load:.1f} s")

    # -- Start state = overview pose -------------------------------------------
    q_start_rad = np.deg2rad(OVERVIEW_JOINTS_DEG).tolist()
    start_state = CuroboJointState.from_position(
        torch.tensor([q_start_rad], dtype=torch.float32, device=tensor_args.device)
    )

    # -- Plan to each scan pose ------------------------------------------------
    results = {}
    plan_config = MotionGenPlanConfig(
        enable_graph=True,
        max_attempts=4,
    )

    for target in targets:
        cell_id = target["cell_id"]
        mat4 = np.array(target["tcp_transform_base"])
        pos, q_wxyz = _mat4_to_pos_quat_wxyz(mat4)

        goal_pose = Pose(
            position=tensor_args.to_device(
                torch.tensor([pos], dtype=torch.float32)
            ),
            quaternion=tensor_args.to_device(
                torch.tensor([q_wxyz], dtype=torch.float32)
            ),
        )

        t_plan = time.time()
        result = motion_gen.plan_single(start_state, goal_pose, plan_config)
        elapsed = time.time() - t_plan

        success = bool(result.success.item())
        status_name = result.status.name if hasattr(result.status, "name") else str(result.status)
        label = _classify(status_name, success)

        results[cell_id] = {
            "label": label,
            "success": success,
            "curobo_status": status_name,
            "plan_time_s": round(elapsed, 3),
            "target_pos_m": [round(v, 4) for v in pos],
            "target_quat_wxyz": [round(v, 6) for v in q_wxyz],
        }
        marker = "OK" if success else "!!"
        print(f"    [{marker}] {cell_id:12s}  {label:28s}  ({elapsed:.2f}s)")

    return results


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="cuRobo dry-run validation – no robot motion"
    )
    parser.add_argument(
        "--candidates",
        default="config/scan_pose_candidates.yaml",
        help="path to scan_pose_candidates.yaml",
    )
    parser.add_argument(
        "--output",
        default=None,
        help="optional YAML file to save results",
    )
    args = parser.parse_args(argv)

    candidates_path = Path(args.candidates)
    if not candidates_path.is_absolute():
        candidates_path = Path(__file__).parent.parent / candidates_path

    if not candidates_path.exists():
        print(f"ERROR: candidates not found: {candidates_path}", file=sys.stderr)
        sys.exit(1)
    if not URDF_PATH.exists():
        print(f"ERROR: URDF not found: {URDF_PATH}", file=sys.stderr)
        sys.exit(1)

    print("=== cuRobo scan pose dry-run validation ===")
    print(f"  torch.cuda.is_available() = {torch.cuda.is_available()}")
    print(f"  CUDA device count         = {torch.cuda.device_count()}")
    print(f"  candidates                = {candidates_path}")
    print(f"  URDF                      = {URDF_PATH}")
    print(f"  start joints (deg)        = {OVERVIEW_JOINTS_DEG}")
    print()

    t0 = time.time()
    results = run_validation(candidates_path)
    total = time.time() - t0

    print()
    print("=== Summary ===")
    labels = {}
    for cell_id, r in results.items():
        label = r["label"]
        labels[label] = labels.get(label, 0) + 1
        print(f"  {cell_id:12s}  {label}")
    print(f"  Total time: {total:.1f} s")
    print(f"  Label counts: {labels}")

    output_data = {
        "scan_pose_validation": {
            "run_date": "2026-05-27",
            "method": "curobo_motion_gen_dry_run",
            "robot_urdf": str(URDF_PATH),
            "ee_link": "gripper_rh_p12_rn_base",
            "world_obstacles": ["floor"],
            "start_state": "overview_pose_operational_limit",
            "start_joints_deg": OVERVIEW_JOINTS_DEG,
            "total_elapsed_s": round(total, 2),
            "use_for_automated_motion": False,
            "results": results,
        }
    }

    if args.output:
        out_path = Path(args.output)
        if not out_path.is_absolute():
            out_path = Path(__file__).parent.parent / out_path
        out_path.parent.mkdir(parents=True, exist_ok=True)
        with out_path.open("w") as fh:
            yaml.dump(output_data, fh, sort_keys=False, allow_unicode=True)
        print(f"\nSaved: {out_path}")

    return output_data


if __name__ == "__main__":
    main()
