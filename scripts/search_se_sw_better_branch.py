#!/usr/bin/env python3
"""Find alternative IK branches for SE and SW that keep the same task pose
but with J1 in the same range as NE/NW (< 60° J1 swing from overview).

Current problem:
  SE: J1=-145.2° → J1 swing 233° from overview (87.98°)
  SW: J1=-39.2°  → J1 swing 127° from overview

Target:
  SE: J1 near +35° (like NE, same X position) → J1 swing ~53°
  SW: J1 near +140° (like NW, same X position) → J1 swing ~52°

Task poses are fixed (camera angle verified). We only want a different
joint configuration that reaches the same TCP transform.

Run:
  cd /home/user/doosan_ws/src/strawberry_finalproject
  python3 scripts/search_se_sw_better_branch.py
"""

from copy import deepcopy
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation
import torch
import yaml

PROJECT_ROOT = Path(__file__).resolve().parent.parent
CUROBO_DIR = Path("/home/user/doosan_ws/src/e0509_gripper_description/config/curobo")
ROBOT_YML = CUROBO_DIR / "e0509_gripper.yml"
URDF_PATH = CUROBO_DIR / "e0509_gripper.urdf"
SPHERES_PATH = CUROBO_DIR / "e0509_spheres.yml"
WORLD_PATH = PROJECT_ROOT / "config" / "scan_collision_world.yaml"
CANDIDATES_YAML = PROJECT_ROOT / "config" / "scan_pose_candidates_refit_candidate.yaml"

_JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

# Overview joints from the candidates YAML
OVERVIEW_DEG = [87.98, -94.92, 129.89, 175.94, -31.34, 93.42]

# Target J1 ranges for the "good" branch
# SE should be like NE (~35°), SW should be like NW (~140°)
SE_J1_TARGET = 34.82   # NE J1
SW_J1_TARGET = 139.76  # NW J1
J1_WINDOW_DEG = 40.0   # accept J1 within ±40° of target


def _load(p):
    with p.open(encoding="utf-8") as f:
        return yaml.safe_load(f)


def _mat4_to_pose(mat4):
    q = Rotation.from_matrix(np.asarray(mat4)[:3, :3]).as_quat()  # xyzw
    pos = np.asarray(mat4)[:3, 3].tolist()
    return pos, [float(q[3]), float(q[0]), float(q[1]), float(q[2])]  # wxyz


def _build_ik_solver(world_meta):
    from curobo.geom.types import Cuboid, WorldConfig
    from curobo.types.base import TensorDeviceType
    from curobo.types.robot import RobotConfig
    from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig

    ta = TensorDeviceType(device=torch.device("cuda:0"))
    cfg = deepcopy(_load(ROBOT_YML))
    cfg["robot_cfg"]["kinematics"]["urdf_path"] = str(URDF_PATH)
    cfg["robot_cfg"]["kinematics"]["collision_spheres"] = str(SPHERES_PATH)
    rc = RobotConfig.from_dict(cfg, tensor_args=ta)
    cuboids = [
        Cuboid(name=o["name"],
               pose=[float(v) for v in o["pose_wxyz"]],
               dims=[float(v) for v in o["dims_m"]])
        for o in world_meta["objects"]
        if o.get("enabled", True) and o.get("type") == "cuboid"
    ]
    wc = WorldConfig(cuboid=cuboids)
    ik_cfg = IKSolverConfig.load_from_robot_config(
        rc, wc, tensor_args=ta,
        num_seeds=64,
        self_collision_check=True,
        self_collision_opt=True,
    )
    return IKSolver(ik_cfg), ta


def _build_mg(world_meta):
    from curobo.geom.types import Cuboid, WorldConfig
    from curobo.types.base import TensorDeviceType
    from curobo.types.robot import RobotConfig
    from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig

    ta = TensorDeviceType(device=torch.device("cuda:0"))
    cfg = deepcopy(_load(ROBOT_YML))
    cfg["robot_cfg"]["kinematics"]["urdf_path"] = str(URDF_PATH)
    cfg["robot_cfg"]["kinematics"]["collision_spheres"] = str(SPHERES_PATH)
    rc = RobotConfig.from_dict(cfg, tensor_args=ta)
    cuboids = [
        Cuboid(name=o["name"],
               pose=[float(v) for v in o["pose_wxyz"]],
               dims=[float(v) for v in o["dims_m"]])
        for o in world_meta["objects"]
        if o.get("enabled", True) and o.get("type") == "cuboid"
    ]
    wc = WorldConfig(cuboid=cuboids)
    mc = MotionGenConfig.load_from_robot_config(
        rc, wc, tensor_args=ta,
        num_trajopt_seeds=32, num_graph_seeds=32,
        collision_cache={"obb": 30, "mesh": 10},
        use_cuda_graph=False,
        self_collision_check=True,
        self_collision_opt=True,
    )
    mg = MotionGen(mc)
    mg.warmup(warmup_js_trajopt=False)
    mg.detach_object_from_robot()
    return mg, ta


def _find_ik_candidates(ik_solver, ta, mat4, j1_target_deg, n_batches=20):
    """Run IK many times, collect solutions with J1 near j1_target_deg."""
    from curobo.types.math import Pose
    from curobo.wrap.reacher.ik_solver import IKSolverConfig

    pos, quat = _mat4_to_pose(mat4)
    goal = Pose(
        position=torch.tensor([pos], dtype=torch.float32, device=ta.device),
        quaternion=torch.tensor([quat], dtype=torch.float32, device=ta.device),
    )

    candidates = []
    for _ in range(n_batches):
        result = ik_solver.solve_single(goal)
        if not result.success.item():
            continue
        joints_rad = result.solution.position.cpu().numpy().flatten()
        joints_deg = np.rad2deg(joints_rad).tolist()
        j1 = joints_deg[0]
        j1_swing = abs(j1 - OVERVIEW_DEG[0])
        j1_dist = abs(j1 - j1_target_deg)
        candidates.append({
            "joints_deg": [round(v, 2) for v in joints_deg],
            "j1": round(j1, 2),
            "j1_swing": round(j1_swing, 1),
            "j1_dist_from_target": round(j1_dist, 1),
        })

    return candidates


def _verify_motion(mg, ta, mat4, cell_name, n_retry=8):
    """MotionGen path plan from overview to TCP target."""
    from curobo.types.math import Pose
    from curobo.types.robot import JointState as CJS
    from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig

    _OP_LIMITS = [(-225, 225), (-95, 95), (-155, 155), (-225, 225), (-130, 130), (-225, 225)]

    pos, quat = _mat4_to_pose(mat4)
    start = CJS.from_position(
        position=torch.tensor([np.deg2rad(OVERVIEW_DEG).tolist()],
                              dtype=torch.float32, device=ta.device),
        joint_names=_JOINT_NAMES,
    )
    goal = Pose(
        position=torch.tensor([pos], dtype=torch.float32, device=ta.device),
        quaternion=torch.tensor([quat], dtype=torch.float32, device=ta.device),
    )
    best_fail = None
    t0 = time.time()
    for _ in range(n_retry):
        r = mg.plan_single(start, goal, MotionGenPlanConfig(enable_graph=True, max_attempts=4))
        if not r.success.item():
            best_fail = {"status": "IK_FAIL", "endpoint_deg": None, "j1_swing": None}
            continue
        traj = r.get_interpolated_plan().position.cpu().numpy()
        deg = np.rad2deg(traj)
        ep = [round(float(d), 2) for d in deg[-1]]
        j1_swing = abs(ep[0] - OVERVIEW_DEG[0])
        ok = all(
            float(np.min(deg[:, j])) >= lo and float(np.max(deg[:, j])) <= hi
            for j, (lo, hi) in enumerate(_OP_LIMITS)
        )
        if ok:
            return {
                "status": "PLAN_VALID",
                "endpoint_deg": ep,
                "j1_swing": round(j1_swing, 1),
                "elapsed_s": round(time.time() - t0, 2),
            }
        best_fail = {
            "status": "JOINT_LIMIT_REJECT",
            "endpoint_deg": ep,
            "j1_swing": round(j1_swing, 1),
        }
    return best_fail or {"status": "PLAN_FAIL", "endpoint_deg": None, "j1_swing": None}


def main():
    cand_data = _load(CANDIDATES_YAML)["scan_pose_candidates"]
    targets = {t["cell_id"]: t for t in cand_data["targets"]}
    world_meta = _load(WORLD_PATH)["scan_collision_world"]

    cells_to_fix = {
        "root/se": (targets["root/se"]["tcp_transform_base"], SE_J1_TARGET),
        "root/sw": (targets["root/sw"]["tcp_transform_base"], SW_J1_TARGET),
    }

    print("Loading cuRobo IK solver...")
    ik_solver, ta = _build_ik_solver(world_meta)
    print("Loading cuRobo MotionGen...")
    mg, _ = _build_mg(world_meta)
    print("Ready.\n")

    all_results = {}

    for cell_id, (tcp_mat_rows, j1_target) in cells_to_fix.items():
        mat4 = np.array(tcp_mat_rows + [[0, 0, 0, 1]])
        current_j1 = targets[cell_id]["endpoint_joints_deg"][0]
        current_swing = abs(current_j1 - OVERVIEW_DEG[0])

        print(f"{'='*60}")
        print(f"{cell_id}")
        print(f"  Current J1={current_j1:.1f}°  swing={current_swing:.1f}°")
        print(f"  Target J1 ≈ {j1_target:.1f}° (window ±{J1_WINDOW_DEG}°)")
        print(f"  Running IK search ({20} batches × 64 seeds)...")

        candidates = _find_ik_candidates(ik_solver, ta, mat4, j1_target, n_batches=20)

        # Filter by J1 window
        good = [c for c in candidates if c["j1_dist_from_target"] <= J1_WINDOW_DEG]
        # Sort by j1_swing from overview
        good.sort(key=lambda c: c["j1_swing"])

        print(f"  Found {len(candidates)} IK solutions, {len(good)} in J1 window")
        if not good:
            print(f"  !! No solutions found near J1={j1_target}°")
            # Show closest
            if candidates:
                candidates.sort(key=lambda c: c["j1_dist_from_target"])
                print(f"  Closest: J1={candidates[0]['j1']:.1f}° (dist={candidates[0]['j1_dist_from_target']:.1f}°)")
            all_results[cell_id] = {"found": False, "candidates": []}
            continue

        print(f"\n  Top IK candidates (sorted by J1 swing from overview):")
        for i, c in enumerate(good[:5]):
            print(f"  [{i}] J1={c['j1']:+.1f}°  swing={c['j1_swing']:.1f}°  "
                  f"joints=[{', '.join(f'{v:+.1f}' for v in c['joints_deg'])}]")

        # Verify best candidate with MotionGen
        best = good[0]
        print(f"\n  Verifying best candidate with MotionGen (J1={best['j1']:.1f}°)...")
        mg_result = _verify_motion(mg, ta, mat4, cell_id)
        print(f"  MotionGen: status={mg_result['status']}  "
              f"ep_J1={mg_result['endpoint_deg'][0] if mg_result['endpoint_deg'] else '?'}°  "
              f"J1_swing={mg_result['j1_swing']}")

        all_results[cell_id] = {
            "found": True,
            "current_j1_deg": current_j1,
            "current_j1_swing_deg": current_swing,
            "target_j1_deg": j1_target,
            "ik_candidates_in_window": good[:10],
            "best_ik_candidate": best,
            "motion_gen_result": mg_result,
        }

    # Save results
    out = PROJECT_ROOT / "docs" / "runs" / "RUN-20260529-003_se_sw_branch_search.yaml"
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", encoding="utf-8") as f:
        yaml.safe_dump({
            "se_sw_branch_search": {
                "run_date": "2026-05-29",
                "description": "Find J1-aligned IK branch for SE/SW to reduce joint swing",
                "overview_joints_deg": OVERVIEW_DEG,
                "results": all_results,
            }
        }, f, sort_keys=False, allow_unicode=True)
    print(f"\nSaved: {out}")


if __name__ == "__main__":
    main()
