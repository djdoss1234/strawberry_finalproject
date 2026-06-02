#!/usr/bin/env python3
"""Optimize scan-pose joint branches while keeping taught TCP poses fixed.

This script is intentionally offline by default. It does not move the robot.

Goal:
  - Keep the manually verified gripper-centered TCP transforms.
  - Search cuRobo IK solutions for each cell.
  - Pick a sequence of joint branches that reduces inter-cell joint motion,
    wrist wind-up, and joint-limit pressure.
  - Write a run report. Only update the candidate YAML with --apply.

Run:
  cd /home/user/doosan_ws/src/strawberry_finalproject
  python3 scripts/optimize_scan_joint_branches.py

Apply suggested endpoint_joints_deg to config:
  python3 scripts/optimize_scan_joint_branches.py --apply
"""

from __future__ import annotations

import argparse
from copy import deepcopy
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

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

JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
SCAN_ORDER = ["root/nw", "root/ne", "root/se", "root/sw"]
WRAP_JOINTS = {0, 3, 5}  # J1/J4/J6 have equivalent angle representations every 360 deg.

# Hard model limits from the cuRobo robot config used by scan_executor_node.
HARD_LIMITS_DEG = [
    tuple(np.rad2deg(v) for v in (-6.273185, 6.273185)),
    tuple(np.rad2deg(v) for v in (-1.648063, 1.648063)),
    tuple(np.rad2deg(v) for v in (-2.6953, 2.6953)),
    tuple(np.rad2deg(v) for v in (-6.273185, 6.273185)),
    tuple(np.rad2deg(v) for v in (-2.346194, 2.346194)),
    tuple(np.rad2deg(v) for v in (-6.273185, 6.273185)),
]

WEIGHTS = {
    "l1_delta": 1.0,
    "max_delta": 1.7,
    "wrist_delta": 1.4,
    "limit_pressure": 35.0,
    "taught_deviation": 0.08,
}


def _load_yaml(path: Path) -> dict:
    with path.open(encoding="utf-8") as f:
        return yaml.safe_load(f)


def _save_yaml(path: Path, data: dict) -> None:
    with path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True)


def _mat4_from_rows(rows: Sequence[Sequence[float]]) -> np.ndarray:
    arr = np.asarray(rows, dtype=float)
    if arr.shape == (3, 4):
        arr = np.vstack([arr, [0.0, 0.0, 0.0, 1.0]])
    if arr.shape != (4, 4):
        raise ValueError(f"Expected 3x4 or 4x4 transform, got {arr.shape}")
    return arr


def _mat4_to_pose(mat4: np.ndarray) -> Tuple[List[float], List[float]]:
    q_xyzw = Rotation.from_matrix(mat4[:3, :3]).as_quat()
    return (
        mat4[:3, 3].astype(float).tolist(),
        [float(q_xyzw[3]), float(q_xyzw[0]), float(q_xyzw[1]), float(q_xyzw[2])],
    )


def _equivalent_near(target_deg: Sequence[float], reference_deg: Sequence[float]) -> List[float]:
    adjusted = [float(v) for v in target_deg]
    for idx in WRAP_JOINTS:
        base = adjusted[idx]
        lo, hi = HARD_LIMITS_DEG[idx]
        candidates = [base + 360.0 * k for k in range(-2, 3)]
        candidates = [c for c in candidates if lo <= c <= hi]
        if candidates:
            adjusted[idx] = min(candidates, key=lambda c: abs(c - reference_deg[idx]))
    return adjusted


def _limit_pressure(joints_deg: Sequence[float]) -> float:
    pressure = 0.0
    for value, (lo, hi) in zip(joints_deg, HARD_LIMITS_DEG):
        center = 0.5 * (lo + hi)
        half_range = 0.5 * (hi - lo)
        if half_range <= 0:
            continue
        normalized = abs(float(value) - center) / half_range
        # Only penalize the final 15% near a hard limit.
        pressure += max(0.0, normalized - 0.85) ** 2
    return pressure


def _transition_cost(prev_deg: Sequence[float], curr_deg: Sequence[float], taught_deg: Sequence[float]) -> float:
    delta = np.abs(np.asarray(curr_deg, dtype=float) - np.asarray(prev_deg, dtype=float))
    taught_delta = np.abs(np.asarray(curr_deg, dtype=float) - np.asarray(taught_deg, dtype=float))
    wrist_delta = delta[3] + delta[5]
    return float(
        WEIGHTS["l1_delta"] * np.sum(delta)
        + WEIGHTS["max_delta"] * np.max(delta)
        + WEIGHTS["wrist_delta"] * wrist_delta
        + WEIGHTS["limit_pressure"] * _limit_pressure(curr_deg)
        + WEIGHTS["taught_deviation"] * np.sum(taught_delta)
    )


def _candidate_key(joints_deg: Sequence[float]) -> Tuple[int, ...]:
    return tuple(int(round(v * 10.0)) for v in joints_deg)


def _build_ik_solver(world_meta: dict):
    from curobo.geom.types import Cuboid, WorldConfig
    from curobo.types.base import TensorDeviceType
    from curobo.types.robot import RobotConfig
    from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig

    tensor_args = TensorDeviceType(device=torch.device("cuda:0"))
    cfg = deepcopy(_load_yaml(ROBOT_YML))
    cfg["robot_cfg"]["kinematics"]["urdf_path"] = str(URDF_PATH)
    cfg["robot_cfg"]["kinematics"]["collision_spheres"] = str(SPHERES_PATH)
    robot_cfg = RobotConfig.from_dict(cfg, tensor_args=tensor_args)
    cuboids = [
        Cuboid(
            name=o["name"],
            pose=[float(v) for v in o["pose_wxyz"]],
            dims=[float(v) for v in o["dims_m"]],
        )
        for o in world_meta["objects"]
        if o.get("enabled", True) and o.get("type") == "cuboid"
    ]
    world_cfg = WorldConfig(cuboid=cuboids)
    self_collision = bool(world_meta.get("self_collision_check_enabled", False))
    ik_cfg = IKSolverConfig.load_from_robot_config(
        robot_cfg,
        world_cfg,
        tensor_args=tensor_args,
        num_seeds=128,
        self_collision_check=self_collision,
        self_collision_opt=self_collision,
    )
    return IKSolver(ik_cfg), tensor_args


def _ik_candidates_for_cell(
    ik_solver,
    tensor_args,
    mat4: np.ndarray,
    taught_deg: Sequence[float],
    overview_deg: Sequence[float],
    batches: int,
) -> List[dict]:
    from curobo.types.math import Pose

    pos, quat_wxyz = _mat4_to_pose(mat4)
    goal = Pose(
        position=torch.tensor([pos], dtype=torch.float32, device=tensor_args.device),
        quaternion=torch.tensor([quat_wxyz], dtype=torch.float32, device=tensor_args.device),
    )

    candidates: Dict[Tuple[int, ...], dict] = {}

    # Always include the physically verified taught pose. This keeps the optimizer
    # fail-safe if IK search cannot find a better equivalent branch.
    taught_norm = _equivalent_near(taught_deg, overview_deg)
    candidates[_candidate_key(taught_norm)] = {
        "source": "taught_equivalent",
        "joints_deg": [round(v, 3) for v in taught_norm],
    }

    for _ in range(batches):
        result = ik_solver.solve_single(goal)
        success = getattr(result, "success", None)
        if success is None or not bool(success.item()):
            continue
        solution = getattr(result, "solution", None)
        if solution is None:
            solution = getattr(result, "js_solution", None)
        if solution is None:
            continue
        if hasattr(solution, "position"):
            joints_tensor = solution.position
        else:
            joints_tensor = solution
        joints_rad = joints_tensor.detach().cpu().numpy().reshape(-1)[:6]
        raw_deg = np.rad2deg(joints_rad).tolist()
        # Normalize once relative to overview; dynamic-programming will normalize
        # again relative to the previous selected branch for transition scoring.
        norm_deg = _equivalent_near(raw_deg, overview_deg)
        key = _candidate_key(norm_deg)
        candidates.setdefault(
            key,
            {
                "source": "curobo_ik",
                "joints_deg": [round(v, 3) for v in norm_deg],
            },
        )

    for cand in candidates.values():
        joints = cand["joints_deg"]
        cand["limit_pressure"] = round(_limit_pressure(joints), 5)
        cand["taught_l1_delta_deg"] = round(
            float(np.sum(np.abs(np.asarray(joints) - np.asarray(taught_norm)))), 3
        )
        cand["overview_l1_delta_deg"] = round(
            float(np.sum(np.abs(np.asarray(joints) - np.asarray(overview_deg)))), 3
        )
    return list(candidates.values())


def _optimize_sequence(
    cell_candidates: Dict[str, List[dict]],
    taught_by_cell: Dict[str, List[float]],
    overview_deg: Sequence[float],
) -> Tuple[List[dict], float]:
    layers: List[Dict[int, Tuple[float, int | None, List[float]]]] = []

    prev_options = [list(map(float, overview_deg))]
    prev_costs = [0.0]

    for cell_id in SCAN_ORDER:
        layer: Dict[int, Tuple[float, int | None, List[float]]] = {}
        candidates = cell_candidates[cell_id]
        for ci, cand in enumerate(candidates):
            raw = list(map(float, cand["joints_deg"]))
            best_cost = None
            best_prev = None
            best_adjusted = None
            for pi, prev in enumerate(prev_options):
                adjusted = _equivalent_near(raw, prev)
                cost = prev_costs[pi] + _transition_cost(prev, adjusted, taught_by_cell[cell_id])
                if best_cost is None or cost < best_cost:
                    best_cost = cost
                    best_prev = pi
                    best_adjusted = adjusted
            if best_cost is not None and best_adjusted is not None:
                layer[ci] = (best_cost, best_prev, best_adjusted)
        if not layer:
            raise RuntimeError(f"No candidates available for {cell_id}")
        layers.append(layer)
        prev_options = [layer[i][2] for i in sorted(layer)]
        prev_costs = [layer[i][0] for i in sorted(layer)]

    # Add final return-to-overview cost.
    last_layer = layers[-1]
    best_final = None
    best_final_key = None
    for key, (cost, prev_key, adjusted) in last_layer.items():
        return_target = _equivalent_near(overview_deg, adjusted)
        total = cost + _transition_cost(adjusted, return_target, overview_deg)
        if best_final is None or total < best_final:
            best_final = total
            best_final_key = key

    if best_final_key is None or best_final is None:
        raise RuntimeError("Failed to select optimized sequence")

    selected_reversed = []
    next_key = best_final_key
    for li in range(len(layers) - 1, -1, -1):
        layer = layers[li]
        cost, prev_key, adjusted = layer[next_key]
        cell_id = SCAN_ORDER[li]
        selected_reversed.append(
            {
                "cell_id": cell_id,
                "candidate_index": next_key,
                "source": cell_candidates[cell_id][next_key]["source"],
                "joints_deg": [round(v, 3) for v in adjusted],
                "cumulative_cost": round(cost, 3),
            }
        )
        next_key = prev_key if prev_key is not None else 0

    return list(reversed(selected_reversed)), float(best_final)


def _transition_table(sequence: Sequence[dict], overview_deg: Sequence[float]) -> List[dict]:
    rows = []
    prev_label = "overview"
    prev = list(map(float, overview_deg))
    for item in sequence:
        curr = list(map(float, item["joints_deg"]))
        delta = np.asarray(curr) - np.asarray(prev)
        rows.append(
            {
                "from": prev_label,
                "to": item["cell_id"],
                "delta_deg": [round(float(v), 2) for v in delta],
                "l1_delta_deg": round(float(np.sum(np.abs(delta))), 2),
                "max_abs_delta_deg": round(float(np.max(np.abs(delta))), 2),
                "wrist_abs_delta_deg": round(float(abs(delta[3]) + abs(delta[5])), 2),
            }
        )
        prev_label = item["cell_id"]
        prev = curr
    delta = np.asarray(_equivalent_near(overview_deg, prev)) - np.asarray(prev)
    rows.append(
        {
            "from": prev_label,
            "to": "overview",
            "delta_deg": [round(float(v), 2) for v in delta],
            "l1_delta_deg": round(float(np.sum(np.abs(delta))), 2),
            "max_abs_delta_deg": round(float(np.max(np.abs(delta))), 2),
            "wrist_abs_delta_deg": round(float(abs(delta[3]) + abs(delta[5])), 2),
        }
    )
    return rows


def _apply_to_yaml(candidate_data: dict, selected: Sequence[dict]) -> dict:
    updated = deepcopy(candidate_data)
    selected_by_cell = {item["cell_id"]: item for item in selected}
    cfg = updated["scan_pose_candidates"]
    cfg["version"] = "v13_curobo_branch_optimized_from_v12"
    cfg["status"] = "curobo_branch_optimized_pending_physical_validation"
    cfg["branch_optimization_record"] = (
        "docs/runs/RUN-20260602-001_curobo_scan_branch_optimization.yaml"
    )
    cfg["validation_scope"] = (
        str(cfg.get("validation_scope", ""))
        + " v13 keeps the v12 manually verified TCP poses fixed and updates only "
        "endpoint_joints_deg using offline cuRobo IK branch optimization. Physical "
        "single-cell and traversal validation are required before unattended use."
    )
    for target in cfg["targets"]:
        cell_id = target["cell_id"]
        if cell_id not in selected_by_cell:
            continue
        target["approach"] = "manual_tcp_curobo_branch_optimized"
        target["curobo_status"] = "IK_BRANCH_OPTIMIZED_PENDING_PHYSICAL_VALIDATION"
        target["endpoint_joints_deg"] = [
            round(float(v), 2) for v in selected_by_cell[cell_id]["joints_deg"]
        ]
        target["branch_optimization_note"] = (
            "TCP transform preserved from v12 manual gripper-centered teaching; "
            "joint endpoint selected offline to reduce inter-cell joint motion."
        )
    return updated


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--batches", type=int, default=80, help="IK solve batches per cell")
    parser.add_argument("--apply", action="store_true", help="Update candidates YAML")
    args = parser.parse_args()

    candidate_data = _load_yaml(CANDIDATES_YAML)
    cfg = candidate_data["scan_pose_candidates"]
    targets = {t["cell_id"]: t for t in cfg["targets"]}
    overview_deg = [float(v) for v in cfg["curobo_start_joints_deg"]]
    world_meta = _load_yaml(WORLD_PATH)["scan_collision_world"]

    print("Loading cuRobo IK solver...")
    ik_solver, tensor_args = _build_ik_solver(world_meta)

    cell_candidates: Dict[str, List[dict]] = {}
    taught_by_cell: Dict[str, List[float]] = {}
    for cell_id in SCAN_ORDER:
        target = targets[cell_id]
        taught = [float(v) for v in target["endpoint_joints_deg"]]
        taught_by_cell[cell_id] = taught
        mat4 = _mat4_from_rows(target["tcp_transform_base"])
        print(f"{cell_id}: searching IK branches ({args.batches} batches)...")
        candidates = _ik_candidates_for_cell(
            ik_solver, tensor_args, mat4, taught, overview_deg, args.batches
        )
        candidates.sort(key=lambda c: (c["limit_pressure"], c["overview_l1_delta_deg"]))
        cell_candidates[cell_id] = candidates
        print(f"  candidates: {len(candidates)}")

    selected, total_cost = _optimize_sequence(cell_candidates, taught_by_cell, overview_deg)
    transitions = _transition_table(selected, overview_deg)

    report = {
        "curobo_scan_branch_optimization": {
            "run_date": datetime.now().strftime("%Y-%m-%d"),
            "source_candidates": str(CANDIDATES_YAML.relative_to(PROJECT_ROOT)),
            "source_world": str(WORLD_PATH.relative_to(PROJECT_ROOT)),
            "description": (
                "Offline cuRobo IK branch optimization with v12 TCP transforms fixed. "
                "No robot motion is executed by this script."
            ),
            "scan_order": SCAN_ORDER,
            "overview_joints_deg": [round(v, 3) for v in overview_deg],
            "weights": WEIGHTS,
            "total_sequence_cost": round(total_cost, 3),
            "selected_sequence": selected,
            "transition_table": transitions,
            "candidate_counts": {k: len(v) for k, v in cell_candidates.items()},
            "top_candidates_by_cell": {
                k: v[:10] for k, v in cell_candidates.items()
            },
        }
    }

    out = PROJECT_ROOT / "docs" / "runs" / "RUN-20260602-001_curobo_scan_branch_optimization.yaml"
    _save_yaml(out, report)
    print(f"\nSaved report: {out}")
    print("\nSelected endpoints:")
    for item in selected:
        print(
            "  {cell}: [{joints}]  source={source}".format(
                cell=item["cell_id"],
                joints=", ".join(f"{v:.2f}" for v in item["joints_deg"]),
                source=item["source"],
            )
        )
    print("\nTransition max deltas:")
    for row in transitions:
        print(
            "  {frm} -> {to}: max={mx:.1f} l1={l1:.1f} wrist={wr:.1f}".format(
                frm=row["from"],
                to=row["to"],
                mx=row["max_abs_delta_deg"],
                l1=row["l1_delta_deg"],
                wr=row["wrist_abs_delta_deg"],
            )
        )

    if args.apply:
        updated = _apply_to_yaml(candidate_data, selected)
        _save_yaml(CANDIDATES_YAML, updated)
        print(f"\nApplied optimized joints to: {CANDIDATES_YAML}")
    else:
        print("\nDry run only. Re-run with --apply to update the candidates YAML.")


if __name__ == "__main__":
    main()
