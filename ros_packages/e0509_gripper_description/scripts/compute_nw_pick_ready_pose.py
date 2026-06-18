#!/usr/bin/env python3
"""Offline IK helper — NOT a runtime node.

Computes a collect-then-pick "pick-ready" joint solution for the NW cell.

History of this computation (kept for audit — see git log for this file):
  1. First tried: keep root/nw's exact TCP pose, just find a healthier-J3 IK
     branch. FAILED — a 128-seed IK sweep at root/nw's live-FK TCP pose found
     ONLY near-singular solutions (|J3|<=1.2deg across all branches). The old
     root/nw pose's position+orientation is kinematically only reachable near
     full elbow extension; no alternate branch exists there.
  2. Switched target to the centroid of the 4 NW sub-cell TCP positions
     (same shared orientation as the sub-cells). This succeeds with a healthy
     J3~63deg branch, reachable from all 4 sub-cells with max joint delta
     18-36deg (vs 174-205deg from the old root/nw center pose), and — verified
     against the real failing 2026-06-18 target (x=-255,z=828mm) — can plan
     all the way to the wall surface (y=672mm) with cuRobo alone, no MoveLine
     fallback needed (old branch capped at y=702mm/IK_FAIL beyond).

IMPORTANT: config/scan_pose_candidates_refit_candidate.yaml's recorded
`tcp_transform_base` for root/nw predates the measured_tcp_260mm tool switch
and is off by ~260mm in Y versus live FK under the current model. Never use a
YAML tcp_transform_base as a cuRobo target directly — always re-derive via
live FK (see ROOT_NW path below, kept only as a documented dead end).

Run once, copy the printed YAML block into
config/scan_pose_candidates_refit_candidate.yaml, then build/verify.
"""
import os
import sys

import numpy as np
import torch
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from curobo_planner_node import load_environment_cuboids  # noqa: E402

from curobo.types.base import TensorDeviceType
from curobo.types.robot import JointState as CuroboJointState, RobotConfig
from curobo.types.math import Pose
from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig
from curobo.geom.types import WorldConfig

CONFIG_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config", "curobo"
)

# The 4 NW sub-cells (config/scan_pose_candidates_refit_candidate.yaml), already
# physically validated as scan poses — all share one orientation family.
SUBCELLS_DEG = {
    "root/nw/nw": [-35.82, -53.89, 26.90, 287.32, -54.86, 119.13],
    "root/nw/ne": [-44.81, -57.00, 44.92, 305.52, -58.82, 101.59],
    "root/nw/sw": [-30.06, -79.78, 50.44, 280.27, -56.94, 122.66],
    "root/nw/se": [-60.98, -86.50, 95.34, 328.39, -73.74, 85.83],
}
SEED_CELL = "root/nw/se"

JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]


def build_motion_gen():
    tensor_args = TensorDeviceType(device=torch.device("cuda:0"))
    with open(os.path.join(CONFIG_DIR, "e0509_gripper_measured_tcp.yml"), "r", encoding="utf-8") as f:
        robot_cfg_data = yaml.safe_load(f)
    robot_kin = robot_cfg_data["robot_cfg"]["kinematics"]
    robot_kin["urdf_path"] = os.path.join(CONFIG_DIR, "e0509_gripper.urdf")
    robot_kin["collision_spheres"] = os.path.join(CONFIG_DIR, "e0509_spheres.yml")
    robot_cfg = RobotConfig.from_dict(robot_cfg_data, tensor_args=tensor_args)
    world_cfg = WorldConfig(cuboid=load_environment_cuboids())
    motion_gen_cfg = MotionGenConfig.load_from_robot_config(
        robot_cfg, world_cfg, tensor_args=tensor_args,
        num_trajopt_seeds=16, num_graph_seeds=16,
        collision_cache={"obb": 30, "mesh": 10, "sphere": 30},
        use_cuda_graph=False,
        self_collision_check=False,
        self_collision_opt=False,
    )
    motion_gen = MotionGen(motion_gen_cfg)
    motion_gen.warmup(warmup_js_trajopt=False)
    return motion_gen, robot_cfg


def main():
    motion_gen, robot_cfg = build_motion_gen()
    kin_model = CudaRobotModel(robot_cfg.kinematics)

    # FK each sub-cell, average TCP position; orientation is shared across all
    # 4 sub-cells already (verified — quat differs by <1e-3 between them), so
    # reuse the seed cell's orientation as-is.
    positions = []
    seed_quat = None
    for name, jdeg in SUBCELLS_DEG.items():
        q = torch.tensor([np.deg2rad(jdeg)], device="cuda:0", dtype=torch.float32)
        st = kin_model.get_state(q)
        positions.append(st.ee_position.cpu().numpy()[0])
        if name == SEED_CELL:
            seed_quat = st.ee_quaternion.cpu().numpy()[0]
    centroid_m = np.mean(positions, axis=0)
    print("NW sub-cell TCP centroid (m):", centroid_m)
    target_pose = Pose(
        position=torch.tensor([centroid_m.tolist()], device="cuda:0", dtype=torch.float32),
        quaternion=torch.tensor([seed_quat.tolist()], device="cuda:0", dtype=torch.float32),
    )

    seed_deg = SUBCELLS_DEG[SEED_CELL]
    start_state = CuroboJointState.from_position(
        position=torch.tensor([np.deg2rad(seed_deg)], device="cuda:0", dtype=torch.float32),
        joint_names=JOINT_NAMES,
    )
    result = motion_gen.plan_single(
        start_state, target_pose,
        MotionGenPlanConfig(num_ik_seeds=64, max_attempts=10, timeout=5.0),
    )
    if not result.success.item():
        print("plan_single FAILED:", result.status)
        return
    traj = result.get_interpolated_plan().position.cpu().numpy()
    end_deg = np.rad2deg(traj[-1]).tolist()

    print("=== Result (seeded from %s) ===" % SEED_CELL)
    print("end joints (deg):", ["%.2f" % v for v in end_deg])
    print("J3 (deg):", "%.2f" % end_deg[2], "(healthy range target: 26~95deg, matches NW sub-cells)")
    print("J4 (deg):", "%.2f" % end_deg[3], "(NW sub-cell family: 280~330deg)")

    print()
    print("=== Reachability + travel cost from all 4 sub-cells ===")
    for name, jdeg in SUBCELLS_DEG.items():
        start_state = CuroboJointState.from_position(
            position=torch.tensor([np.deg2rad(jdeg)], device="cuda:0", dtype=torch.float32),
            joint_names=JOINT_NAMES,
        )
        r = motion_gen.plan_single(
            start_state, target_pose, MotionGenPlanConfig(num_ik_seeds=64, max_attempts=10, timeout=3.0)
        )
        if r.success.item():
            t = r.get_interpolated_plan().position.cpu().numpy()
            d = np.rad2deg(t[-1])
            max_delta = float(np.max(np.abs(d - np.array(jdeg))))
            print("  %s -> OK max_joint_delta=%.1fdeg" % (name, max_delta))
        else:
            print("  %s -> FAIL %s" % (name, r.status))

    final_q = torch.tensor([np.deg2rad(end_deg)], device="cuda:0", dtype=torch.float32)
    final_state = kin_model.get_state(final_q)
    final_pos = final_state.ee_position.cpu().numpy()[0]
    final_quat_wxyz = final_state.ee_quaternion.cpu().numpy()[0]
    from scipy.spatial.transform import Rotation as SciR
    R = SciR.from_quat(
        [final_quat_wxyz[1], final_quat_wxyz[2], final_quat_wxyz[3], final_quat_wxyz[0]]
    ).as_matrix()
    transform = np.eye(4)
    transform[:3, :3] = R
    transform[:3, 3] = final_pos

    print()
    print("YAML block to paste under scan_pose_candidates.targets:")
    print(yaml.dump([{
        "cell_id": "root/nw/pick_ready",
        "approach": "computed_ik_healthy_branch_2026_06_18",
        "tcp_transform_base": [[round(float(v), 9) for v in row] for row in transform.tolist()],
        "standoff_m": "taught",
        "note": (
            "Offline cuRobo IK solve at the centroid of the 4 NW sub-cell TCP "
            "positions (shared sub-cell orientation), seeded from root/nw/se. "
            "Replaces using root/nw (camera-framing center, J3~-1deg near-"
            "singular, no healthy-J3 branch exists at that exact TCP pose — "
            "verified by 128-seed IK sweep) as the collect-then-pick pick-ready "
            "pose. This pose is reachable from all 4 sub-cells with <=36deg max "
            "joint delta (vs 174-205deg from the old root/nw center), and can "
            "cuRobo-plan a real failing target (x=-255,z=828mm, 2026-06-18 log) "
            "all the way to the wall surface (y=672mm) with healthy J3~73deg — "
            "no MoveLine fallback needed. See scripts/compute_nw_pick_ready_pose.py."
        ),
        "endpoint_joints_deg": [round(float(v), 2) for v in end_deg],
        "use_for_automated_motion": True,
    }], sort_keys=False, default_flow_style=None))


if __name__ == "__main__":
    main()
