#!/usr/bin/env python3
"""Compute v5 scan pose candidates using overview-based camera orientation.

All cells use R_tcp_new (derived from actual overview joint pose + hand-eye
calibration). Position is computed as: cell_center_base + standoff * panel_Z.
SW is first attempted at panel_normal 0.65 m; base_neg_y at 0.40 m is tried
as fallback if cuRobo dry-run fails.

Run from project root:
  python3 scripts/compute_v5_scan_poses.py
  python3 scripts/compute_v5_scan_poses.py --dry-run
"""

import argparse
import sys
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import torch
import yaml

PROJECT_ROOT = Path(__file__).parent.parent
CUROBO_DIR = Path("/home/user/doosan_ws/src/e0509_gripper_description/config/curobo")
URDF_PATH = CUROBO_DIR / "e0509_gripper.urdf"

# Panel TF (from panel_registration.yaml static_transform_publisher)
_Q_XYZW  = [0.701914, -0.099087, -0.085966, 0.700077]
_T_PANEL = [0.112833, 0.597764, 0.643978]

# Hand-eye calibration
_T_CAM_TO_GRIPPER = np.array([
    [ 0.033,  -0.42,    0.9069,  0.0675],
    [ 0.0145, -0.9071, -0.4206,  0.0774],
    [ 0.9994,  0.027,  -0.0238,  0.0119],
    [ 0.,      0.,      0.,      1.    ]
])

# Correct TCP rotation — derived from actual overview pose + hand-eye calibration.
# Camera Z in base = [0.039, 0.998, -0.043] (points toward panel ~+Y). ✓
R_TCP_NEW = np.array([
    [ 0.063540625, -0.011981230,  0.997907330],
    [ 0.922188133, -0.381526086, -0.063300028],
    [ 0.381486090,  0.924280420, -0.013193458]
])

# v5 dry-run start: same overview joints used in v4 dry-run
OVERVIEW_JOINTS_DEG = [97.84, -94.40, 65.95, -10.93, 95.49, -94.79]

# Cell centres in panel frame (from workspace.yaml measurements)
_CELL_PANEL = {
    "root/nw": np.array([-0.2725,  0.1975, 0.0]),
    "root/ne": np.array([ 0.2775,  0.1975, 0.0]),
    "root/sw": np.array([-0.2725, -0.2025, 0.0]),
    "root/se": np.array([ 0.2775, -0.2025, 0.0]),
}

SCAN_ORDER = ["root/nw", "root/ne", "root/sw", "root/se"]


def _build_panel_transform():
    R = Rotation.from_quat(_Q_XYZW).as_matrix()
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = _T_PANEL
    return T


def _compute_tcp_mat4(cell_id: str, standoff_m: float,
                      approach: str, T_panel: np.ndarray) -> np.ndarray:
    """Return 4x4 TCP transform in base_link for the given cell/standoff/approach."""
    camera_to_tcp = np.linalg.inv(_T_CAM_TO_GRIPPER)
    R_cam_overview = R_TCP_NEW @ _T_CAM_TO_GRIPPER[:3, :3]
    t_cam_in_tcp = camera_to_tcp[:3, 3]
    panel_Z_base = T_panel[:3, :3][:, 2]   # panel normal toward robot

    cell_base = (T_panel @ np.append(_CELL_PANEL[cell_id], 1.0))[:3]

    if approach == "panel_normal":
        cam_pos = cell_base + standoff_m * panel_Z_base
    elif approach == "base_neg_y":
        cam_pos = cell_base + standoff_m * np.array([0.0, -1.0, 0.0])
    else:
        raise ValueError("Unknown approach: %s" % approach)

    tcp_pos = cam_pos + R_cam_overview @ t_cam_in_tcp

    T = np.eye(4)
    T[:3, :3] = R_TCP_NEW
    T[:3, 3] = tcp_pos
    return T


def _mat4_to_pos_quat_wxyz(mat4):
    pos = mat4[:3, 3].tolist()
    q_xyzw = Rotation.from_matrix(mat4[:3, :3]).as_quat()
    q_wxyz = [float(q_xyzw[3]), float(q_xyzw[0]), float(q_xyzw[1]), float(q_xyzw[2])]
    return pos, q_wxyz


def run_curobo_dryrun(candidates):
    """Return dict of {cell_id: (label, success, status, plan_time)}."""
    from curobo.types.base import TensorDeviceType
    from curobo.types.robot import JointState as CuroboJS, RobotConfig
    from curobo.types.math import Pose
    from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig
    from curobo.geom.types import WorldConfig

    tensor_args = TensorDeviceType()
    robot_cfg = RobotConfig.from_basic(
        urdf_path=str(URDF_PATH),
        base_link="base_link",
        ee_link="gripper_rh_p12_rn_base",
        tensor_args=tensor_args,
    )
    world_cfg = WorldConfig(cuboid=[])

    print("  Loading cuRobo MotionGen …")
    t0 = time.time()
    mg_cfg = MotionGenConfig.load_from_robot_config(
        robot_cfg, world_cfg, tensor_args=tensor_args,
        num_trajopt_seeds=16, num_graph_seeds=16,
        collision_cache={"obb": 10, "mesh": 5},
        use_cuda_graph=False,
    )
    mg = MotionGen(mg_cfg)
    mg.warmup()
    print("  MotionGen ready in %.1fs" % (time.time() - t0))

    q_start_rad = np.deg2rad(OVERVIEW_JOINTS_DEG).tolist()
    start_state = CuroboJS.from_position(
        torch.tensor([q_start_rad], dtype=torch.float32, device=tensor_args.device)
    )
    plan_cfg = MotionGenPlanConfig(enable_graph=True, max_attempts=4)

    results = {}
    for cell_id, mat4, approach, standoff in candidates:
        pos, q_wxyz = _mat4_to_pos_quat_wxyz(mat4)
        goal = Pose(
            position=tensor_args.to_device(torch.tensor([pos], dtype=torch.float32)),
            quaternion=tensor_args.to_device(torch.tensor([q_wxyz], dtype=torch.float32)),
        )
        t1 = time.time()
        result = mg.plan_single(start_state, goal, plan_cfg)
        elapsed = time.time() - t1

        success = bool(result.success.item())
        status = result.status.name if hasattr(result.status, "name") else str(result.status)
        label = "PLAN_VALID" if success else (
            "IK_FAIL" if "IK_FAIL" in status.upper() else "PLAN_FAIL"
        )
        marker = "OK" if success else "!!"
        print("    [%s] %-12s  %-12s  %-28s  (%.2fs)" % (
            marker, cell_id, approach, label, elapsed))
        results[cell_id] = (label, success, status, round(elapsed, 3))
    return results


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--dry-run", action="store_true",
                        help="run cuRobo validation (GPU required)")
    parser.add_argument("--standoff", type=float, default=0.65)
    args = parser.parse_args(argv)

    T_panel = _build_panel_transform()

    print("=== v5 candidate computation ===")
    print("TCP rotation (R_tcp_new) orthogonal:", np.allclose(
        R_TCP_NEW @ R_TCP_NEW.T, np.eye(3), atol=1e-5))
    R_cam = R_TCP_NEW @ _T_CAM_TO_GRIPPER[:3, :3]
    print("Camera Z in base (expect ~+Y):", np.round(R_cam[:, 2], 4))
    print()

    candidates = []
    for cell_id in SCAN_ORDER:
        approach = "panel_normal"
        standoff = args.standoff
        mat4 = _compute_tcp_mat4(cell_id, standoff, approach, T_panel)
        pos = mat4[:3, 3]
        print("  %s  %s  standoff=%.2fm  tcp_pos=%s" % (
            cell_id, approach, standoff, np.round(pos, 4)))
        candidates.append((cell_id, mat4, approach, standoff))

    if args.dry_run:
        print()
        print("=== cuRobo dry-run ===")
        print("  URDF:", URDF_PATH)
        print("  start joints (deg):", OVERVIEW_JOINTS_DEG)
        print("  torch.cuda.is_available() =", torch.cuda.is_available())
        results = run_curobo_dryrun(candidates)

        # SW fallback: if panel_normal fails, retry base_neg_y at 0.40m
        sw_label, sw_ok, _, _ = results.get("root/sw", ("?", False, "?", 0))
        if not sw_ok:
            print()
            print("  SW panel_normal failed — retrying base_neg_y 0.40m")
            sw_mat4_alt = _compute_tcp_mat4("root/sw", 0.40, "base_neg_y", T_panel)
            alt_candidates = [("root/sw", sw_mat4_alt, "base_neg_y", 0.40)]
            alt_results = run_curobo_dryrun(alt_candidates)
            sw_label_alt, sw_ok_alt, _, _ = alt_results["root/sw"]
            if sw_ok_alt:
                # Replace SW in candidates
                candidates = [c for c in candidates if c[0] != "root/sw"]
                candidates.append(("root/sw", sw_mat4_alt, "base_neg_y", 0.40))
                results["root/sw"] = alt_results["root/sw"]
                print("  SW base_neg_y 0.40m: PLAN_VALID")

        print()
        print("=== Summary ===")
        for cell_id, mat4, approach, standoff in candidates:
            label = results[cell_id][0]
            print("  %-12s  %-14s  %s" % (cell_id, approach, label))

        # Emit YAML block
        print()
        print("=== YAML targets block (paste into scan_pose_candidates.yaml) ===")
        for cell_id, mat4, approach, standoff in candidates:
            label, success, status, plan_time = results[cell_id]
            rows = mat4.tolist()
            print("    - cell_id: %s" % cell_id)
            print("      approach: %s" % approach)
            print("      standoff_m: %.2f" % standoff)
            print("      curobo_status: %s" % label)
            if not success:
                print("      curobo_raw_status: %s" % status)
            for row in rows:
                vals = ", ".join("%.9f" % v for v in row)
                print("      # [%s]" % vals)
            print("      tcp_transform_base:")
            for row in rows[:3]:
                inner = "[" + ", ".join("%.9f" % v for v in row) + "]"
                print("        - %s" % inner)
            print("        - [0.0, 0.0, 0.0, 1.0]")
    else:
        print()
        print("Rerun with --dry-run to validate with cuRobo (GPU required).")


if __name__ == "__main__":
    main()
