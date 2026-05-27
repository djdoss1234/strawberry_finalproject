#!/usr/bin/env python3
"""Compute v6 scan pose candidates with corrected gripper_rh TCP rotation.

Root cause of v5 failure:
  v5 set R_TCP_NEW = FK_calib(overview), where the 'calib' frame is
  link_6 @ Rz(π) @ Ry(-π/2) (calibration TCP_fixed).
  But cuRobo plans for gripper_rh_p12_rn_base = link_6 @ Rz(π/2)
  (gripper_attach_joint, rpy="0 0 1.5708").
  These two frames differ by a significant rotation, causing the IK to
  place the gripper at a completely different physical orientation than
  intended, so the camera faced ~-X instead of +Y.

v6 fix:
  R_TCP_NEW_V6 = FK_gripper_rh_p12_rn_base(overview_joints)
  This is the actual URDF FK of the ee_link that cuRobo plans for.
  Camera Z in base = [0.039, 0.998, -0.043] (faces panel +Y). ✓

Run from project root:
  python3 scripts/compute_v6_scan_poses.py
  python3 scripts/compute_v6_scan_poses.py --dry-run
"""

import argparse
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import torch

PROJECT_ROOT = Path(__file__).parent.parent
CUROBO_DIR = Path("/home/user/doosan_ws/src/e0509_gripper_description/config/curobo")
URDF_PATH = CUROBO_DIR / "e0509_gripper.urdf"

_Q_XYZW  = [0.701914, -0.099087, -0.085966, 0.700077]
_T_PANEL = [0.112833, 0.597764, 0.643978]

_CALIBRATION_NPZ = Path(
    "/home/user/doosan_ws/src/e0509_gripper_description/config/"
    "calibration_eye_in_hand_1.npz"
)

OVERVIEW_JOINTS_DEG = [97.84, -94.40, 65.95, -10.93, 95.49, -94.79]


# ── URDF FK for gripper_rh_p12_rn_base ────────────────────────────────────────

def _Rz(q):
    c, s = np.cos(q), np.sin(q)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _make_T(xyz, rpy, q=0.0):
    R = Rotation.from_euler('xyz', rpy).as_matrix() @ _Rz(q)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = xyz
    return T


def fk_gripper_rh(q_deg):
    """FK of gripper_rh_p12_rn_base in base_link (the cuRobo ee_link)."""
    q = np.deg2rad(q_deg)
    T = np.eye(4)
    T = T @ _make_T([0, 0, 0.2045],   [0, 0, 0],                q[0])
    T = T @ _make_T([0, 0, 0],        [0, -np.pi/2, -np.pi/2],  q[1])
    T = T @ _make_T([0.373, 0, 0],    [0, 0, np.pi/2],           q[2])
    T = T @ _make_T([0, -0.373, 0],   [np.pi/2, 0, 0],           q[3])
    T = T @ _make_T([0, 0, 0],        [-np.pi/2, 0, 0],          q[4])
    T = T @ _make_T([0, -0.1725, 0],  [np.pi/2, 0, 0],           q[5])
    T = T @ _make_T([0, 0, 0],        [0, 0, np.pi/2])           # gripper_attach_joint
    return T


def _derive_R_tcp():
    """Derive R_TCP_NEW from actual overview FK of gripper_rh_p12_rn_base."""
    return fk_gripper_rh(OVERVIEW_JOINTS_DEG)[:3, :3]


# ── Compute TCP transforms ─────────────────────────────────────────────────────

def _build_panel_transform():
    R = Rotation.from_quat(_Q_XYZW).as_matrix()
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = _T_PANEL
    return T


def _derive_t_cam_in_rh():
    """Camera origin position in gripper_rh_p12_rn_base frame."""
    data = np.load(_CALIBRATION_NPZ)
    t_cc = data['T_cam_to_gripper'][:3, 3]
    # gripper_rh = link_6 @ Rz(π/2); calib = link_6 @ Rx(π)@Ry(-π/2)
    # T_cam_to_rh = inv(Rz(π/2)) @ Rx(π)@Ry(-π/2) @ T_cam_to_calib
    M = (Rotation.from_euler('xyz', [0, 0, -np.pi/2]).as_matrix() @
         Rotation.from_euler('xyz', [np.pi, -np.pi/2, 0]).as_matrix())
    return M @ t_cc


_CELL_PANEL = {
    "root/nw": np.array([-0.2725,  0.1975, 0.0]),
    "root/ne": np.array([ 0.2775,  0.1975, 0.0]),
    "root/sw": np.array([-0.2725, -0.2025, 0.0]),
    "root/se": np.array([ 0.2775, -0.2025, 0.0]),
}

SCAN_ORDER = ["root/nw", "root/ne", "root/sw", "root/se"]


def _compute_tcp_mat4(cell_id, standoff, approach, T_panel, R_tcp, t_cam_in_rh):
    panel_Z_base = T_panel[:3, :3][:, 2]
    cell_base = (T_panel @ np.append(_CELL_PANEL[cell_id], 1.0))[:3]

    if approach == "panel_normal":
        cam_pos = cell_base + standoff * panel_Z_base
    elif approach == "base_neg_y":
        cam_pos = cell_base + standoff * np.array([0.0, -1.0, 0.0])
    else:
        raise ValueError("Unknown approach: %s" % approach)

    tcp_pos = cam_pos - R_tcp @ t_cam_in_rh

    T = np.eye(4)
    T[:3, :3] = R_tcp
    T[:3, 3] = tcp_pos
    return T


def _mat4_to_pos_quat_wxyz(mat4):
    pos = mat4[:3, 3].tolist()
    q_xyzw = Rotation.from_matrix(mat4[:3, :3]).as_quat()
    q_wxyz = [float(q_xyzw[3]), float(q_xyzw[0]), float(q_xyzw[1]), float(q_xyzw[2])]
    return pos, q_wxyz


# ── cuRobo dry-run ─────────────────────────────────────────────────────────────

def run_curobo_dryrun(candidates):
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
        print("    [%s] %-12s  %-14s  %-28s  (%.2fs)" % (
            marker, cell_id, approach, label, elapsed))
        results[cell_id] = (label, success, status, round(elapsed, 3))
    return results


# ── main ───────────────────────────────────────────────────────────────────────

def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--dry-run", action="store_true",
                        help="run cuRobo validation (GPU required)")
    parser.add_argument("--standoff", type=float, default=0.65)
    args = parser.parse_args(argv)

    R_tcp = _derive_R_tcp()
    t_cam_in_rh = _derive_t_cam_in_rh()
    T_panel = _build_panel_transform()

    # Load calibration frame correction for verification
    data = np.load(_CALIBRATION_NPZ)
    M = (Rotation.from_euler('xyz', [0, 0, -np.pi/2]).as_matrix() @
         Rotation.from_euler('xyz', [np.pi, -np.pi/2, 0]).as_matrix())
    R_cam_to_rh = M @ data['T_cam_to_gripper'][:3, :3]
    cam_Z_base = R_tcp @ R_cam_to_rh @ [0, 0, 1]

    print("=== v6 candidate computation ===")
    print("R_tcp orthogonal:", np.allclose(R_tcp @ R_tcp.T, np.eye(3), atol=1e-5))
    print("Camera Z in base (expect ~+Y):", np.round(cam_Z_base, 4))
    print("Source: FK(gripper_rh_p12_rn_base, overview_joints)")
    print()

    candidates = []
    for cell_id in SCAN_ORDER:
        approach = "panel_normal"
        standoff = args.standoff
        mat4 = _compute_tcp_mat4(cell_id, standoff, approach, T_panel, R_tcp, t_cam_in_rh)
        pos = mat4[:3, 3]
        print("  %s  %s  standoff=%.2fm  tcp_pos=%s" % (
            cell_id, approach, standoff, np.round(pos, 4)))
        candidates.append((cell_id, mat4, approach, standoff))

    if args.dry_run:
        print()
        print("=== cuRobo dry-run ===")
        print("  URDF:", URDF_PATH)
        print("  ee_link: gripper_rh_p12_rn_base  (from_basic, empty world)")
        print("  start joints (deg):", OVERVIEW_JOINTS_DEG)
        print("  torch.cuda.is_available() =", torch.cuda.is_available())
        results = run_curobo_dryrun(candidates)

        # Retry any panel_normal failures with base_neg_y 0.40m
        for retry_id in ["root/sw", "root/se", "root/nw", "root/ne"]:
            lbl, ok, _, _ = results.get(retry_id, ("?", False, "?", 0))
            if not ok:
                print()
                print("  %s panel_normal failed — retrying base_neg_y 0.40m" % retry_id)
                alt_mat4 = _compute_tcp_mat4(retry_id, 0.40, "base_neg_y",
                                             T_panel, R_tcp, t_cam_in_rh)
                alt_res = run_curobo_dryrun([(retry_id, alt_mat4, "base_neg_y", 0.40)])
                alt_lbl, alt_ok, _, _ = alt_res[retry_id]
                if alt_ok:
                    candidates = [c for c in candidates if c[0] != retry_id]
                    candidates.append((retry_id, alt_mat4, "base_neg_y", 0.40))
                    results[retry_id] = alt_res[retry_id]

        print()
        print("=== Summary ===")
        for cell_id, mat4, approach, standoff in candidates:
            label = results[cell_id][0]
            print("  %-12s  %-14s  %s" % (cell_id, approach, label))

        print()
        print("=== YAML targets block ===")
        for cell_id, mat4, approach, standoff in candidates:
            label, success, status, plan_time = results[cell_id]
            rows = mat4.tolist()
            print("    - cell_id: %s" % cell_id)
            print("      approach: %s" % approach)
            print("      standoff_m: %.2f" % standoff)
            print("      curobo_status: %s" % label)
            if not success:
                print("      curobo_raw_status: %s" % status)
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
