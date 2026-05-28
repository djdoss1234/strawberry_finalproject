#!/usr/bin/env python3
"""Re-derive all 4 scan poses using camera-centered positioning.

Current poses use:  cam_pos = cell_center + panel_normal * standoff
  → camera is NOT looking at cell center (22° upward tilt causes offset)

This script uses: cam_pos = cell_center - cam_dir * standoff
  → camera center ray passes exactly through cell center at distance standoff

Run from project root:
  python3 scripts/search_all_cells_camera_centered.py
"""

from copy import deepcopy
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation
import torch
import yaml

PROJECT_ROOT = Path(__file__).resolve().parent.parent
CUROBO_DIR   = Path("/home/user/doosan_ws/src/e0509_gripper_description/config/curobo")
ROBOT_YML    = CUROBO_DIR / "e0509_gripper.yml"
URDF_PATH    = CUROBO_DIR / "e0509_gripper.urdf"
SPHERES_PATH = CUROBO_DIR / "e0509_spheres.yml"
WORLD_PATH   = PROJECT_ROOT / "config" / "scan_collision_world.yaml"
REG_PATH     = PROJECT_ROOT / "config" / "panel_registration.yaml"
CAL_PATH     = Path("/home/user/doosan_ws/src/e0509_gripper_description/config/calibration_eye_in_hand_1.npz")

OVERVIEW_DEG  = [97.84, -94.40, 65.95, -10.93, 95.49, -94.79]
JOINT_NAMES   = ["joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"]
OP_LIMITS_DEG = [(-225,225),(-95,95),(-155,155),(-170,170),(-130,130),(-225,225)]

# Cell centres in cultivation_panel frame  [X, Y, Z]
# Panel X≈+X_base (right), Panel Y≈+Z_base (up), Panel Z≈-Y_base (toward robot)
CELLS = {
    "nw": np.array([-0.2725,  0.2025, 0.0]),
    "ne": np.array([ 0.2725,  0.2025, 0.0]),
    "sw": np.array([-0.2725, -0.2025, 0.0]),
    "se": np.array([ 0.2725, -0.2025, 0.0]),
}

STANDOFFS = [0.25, 0.30, 0.35, 0.40, 0.45]


# ── FK ────────────────────────────────────────────────────────────────────────

def _Rz(q):
    c, s = np.cos(q), np.sin(q)
    return np.array([[c,-s,0],[s,c,0],[0,0,1]])

def _make_T(xyz, rpy, q=0.0):
    R = Rotation.from_euler("xyz", rpy).as_matrix() @ _Rz(q)
    T = np.eye(4); T[:3,:3] = R; T[:3,3] = xyz
    return T

def fk_gripper_rh(q_deg):
    q = np.deg2rad(q_deg)
    T = np.eye(4)
    T = T @ _make_T([0,0,0.2045],  [0,0,0],               q[0])
    T = T @ _make_T([0,0,0],       [0,-np.pi/2,-np.pi/2],  q[1])
    T = T @ _make_T([0.373,0,0],   [0,0,np.pi/2],          q[2])
    T = T @ _make_T([0,-0.373,0],  [np.pi/2,0,0],          q[3])
    T = T @ _make_T([0,0,0],       [-np.pi/2,0,0],         q[4])
    T = T @ _make_T([0,-0.1725,0], [np.pi/2,0,0],          q[5])
    T = T @ _make_T([0,0,0],       [0,0,np.pi/2])
    return T


# ── helpers ───────────────────────────────────────────────────────────────────

def _load(p):
    with p.open(encoding="utf-8") as f:
        return yaml.safe_load(f)

def _panel_T():
    t = _load(REG_PATH)["panel_registration"]["transform"]
    tr, ro = t["translation_m"], t["rotation_xyzw"]
    R = Rotation.from_quat([ro["x"],ro["y"],ro["z"],ro["w"]]).as_matrix()
    T = np.eye(4); T[:3,:3] = R; T[:3,3] = [tr["x"],tr["y"],tr["z"]]
    return T

def _cam_offset_in_rh():
    data = np.load(CAL_PATH)
    t = data["T_cam_to_gripper"][:3, 3]
    M = (Rotation.from_euler("xyz",[0,0,-np.pi/2]).as_matrix()
         @ Rotation.from_euler("xyz",[np.pi,-np.pi/2,0]).as_matrix())
    return M @ t

def _build_tcp_mat4_camera_centered(cell_base, standoff, R_tcp, t_cam_in_rh):
    """Place camera so its center ray hits cell_base at distance standoff."""
    cam_dir = R_tcp[:, 2]                    # TCP Z-axis = camera viewing direction
    cam_pos = cell_base - standoff * cam_dir  # camera at standoff, looking at cell_base
    tcp_pos = cam_pos - R_tcp @ t_cam_in_rh  # TCP from camera offset
    T = np.eye(4); T[:3,:3] = R_tcp; T[:3,3] = tcp_pos
    return T, tcp_pos, cam_pos

def _mat4_to_pose(mat4):
    q = Rotation.from_matrix(mat4[:3,:3]).as_quat()  # xyzw
    return mat4[:3,3].tolist(), [float(q[3]),float(q[0]),float(q[1]),float(q[2])]

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
    cuboids = [Cuboid(name=o["name"],
                      pose=[float(v) for v in o["pose_wxyz"]],
                      dims=[float(v) for v in o["dims_m"]])
               for o in world_meta["objects"]
               if o.get("enabled",True) and o.get("type")=="cuboid"]
    wc = WorldConfig(cuboid=cuboids)
    mc = MotionGenConfig.load_from_robot_config(
        rc, wc, tensor_args=ta,
        num_trajopt_seeds=16, num_graph_seeds=16,
        collision_cache={"obb":30,"mesh":10},
        use_cuda_graph=False,
        self_collision_check=world_meta["self_collision_check_enabled"],
        self_collision_opt=world_meta["self_collision_check_enabled"],
    )
    mg = MotionGen(mc)
    mg.warmup(warmup_js_trajopt=False)
    mg.detach_object_from_robot()
    return mg, ta

def _try(mg, ta, mat4, n_retry=5):
    from curobo.types.math import Pose
    from curobo.types.robot import JointState as CJS
    from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig

    pos, quat = _mat4_to_pose(mat4)
    start = CJS.from_position(
        position=torch.tensor([np.deg2rad(OVERVIEW_DEG).tolist()], dtype=torch.float32, device=ta.device),
        joint_names=JOINT_NAMES)
    goal = Pose(
        position=torch.tensor([pos], dtype=torch.float32, device=ta.device),
        quaternion=torch.tensor([quat], dtype=torch.float32, device=ta.device))

    t0 = time.time()
    best_fail = None
    for _ in range(n_retry):
        r = mg.plan_single(start, goal, MotionGenPlanConfig(enable_graph=True, max_attempts=4))
        if not r.success.item():
            best_fail = (False, "IK_FAIL", round(time.time()-t0,2), None, None)
            continue
        traj = r.get_interpolated_plan().position.cpu().numpy()
        deg  = np.rad2deg(traj)
        ep   = [round(float(d),1) for d in deg[-1]]
        jr   = {}
        ok   = True
        for j,(lo,hi) in enumerate(OP_LIMITS_DEG):
            vmin,vmax = float(np.min(deg[:,j])), float(np.max(deg[:,j]))
            jr[f"J{j+1}"] = [round(vmin,1), round(vmax,1)]
            if vmin<lo or vmax>hi: ok=False
        if ok:
            return True, "PLAN_VALID", round(time.time()-t0,2), jr, ep
        best_fail = (False, "JOINT_LIMIT_REJECT", round(time.time()-t0,2), jr, ep)

    return best_fail or (False,"PLAN_FAIL",0,None,None)


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    T_panel     = _panel_T()
    R_tcp       = fk_gripper_rh(OVERVIEW_DEG)[:3,:3]
    t_cam       = _cam_offset_in_rh()
    world_meta  = _load(WORLD_PATH)["scan_collision_world"]

    cam_dir = R_tcp[:, 2]
    print(f"Camera direction (base): {np.round(cam_dir, 3)}")
    print(f"  → tilt from horizontal: {np.degrees(np.arcsin(cam_dir[2])):.1f}° upward")
    print(f"  → horizontal (Y) component: {cam_dir[1]:.3f}\n")

    # Print cell centers in base
    print("Cell centers in base frame:")
    cell_base = {}
    for name, pf in CELLS.items():
        cb = (T_panel @ np.append(pf,1.0))[:3]
        cell_base[name] = cb
        print(f"  {name.upper()}: [{cb[0]:+.3f}, {cb[1]:+.3f}, {cb[2]:+.3f}]")
    print()

    print("Loading cuRobo …")
    mg, ta = _build_mg(world_meta)
    print("Ready.\n")

    print(f"{'':3s} {'cell':4s} {'standoff':8s} {'tcp_pos':38s}  {'result':14s}  time  J1sw  J6range")
    print("-"*110)

    results = {n: [] for n in CELLS}

    for standoff in STANDOFFS:
        print(f"\n── standoff = {standoff:.2f}m ──")
        for cell_name, cb in cell_base.items():
            mat4, tcp, cam_pos = _build_tcp_mat4_camera_centered(cb, standoff, R_tcp, t_cam)
            ok, status, elapsed, jr, ep = _try(mg, ta, mat4)
            marker = "OK" if ok else "!!"
            j1sw = abs(ep[0]-OVERVIEW_DEG[0]) if ep else 0
            j6r  = jr["J6"] if jr else ["?","?"]
            print(f"[{marker}] {cell_name.upper():4s} {standoff:.2f}m  "
                  f"tcp=[{tcp[0]:+.3f},{tcp[1]:+.3f},{tcp[2]:+.3f}]  "
                  f"{status:14s}  {elapsed:.1f}s  "
                  f"J1Δ={j1sw:.0f}°  J6=[{j6r[0]},{j6r[1]}]")
            if ep:
                print(f"       end: [{', '.join(f'{v:+.1f}' for v in ep)}]")
            results[cell_name].append({
                "standoff_m": standoff,
                "success": ok,
                "status": status,
                "tcp_pos": [round(float(v),6) for v in tcp],
                "cam_pos": [round(float(v),6) for v in cam_pos],
                "tcp_transform_base": [list(r) for r in mat4[:3].tolist()],
                "endpoint_joints_deg": ep,
                "joint_range_deg": jr,
                "elapsed_s": elapsed,
            })

    # Summary: best (largest valid standoff) per cell
    print("\n" + "="*60)
    print("BEST VALID CANDIDATES")
    print("="*60)
    best = {}
    for cell_name, rows in results.items():
        valid = [r for r in rows if r["success"]]
        if valid:
            b = valid[-1]  # largest valid standoff
            best[cell_name] = b
            j1sw = abs(b["endpoint_joints_deg"][0]-OVERVIEW_DEG[0])
            print(f"  {cell_name.upper()}: standoff={b['standoff_m']}m  "
                  f"tcp_y={b['tcp_pos'][1]:.3f}  J1Δ={j1sw:.0f}°  "
                  f"J6={b['joint_range_deg']['J6']}")
            print(f"       end={b['endpoint_joints_deg']}")
        else:
            print(f"  {cell_name.upper()}: NO VALID CANDIDATE")

    # Save
    out = PROJECT_ROOT/"docs"/"runs"/"RUN-20260528-005_camera_centered_search.yaml"
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w",encoding="utf-8") as f:
        yaml.safe_dump({
            "camera_centered_search": {
                "run_date": "2026-05-28",
                "description": "cam_pos = cell_center - standoff*cam_dir  (camera aimed at cell center)",
                "overview_joints_deg": OVERVIEW_DEG,
                "cam_dir_base": [round(float(v),4) for v in cam_dir],
                "results": {n: v for n,v in results.items()},
            }
        }, f, sort_keys=False, allow_unicode=True)
    print(f"\nSaved: {out}")
    return best


if __name__ == "__main__":
    main()
