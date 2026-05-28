#!/usr/bin/env python3
"""Search for a valid root/se scan pose under the board-shift panel registration.

Run from project root:
  python3 scripts/search_se_candidates.py
"""

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
URDF_PATH      = BASELINE_CUROBO_DIR / "e0509_gripper.urdf"
SPHERES_PATH   = BASELINE_CUROBO_DIR / "e0509_spheres.yml"
WORLD_PATH     = PROJECT_ROOT / "config" / "scan_collision_world.yaml"
REG_PATH       = PROJECT_ROOT / "config" / "panel_registration.yaml"
CAL_PATH       = Path(
    "/home/user/doosan_ws/src/e0509_gripper_description/config/"
    "calibration_eye_in_hand_1.npz"
)

OVERVIEW_JOINTS_DEG = [97.84, -94.40, 65.95, -10.93, 95.49, -94.79]
JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
OP_LIMITS_DEG = [
    (-225.0, 225.0),
    (-95.0,   95.0),
    (-155.0, 155.0),
    (-170.0, 170.0),
    (-130.0, 130.0),
    (-225.0, 225.0),
]

# SE cell centre in cultivation_panel frame
_SE_CELL_PANEL = np.array([0.2725, -0.2025, 0.0])


def _Rz(q):
    c, s = np.cos(q), np.sin(q)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _make_T(xyz, rpy, q=0.0):
    R = Rotation.from_euler("xyz", rpy).as_matrix() @ _Rz(q)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = xyz
    return T


def fk_gripper_rh(q_deg):
    q = np.deg2rad(q_deg)
    T = np.eye(4)
    T = T @ _make_T([0, 0, 0.2045],  [0, 0, 0],                q[0])
    T = T @ _make_T([0, 0, 0],       [0, -np.pi/2, -np.pi/2],  q[1])
    T = T @ _make_T([0.373, 0, 0],   [0, 0, np.pi/2],           q[2])
    T = T @ _make_T([0, -0.373, 0],  [np.pi/2, 0, 0],           q[3])
    T = T @ _make_T([0, 0, 0],       [-np.pi/2, 0, 0],          q[4])
    T = T @ _make_T([0, -0.1725, 0], [np.pi/2, 0, 0],           q[5])
    T = T @ _make_T([0, 0, 0],       [0, 0, np.pi/2])
    return T


def _load_yaml(path):
    with path.open(encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def _build_panel_T(reg_path):
    reg = _load_yaml(reg_path)["panel_registration"]["transform"]
    t = reg["translation_m"]
    r = reg["rotation_xyzw"]
    R = Rotation.from_quat([r["x"], r["y"], r["z"], r["w"]]).as_matrix()
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [t["x"], t["y"], t["z"]]
    return T


def _cam_offset_in_rh():
    data = np.load(CAL_PATH)
    t_cc = data["T_cam_to_gripper"][:3, 3]
    M = (Rotation.from_euler("xyz", [0, 0, -np.pi / 2]).as_matrix()
         @ Rotation.from_euler("xyz", [np.pi, -np.pi / 2, 0]).as_matrix())
    return M @ t_cc


def _tcp_pos(cam_pos, R_tcp, t_cam_in_rh):
    return cam_pos - R_tcp @ t_cam_in_rh


def _build_tcp_mat4(approach, standoff, T_panel, R_tcp, t_cam_in_rh):
    panel_Z = T_panel[:3, :3][:, 2]
    se_base = (T_panel @ np.append(_SE_CELL_PANEL, 1.0))[:3]
    if approach == "panel_normal":
        cam_pos = se_base + standoff * panel_Z
    elif approach == "base_neg_y":
        cam_pos = se_base + standoff * np.array([0.0, -1.0, 0.0])
    else:
        raise ValueError(f"Unknown approach: {approach}")
    tcp = _tcp_pos(cam_pos, R_tcp, t_cam_in_rh)
    T = np.eye(4)
    T[:3, :3] = R_tcp
    T[:3, 3] = tcp
    return T, tcp


def _build_motion_gen(world_meta):
    from curobo.geom.types import Cuboid, WorldConfig
    from curobo.types.base import TensorDeviceType
    from curobo.types.robot import RobotConfig
    from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig

    tensor_args = TensorDeviceType(device=torch.device("cuda:0"))
    robot_cfg_data = deepcopy(_load_yaml(ROBOT_YML_PATH))
    kine = robot_cfg_data["robot_cfg"]["kinematics"]
    kine["urdf_path"] = str(URDF_PATH)
    kine["collision_spheres"] = str(SPHERES_PATH)
    robot_cfg = RobotConfig.from_dict(robot_cfg_data, tensor_args=tensor_args)
    cuboids = [
        Cuboid(name=o["name"],
               pose=[float(v) for v in o["pose_wxyz"]],
               dims=[float(v) for v in o["dims_m"]])
        for o in world_meta["objects"]
        if o.get("enabled", True) and o.get("type") == "cuboid"
    ]
    world_cfg = WorldConfig(cuboid=cuboids)
    mg_cfg = MotionGenConfig.load_from_robot_config(
        robot_cfg, world_cfg, tensor_args=tensor_args,
        num_trajopt_seeds=16, num_graph_seeds=16,
        collision_cache={"obb": 30, "mesh": 10},
        use_cuda_graph=False,
        self_collision_check=world_meta["self_collision_check_enabled"],
        self_collision_opt=world_meta["self_collision_check_enabled"],
    )
    mg = MotionGen(mg_cfg)
    mg.warmup(warmup_js_trajopt=False)
    mg.detach_object_from_robot()
    return mg, tensor_args


def _mat4_to_pose(mat4):
    q_xyzw = Rotation.from_matrix(np.asarray(mat4)[:3, :3]).as_quat()
    return mat4[:3, 3].tolist(), [float(q_xyzw[3]), float(q_xyzw[0]),
                                   float(q_xyzw[1]), float(q_xyzw[2])]


def _try_candidate(mg, tensor_args, mat4, n_retries=5):
    from curobo.types.math import Pose
    from curobo.types.robot import JointState as CJS
    from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig

    pos, quat = _mat4_to_pose(mat4)
    start = CJS.from_position(
        position=torch.tensor([np.deg2rad(OVERVIEW_JOINTS_DEG).tolist()],
                              dtype=torch.float32, device=tensor_args.device),
        joint_names=JOINT_NAMES,
    )
    goal = Pose(
        position=torch.tensor([pos], dtype=torch.float32, device=tensor_args.device),
        quaternion=torch.tensor([quat], dtype=torch.float32, device=tensor_args.device),
    )

    best = None  # (success, raw, elapsed, j_ranges, endpoint_deg, is_trivial)
    t0 = time.time()
    for _ in range(n_retries):
        result = mg.plan_single(start, goal, MotionGenPlanConfig(enable_graph=True, max_attempts=4))
        elapsed = time.time() - t0
        success = bool(result.success.item())
        raw = result.status.name if hasattr(result.status, "name") else str(result.status)
        if success and raw in ("None", "NoneType"):
            raw = "SUCCESS"

        if not success:
            if best is None:
                best = (False, raw, round(elapsed, 2), None, None, False)
            continue

        traj = result.get_interpolated_plan().position.cpu().numpy()
        deg = np.rad2deg(traj)
        endpoint_deg = [round(float(d), 1) for d in deg[-1]]
        delta = np.abs(np.array(endpoint_deg) - np.array(OVERVIEW_JOINTS_DEG))
        is_trivial = bool(np.max(delta) < 5.0)

        j_ranges = {}
        ok_lim = True
        for j, (lo, hi) in enumerate(OP_LIMITS_DEG):
            vmin, vmax = float(np.min(deg[:, j])), float(np.max(deg[:, j]))
            j_ranges[f"J{j+1}"] = [round(vmin, 1), round(vmax, 1)]
            if vmin < lo or vmax > hi:
                ok_lim = False

        if ok_lim:
            return True, raw, round(elapsed, 2), j_ranges, endpoint_deg, is_trivial

        best = (False, "JOINT_LIMIT_REJECT", round(elapsed, 2), j_ranges, endpoint_deg, is_trivial)

    return best if best else (False, "PLAN_FAIL", 0.0, None, None, False)


CANDIDATES = [
    ("panel_normal", 0.65),
    ("panel_normal", 0.55),
    ("panel_normal", 0.50),
    ("panel_normal", 0.45),
    ("panel_normal", 0.40),
    ("panel_normal", 0.35),
    ("panel_normal", 0.30),
    ("base_neg_y",   0.50),
    ("base_neg_y",   0.40),
    ("base_neg_y",   0.35),
    ("base_neg_y",   0.30),
]


def main():
    if not torch.cuda.is_available():
        raise SystemExit("CUDA GPU not available")

    T_panel     = _build_panel_T(REG_PATH)
    R_tcp       = fk_gripper_rh(OVERVIEW_JOINTS_DEG)[:3, :3]
    t_cam_in_rh = _cam_offset_in_rh()
    world_meta  = _load_yaml(WORLD_PATH)["scan_collision_world"]

    panel_Z  = T_panel[:3, :3][:, 2]
    se_base  = (T_panel @ np.append(_SE_CELL_PANEL, 1.0))[:3]
    print(f"SE cell in base:         {np.round(se_base, 4)}")
    print(f"Panel Z (toward robot):  {np.round(panel_Z, 4)}")
    print()

    print("Loading cuRobo …")
    mg, tensor_args = _build_motion_gen(world_meta)
    print("Ready.\n")

    print(f"{'':3s} {'approach':14s} {'standoff':8s} {'tcp_pos':38s}  {'result':26s}  time")
    print("-" * 106)

    rows = []
    for approach, standoff in CANDIDATES:
        mat4, tcp = _build_tcp_mat4(approach, standoff, T_panel, R_tcp, t_cam_in_rh)
        ok, raw, elapsed, j_ranges, endpoint_deg, is_trivial = _try_candidate(
            mg, tensor_args, mat4
        )
        label = "PLAN_VALID" if ok else ("TRIVIAL" if is_trivial else raw)
        marker = "OK" if (ok and not is_trivial) else ("~~" if is_trivial else "!!")
        print(f"[{marker}] {approach:14s} {standoff:.2f}m  "
              f"tcp=[{tcp[0]:+.3f},{tcp[1]:+.3f},{tcp[2]:+.3f}]  "
              f"{label:28s}  {elapsed:.2f}s")
        if j_ranges:
            print("     range: " + "  ".join(
                f"J{j}=[{v[0]:.0f},{v[1]:.0f}]" for j, v in j_ranges.items()
            ))
        if endpoint_deg:
            delta = [round(abs(e - o), 1) for e, o in zip(endpoint_deg, OVERVIEW_JOINTS_DEG)]
            print(f"     end:   [{', '.join(f'{d:+.1f}' for d in endpoint_deg)}]")
            print(f"     J1_swing={abs(endpoint_deg[0]-OVERVIEW_JOINTS_DEG[0]):.1f}°  "
                  f"maxΔ={max(delta):.1f}°")
        rows.append({
            "approach": approach, "standoff_m": standoff,
            "label": label, "success": ok and not is_trivial,
            "is_trivial": is_trivial, "raw_status": raw,
            "plan_time_s": elapsed,
            "tcp_pos_m": [round(float(v), 6) for v in tcp],
            "tcp_transform_base": [list(r) for r in mat4[:3].tolist()],
            "endpoint_joints_deg": endpoint_deg,
            "joint_range_deg": j_ranges,
        })

    print()
    valid = [r for r in rows if r["success"]]
    if valid:
        best = valid[0]
        print(f"Best candidate: {best['approach']} {best['standoff_m']}m")
        print(f"  endpoint: {best['endpoint_joints_deg']}")
        if best["joint_range_deg"]:
            print(f"  J6 range: {best['joint_range_deg'].get('J6')}")
    else:
        print("NO valid candidate found.")

    out_path = PROJECT_ROOT / "docs" / "runs" / "RUN-20260528-004_se_candidate_search.yaml"
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", encoding="utf-8") as fh:
        yaml.safe_dump({
            "se_candidate_search": {
                "run_date": "2026-05-28",
                "description": "root/se standoff/approach grid search — board-shift panel registration",
                "panel_registration": str(REG_PATH.relative_to(PROJECT_ROOT)),
                "collision_world": str(WORLD_PATH.relative_to(PROJECT_ROOT)),
                "start_joints_deg": OVERVIEW_JOINTS_DEG,
                "se_cell_panel_frame": _SE_CELL_PANEL.tolist(),
                "se_cell_base_m": [round(float(v), 6) for v in se_base],
                "use_for_automated_motion": False,
                "results": rows,
            }
        }, fh, sort_keys=False, allow_unicode=True)
    print(f"Saved: {out_path}")


if __name__ == "__main__":
    main()
