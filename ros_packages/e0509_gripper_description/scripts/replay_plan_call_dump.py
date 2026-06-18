#!/usr/bin/env python3
"""Replay logged plan() calls offline to find why a live run diverges from a
clean offline test with the same (start_joints, target, world).

Usage:
    ros2 run e0509_gripper_description curobo_planner_node.py --ros-args \
      -p debug_dump_plan_calls:=true ...   # produces plan_call_debug_dump events
    python3 replay_plan_call_dump.py <runtime_jsonl_path>

For each plan_call_debug_dump event, rebuilds the exact world (static + dynamic
cuboids, neighbor spheres) and start/target the live node used, re-solves with
motion_gen.plan_single, and compares the replayed J3/success against the
live curobo_plan_success/curobo_plan_fail event that followed it in the log.
"""
import json
import os
import sys

import numpy as np
import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from compute_nw_pick_ready_pose import build_motion_gen, JOINT_NAMES  # noqa: E402

from curobo.types.robot import JointState as CuroboJointState
from curobo.types.math import Pose
from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig
from curobo.geom.types import WorldConfig, Cuboid, Sphere


def load_events(path):
    events = []
    with open(path) as f:
        for line in f:
            try:
                events.append(json.loads(line))
            except Exception:
                continue
    return events


def main():
    if len(sys.argv) < 2:
        print("usage: replay_plan_call_dump.py <runtime_jsonl_path>")
        return
    events = load_events(sys.argv[1])

    motion_gen, _ = build_motion_gen()

    dumps = [(i, e) for i, e in enumerate(events) if e.get("event") == "plan_call_debug_dump"]
    if not dumps:
        print("No plan_call_debug_dump events found. Re-run with "
              "-p debug_dump_plan_calls:=true first.")
        return
    print("Found %d plan_call_debug_dump events" % len(dumps))

    for idx, dump_event in dumps:
        d = dump_event["data"]
        # the live result is the next curobo_plan_success/curobo_plan_fail event
        live_result = None
        for e in events[idx + 1: idx + 5]:
            if e.get("event") in ("curobo_plan_success", "curobo_plan_fail"):
                live_result = e
                break

        cuboids = [
            Cuboid(name=c["name"], pose=c["pose"], dims=c["dims"])
            for c in d.get("static_cuboids", []) + d.get("dynamic_cuboids", [])
        ]
        spheres = [
            Sphere(name=s["name"], pose=s["pose"], radius=s["radius"])
            for s in d.get("neighbor_spheres", [])
        ]
        motion_gen.update_world(WorldConfig(cuboid=cuboids, sphere=spheres))

        start_state = CuroboJointState.from_position(
            position=torch.tensor([d["start_joints_rad"]], device="cuda:0", dtype=torch.float32),
            joint_names=JOINT_NAMES,
        )
        target_pose = Pose(
            position=torch.tensor([d["target_pos_m"]], device="cuda:0", dtype=torch.float32),
            quaternion=torch.tensor([d["target_quat_wxyz"]], device="cuda:0", dtype=torch.float32),
        )
        result = motion_gen.plan_single(
            start_state, target_pose,
            MotionGenPlanConfig(
                num_ik_seeds=d["num_ik_seeds"],
                max_attempts=d["max_attempts"],
                timeout=d["timeout_sec"],
            ),
        )

        live_success = live_result.get("event") == "curobo_plan_success" if live_result else None
        replay_success = bool(result.success.item())
        if replay_success:
            replay_j3 = float(np.rad2deg(result.get_interpolated_plan().position.cpu().numpy()[-1][2]))
        else:
            replay_j3 = None

        match = "MATCH" if live_success == replay_success else "DIVERGED"
        print(
            "[%d] target=%s live_success=%s replay_success=%s replay_J3=%s "
            "neighbor_spheres=%d -> %s"
            % (
                idx, [round(v * 1000) for v in d["target_pos_m"]], live_success,
                replay_success, ("%.1f" % replay_j3) if replay_j3 is not None else "-",
                len(spheres), match,
            )
        )


if __name__ == "__main__":
    main()
