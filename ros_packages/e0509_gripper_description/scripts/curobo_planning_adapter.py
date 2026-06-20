#!/usr/bin/env python3
"""cuRobo planning adapter and diagnostics for the harvest planner."""

import time

import numpy as np
import torch
from curobo.geom.types import Cuboid, WorldConfig
from curobo.types.math import Pose
from curobo.types.robot import JointState as CuroboJointState
from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig

from harvest_motion_params import (
    CARTESIAN_PLAN_MAX_ATTEMPTS,
    CARTESIAN_PLAN_TIMEOUT_SEC,
    DEBUG_START_COLLISION,
)


class CuroboPlanningAdapter:
    """Thin adapter around MotionGen plan calls plus runtime diagnostics."""

    def __init__(self, node, runtime_log, motion_gen, joint_names, joint_limits,
                 trajectory_guards, world_state_getter,
                 debug_dump_plan_calls: bool = False):
        self.node = node
        self.runtime_log = runtime_log
        self.motion_gen = motion_gen
        self.joint_names = joint_names
        self.joint_limits = joint_limits
        self.trajectory_guards = trajectory_guards
        self.world_state_getter = world_state_getter
        self.debug_dump_plan_calls = debug_dump_plan_calls

    def _log(self):
        return self.node.get_logger()

    def clamp_joints(self, joints):
        return [
            float(np.clip(j, lo, hi))
            for j, (lo, hi) in zip(joints, self.joint_limits)
        ]

    def check_state_feasible_with_world(self, joints, cuboids):
        try:
            self.motion_gen.update_world(WorldConfig(cuboid=cuboids))
            state = CuroboJointState.from_position(
                position=torch.tensor(
                    [self.clamp_joints(joints)],
                    device="cuda:0",
                    dtype=torch.float32,
                ),
                joint_names=self.joint_names,
            )
            valid, status = self.motion_gen.check_start_state(state)
            return bool(valid), status
        finally:
            try:
                self.motion_gen.rollout_fn.primitive_collision_constraint.enable_cost()
                self.motion_gen.rollout_fn.robot_self_collision_constraint.enable_cost()
            except Exception:
                pass

    def diagnose_start_world_collision(self, joints, label):
        if not DEBUG_START_COLLISION:
            return
        static_cuboids, dynamic_cuboids, _ = self.world_state_getter()
        full_world = static_cuboids + dynamic_cuboids
        far_dummy = Cuboid(
            name="debug_far_dummy",
            pose=[10.0, 10.0, 10.0, 1.0, 0.0, 0.0, 0.0],
            dims=[0.01, 0.01, 0.01],
        )
        tests = [("empty_world", [far_dummy])]
        tests += [(f"static:{c.name}", [c]) for c in static_cuboids]
        tests += [(f"dynamic:{c.name}", [c]) for c in dynamic_cuboids]
        bad = []
        try:
            for name, cuboids in tests:
                feasible, status = self.check_state_feasible_with_world(joints, cuboids)
                self._log().warn(
                    f"{label} collision diag {name}: "
                    f"{'OK' if feasible else 'COLLISION'} status={status}")
                if not feasible:
                    bad.append(f"{name}:{status}")
        except Exception as e:
            self._log().warn(f"{label} collision diag failed: {e}")
        finally:
            self.motion_gen.update_world(WorldConfig(cuboid=full_world))
        if bad:
            self._log().error(f"{label} start collision suspects: {bad}")
        else:
            self._log().warn(f"{label} no single obstacle reproduced the collision")

    def diagnose_js_endpoint_collision(self, start_joints, target_joints, label):
        if not DEBUG_START_COLLISION:
            return
        self._log().warn(f"{label} endpoint collision diagnostic")
        self.diagnose_start_world_collision(start_joints, f"{label} start")
        self.diagnose_start_world_collision(target_joints, f"{label} goal")

    def plan(self, start_joints, target_pos, target_quat_wxyz, num_ik_seeds=32,
             max_attempts=None, timeout_sec=None, max_joint_delta_deg=None):
        t0 = time.time()
        start_joints = self.clamp_joints(start_joints)
        start_state = CuroboJointState.from_position(
            position=torch.tensor([start_joints], device="cuda:0", dtype=torch.float32),
            joint_names=self.joint_names,
        )
        target_pose = Pose(
            position=torch.tensor([target_pos], device="cuda:0", dtype=torch.float32),
            quaternion=torch.tensor([target_quat_wxyz], device="cuda:0", dtype=torch.float32),
        )
        if self.debug_dump_plan_calls:
            static_cuboids, dynamic_cuboids, neighbor_spheres = self.world_state_getter()
            self.runtime_log.log(
                "plan_call_debug_dump",
                start_joints_rad=start_joints,
                target_pos_m=target_pos,
                target_quat_wxyz=target_quat_wxyz,
                num_ik_seeds=num_ik_seeds,
                max_attempts=(
                    max_attempts if max_attempts is not None else CARTESIAN_PLAN_MAX_ATTEMPTS
                ),
                timeout_sec=(
                    timeout_sec if timeout_sec is not None else CARTESIAN_PLAN_TIMEOUT_SEC
                ),
                max_joint_delta_deg=max_joint_delta_deg,
                static_cuboids=[
                    {"name": c.name, "pose": c.pose, "dims": c.dims}
                    for c in static_cuboids
                ],
                dynamic_cuboids=[
                    {"name": c.name, "pose": c.pose, "dims": c.dims}
                    for c in dynamic_cuboids
                ],
                neighbor_spheres=[
                    {"name": s.name, "pose": s.pose, "radius": s.radius}
                    for s in neighbor_spheres
                ],
            )
        result = self.motion_gen.plan_single(
            start_state, target_pose,
            MotionGenPlanConfig(
                num_ik_seeds=num_ik_seeds,
                max_attempts=(
                    max_attempts
                    if max_attempts is not None
                    else CARTESIAN_PLAN_MAX_ATTEMPTS
                ),
                timeout=(
                    timeout_sec
                    if timeout_sec is not None
                    else CARTESIAN_PLAN_TIMEOUT_SEC
                ),
                enable_graph_attempt=None,
            ),
        )
        dt = (time.time() - t0) * 1000

        if result.success.item():
            traj = result.get_interpolated_plan().position.cpu().numpy()
            traj = self.trajectory_guards.normalize_equivalents(
                traj, "Cartesian plan")
            if not self.trajectory_guards.in_operational_limits(traj, "Cartesian plan"):
                self._log_cartesian_reject(
                    "operational_joint_limits", start_joints, target_pos,
                    target_quat_wxyz, traj)
                return None
            if not self.trajectory_guards.has_no_spline_jumps(traj, "Cartesian plan"):
                self._log_cartesian_reject(
                    "spline_jump", start_joints, target_pos, target_quat_wxyz, traj)
                return None
            if not self.trajectory_guards.has_reasonable_swing(
                    traj, start_joints, "Cartesian plan",
                    max_joint_delta_deg=max_joint_delta_deg):
                self._log_cartesian_reject(
                    "joint_swing", start_joints, target_pos, target_quat_wxyz, traj)
                return None
            motion_time = float(result.motion_time.item())
            end_deg = [f"{np.rad2deg(v):.1f}" for v in traj[-1]]
            self._log().info(
                f"Plan OK {dt:.0f}ms {traj.shape[0]}pts {motion_time:.2f}s | "
                f"goal={[f'{v*1000:.0f}' for v in target_pos]}mm | "
                f"end_J=[{', '.join(end_deg)}]deg")
            self.runtime_log.log(
                "curobo_plan_success",
                planner="cartesian",
                planning_latency_ms=dt,
                motion_time_sec=motion_time,
                start_joints_rad=start_joints,
                target_pos_m=target_pos,
                target_quat_wxyz=target_quat_wxyz,
                trajectory_rad=traj,
            )
            return traj, motion_time

        status = str(getattr(result, "status", "UNKNOWN"))
        start_deg = [f"{np.rad2deg(v):.1f}" for v in start_joints]
        self._log().error(
            f"Plan FAIL {dt:.0f}ms | status={status} | "
            f"goal={[f'{v*1000:.0f}' for v in target_pos]}mm | "
            f"start_J=[{', '.join(start_deg)}]deg")
        if "INVALID_START_STATE_WORLD_COLLISION" in status:
            self.diagnose_start_world_collision(start_joints, "Cartesian plan")
        self.runtime_log.log(
            "curobo_plan_fail",
            planner="cartesian",
            status=status,
            planning_latency_ms=dt,
            start_joints_rad=start_joints,
            target_pos_m=target_pos,
            target_quat_wxyz=target_quat_wxyz,
        )
        return None

    def plan_js(self, start_joints, target_joints_rad, label,
                skip_swing_check=False, max_joint_delta_deg=None):
        t0 = time.time()
        start_joints = self.clamp_joints(start_joints)
        target_joints_rad = self.clamp_joints(target_joints_rad)
        start_state = CuroboJointState.from_position(
            position=torch.tensor([start_joints], device="cuda:0", dtype=torch.float32),
            joint_names=self.joint_names,
        )
        goal_state = CuroboJointState.from_position(
            position=torch.tensor([target_joints_rad], device="cuda:0", dtype=torch.float32),
            joint_names=self.joint_names,
        )
        result = self.motion_gen.plan_single_js(
            start_state, goal_state, MotionGenPlanConfig(enable_graph=True)
        )
        dt = (time.time() - t0) * 1000

        if result.success.item():
            traj = result.get_interpolated_plan().position.cpu().numpy()
            traj = self.trajectory_guards.normalize_equivalents(traj, label)
            if not self.trajectory_guards.in_operational_limits(traj, label):
                self.runtime_log.log(
                    "curobo_plan_rejected", planner="joint_space", label=label,
                    reason="operational_joint_limits", trajectory_rad=traj)
                return None
            if not self.trajectory_guards.has_no_spline_jumps(traj, label):
                self.runtime_log.log(
                    "curobo_plan_rejected", planner="joint_space", label=label,
                    reason="spline_jump", trajectory_rad=traj)
                return None
            if not skip_swing_check and not self.trajectory_guards.has_reasonable_swing(
                    traj, start_joints, label, max_joint_delta_deg):
                self.runtime_log.log(
                    "curobo_plan_rejected", planner="joint_space", label=label,
                    reason="joint_swing", trajectory_rad=traj)
                return None
            motion_time = float(result.motion_time.item())
            self._log().info(
                f"{label} JS Plan OK {dt:.0f}ms {traj.shape[0]}pts {motion_time:.2f}s | "
                f"goal={[f'{v:.1f}' for v in np.rad2deg(target_joints_rad)]}deg")
            self.runtime_log.log(
                "curobo_plan_success",
                planner="joint_space",
                label=label,
                planning_latency_ms=dt,
                motion_time_sec=motion_time,
                start_joints_rad=start_joints,
                target_joints_rad=target_joints_rad,
                trajectory_rad=traj,
            )
            return traj, motion_time

        status = getattr(result, "status", "?")
        self._log().error(
            f"{label} JS Plan FAIL {dt:.0f}ms | status={status} | "
            f"goal={[f'{v:.1f}' for v in np.rad2deg(target_joints_rad)]}deg")
        if "INVALID_START_STATE_WORLD_COLLISION" in str(status) or "GRAPH_FAIL" in str(status):
            self.diagnose_js_endpoint_collision(start_joints, target_joints_rad, label)
        self.runtime_log.log(
            "curobo_plan_fail",
            planner="joint_space",
            label=label,
            status=str(status),
            planning_latency_ms=dt,
            start_joints_rad=start_joints,
            target_joints_rad=target_joints_rad,
        )
        return None

    def _log_cartesian_reject(self, reason, start_joints, target_pos,
                              target_quat_wxyz, traj):
        self.runtime_log.log(
            "curobo_plan_rejected",
            planner="cartesian",
            reason=reason,
            start_joints_rad=start_joints,
            target_pos_m=target_pos,
            target_quat_wxyz=target_quat_wxyz,
            trajectory_rad=traj,
        )
