#!/usr/bin/env python3
"""Gripper service/action wrapper for the harvest planner."""

import time

from harvest_motion_params import (
    GRASP_EMPTY_POSITION_THRESHOLD,
    GRASP_VERIFY_TIMEOUT_SEC,
    GRIPPER_APPROACH_POS,
    GRIPPER_CLOSE_SETTLE_SEC,
)


class HarvestGripperClient:
    """Blocking helpers around /gripper_service SetPosition/GetState/SafeGrasp."""

    def __init__(self, node, runtime_log, cli_set_position, cli_get_state,
                 safe_grasp_cli, safe_grasp_action_type,
                 use_safe_grasp_action: bool,
                 safe_grasp_max_current: int,
                 safe_grasp_current_delta_threshold: int,
                 safe_grasp_timeout_sec: float,
                 grasp_current_contact_threshold_raw: int,
                 set_position_type, get_state_type):
        self.node = node
        self.runtime_log = runtime_log
        self.cli_set_position = cli_set_position
        self.cli_get_state = cli_get_state
        self.safe_grasp_cli = safe_grasp_cli
        self.safe_grasp_action_type = safe_grasp_action_type
        self.use_safe_grasp_action = use_safe_grasp_action
        self.safe_grasp_max_current = safe_grasp_max_current
        self.safe_grasp_current_delta_threshold = safe_grasp_current_delta_threshold
        self.safe_grasp_timeout_sec = safe_grasp_timeout_sec
        self.grasp_current_contact_threshold_raw = grasp_current_contact_threshold_raw
        self.set_position_type = set_position_type
        self.get_state_type = get_state_type

    def _log(self):
        return self.node.get_logger()

    def reset(self):
        """Return gripper to the approach/open position."""
        self.set_position(GRIPPER_APPROACH_POS, timeout_sec=5.0)

    def open_for_stem_descent(self):
        """Open gripper before horizontal approach and open-stem descent."""
        self._log().info(
            f"1 open gripper for stem descent: set_position={GRIPPER_APPROACH_POS}")
        self.runtime_log.log(
            "gripper_command",
            command="set_position",
            position=GRIPPER_APPROACH_POS,
            purpose="open_during_horizontal_approach_and_stem_descent",
        )
        return self.set_position(GRIPPER_APPROACH_POS, timeout_sec=3.0)

    def set_position(self, position: int, timeout_sec: float = 5.0) -> bool:
        if self.cli_set_position is None or self.set_position_type is None:
            self._log().warn("GRIPPER: cli_set_position unavailable (virtual?)")
            return False
        if not self.cli_set_position.wait_for_service(timeout_sec=1.0):
            self._log().warn("GRIPPER: /gripper_service/set_position not available")
            return False
        req = self.set_position_type.Request()
        req.position = int(position)
        req.timeout_sec = float(timeout_sec)
        future = self.cli_set_position.call_async(req)
        t0 = time.time()
        while not future.done() and (time.time() - t0) < timeout_sec + 2.0:
            time.sleep(0.05)
        if not future.done():
            self._log().warn(f"GRIPPER: set_position({position}) timed out")
            return False
        res = future.result()
        if res is None or not res.success:
            self._log().warn(
                f"GRIPPER: set_position({position}) failed: "
                f"{getattr(res, 'message', 'None')}")
            return False
        return True

    def verify_grasp(self):
        """Read gripper state and classify contact after a position close."""
        if self.cli_get_state is None or self.get_state_type is None:
            return "GRASP_UNVERIFIED", -1, -1, "get_state unavailable (virtual?)"
        if not self.cli_get_state.wait_for_service(timeout_sec=0.5):
            return "GRASP_UNVERIFIED", -1, -1, "get_state service unavailable"
        req = self.get_state_type.Request()
        req.force_read = True
        future = self.cli_get_state.call_async(req)
        t0 = time.time()
        while not future.done() and (time.time() - t0) < GRASP_VERIFY_TIMEOUT_SEC:
            time.sleep(0.05)
        if not future.done():
            return "GRASP_UNVERIFIED", -1, -1, "get_state timeout"
        res = future.result()
        if not res or not res.success:
            return "GRASP_UNVERIFIED", -1, -1, "get_state service error"
        position = res.state.present_position
        current_raw = res.state.present_current
        if position < 0 or current_raw < 0:
            return (
                "GRASP_UNVERIFIED", position, current_raw,
                "hardware state read failed (virtual mode or serial error)")
        if position >= GRASP_EMPTY_POSITION_THRESHOLD:
            return "GRASP_EMPTY", position, current_raw, (
                f"fully closed (pos={position} >= threshold={GRASP_EMPTY_POSITION_THRESHOLD})")
        if (
            self.grasp_current_contact_threshold_raw >= 0
            and current_raw < self.grasp_current_contact_threshold_raw
        ):
            return "GRASP_UNVERIFIED", position, current_raw, (
                f"position indicates contact but current={current_raw} below calibrated "
                f"threshold={self.grasp_current_contact_threshold_raw}")
        return "GRASP_CONTACT_DETECTED", position, current_raw, (
            f"jaw stopped at pos={position} and current_raw={current_raw}; "
            f"current threshold={'disabled' if self.grasp_current_contact_threshold_raw < 0 else self.grasp_current_contact_threshold_raw}")

    def close_and_verify_grasp(self):
        """Close gripper and return a normalized grasp result tuple."""
        self._log().info("3 close gripper + verify grasp")
        self.runtime_log.log("gripper_command", command="close")
        if self.use_safe_grasp_action and self.safe_grasp_cli is not None:
            if self.safe_grasp_cli.server_is_ready():
                result = self.close_via_safe_grasp_action()
                self.log_verify_result(result)
                return result
            self._log().warn(
                "SAFE_GRASP: action server not ready — fallback to set_position+get_state")
        close_ok = self.set_position(700, timeout_sec=10.0)
        if not close_ok:
            return "GRIPPER_CLOSE_FAILED", -1, -1, "set_position(700) failed"
        time.sleep(GRIPPER_CLOSE_SETTLE_SEC)
        result = self.verify_grasp()
        self.log_verify_result(result)
        return result

    def log_verify_result(self, result):
        grasp_result, present_pos, present_current_raw, grasp_reason = result
        if grasp_result == "GRIPPER_CLOSE_FAILED":
            return
        self._log().info(
            f"VERIFY_GRASP: {grasp_result} present_pos={present_pos} "
            f"current_raw={present_current_raw} — {grasp_reason}")
        self.runtime_log.log(
            "verify_grasp",
            result_code=grasp_result,
            present_position=present_pos,
            present_current_raw=present_current_raw,
            reason=grasp_reason,
            close_command_pos=700,
            empty_threshold=GRASP_EMPTY_POSITION_THRESHOLD,
            current_contact_threshold_raw=self.grasp_current_contact_threshold_raw,
        )

    def close_via_safe_grasp_action(self):
        if self.safe_grasp_action_type is None or self.safe_grasp_cli is None:
            return "GRASP_UNVERIFIED", -1, -1, "SafeGrasp: action unavailable"
        goal = self.safe_grasp_action_type.Goal()
        goal.target_position = 700
        goal.max_current = self.safe_grasp_max_current
        goal.current_delta_threshold = self.safe_grasp_current_delta_threshold
        goal.timeout_sec = self.safe_grasp_timeout_sec

        feedback_samples = []

        def _on_feedback(fb_msg):
            fb = fb_msg.feedback
            feedback_samples.append({
                "present_position": fb.present_position,
                "present_current": fb.present_current,
                "current_delta": fb.current_delta,
                "grasp_detected": fb.grasp_detected,
            })

        send_future = self.safe_grasp_cli.send_goal_async(
            goal, feedback_callback=_on_feedback)
        deadline = time.time() + self.safe_grasp_timeout_sec + 5.0
        while not send_future.done() and time.time() < deadline:
            time.sleep(0.05)
        if not send_future.done():
            return "GRASP_UNVERIFIED", -1, -1, "SafeGrasp: goal send timeout"

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return "GRASP_UNVERIFIED", -1, -1, "SafeGrasp: goal rejected"

        result_future = goal_handle.get_result_async()
        while not result_future.done() and time.time() < deadline:
            time.sleep(0.05)
        if not result_future.done():
            return "GRASP_UNVERIFIED", -1, -1, "SafeGrasp: result timeout"

        wrapped = result_future.result()
        if wrapped is None:
            return "GRASP_UNVERIFIED", -1, -1, "SafeGrasp: result None"

        res = wrapped.result
        final_pos = res.final_position
        final_cur = res.final_current

        self.runtime_log.log(
            "safe_grasp_action",
            grasp_detected=res.grasp_detected,
            object_lost=res.object_lost,
            final_position=final_pos,
            final_current=final_cur,
            message=res.message,
            max_current=self.safe_grasp_max_current,
            current_delta_threshold=self.safe_grasp_current_delta_threshold,
            feedback_samples=feedback_samples,
        )

        if final_pos < 0 or final_cur < 0:
            return (
                "GRASP_UNVERIFIED", final_pos, final_cur,
                f"SafeGrasp: invalid state (virtual/serial error) — {res.message}")

        if res.grasp_detected:
            if final_pos >= GRASP_EMPTY_POSITION_THRESHOLD:
                return "GRASP_EMPTY", final_pos, final_cur, (
                    f"SafeGrasp: current spike but jaw fully closed "
                    f"pos={final_pos} >= {GRASP_EMPTY_POSITION_THRESHOLD}")
            return "GRASP_CONTACT_DETECTED", final_pos, final_cur, (
                f"SafeGrasp detected: pos={final_pos} current={final_cur} — {res.message}")

        if final_pos >= GRASP_EMPTY_POSITION_THRESHOLD:
            return "GRASP_EMPTY", final_pos, final_cur, (
                f"SafeGrasp: jaw fully closed pos={final_pos} >= "
                f"{GRASP_EMPTY_POSITION_THRESHOLD} — {res.message}")

        return "GRASP_UNVERIFIED", final_pos, final_cur, (
            f"SafeGrasp: no detection, pos={final_pos} below threshold — {res.message}")
