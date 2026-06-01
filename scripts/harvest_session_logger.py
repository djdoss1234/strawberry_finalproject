#!/usr/bin/env python3
"""Harvest session KPI logger.

Subscribes to scan/pick events and logs per-attempt results to a YAML file.
Launch alongside the scan+pick nodes.

Topics consumed:
  /strawberry/scan/status  (String) — PICK_TRIGGER, PICK_COMPLETE, PICK_TIMEOUT
  /dsr01/curobo/pick_complete (Empty) — pick cycle done
  /strawberry/vla/request (PoseStamped) — grasp fail → VLA handoff

Output:
  docs/harvest_logs/session_YYYYMMDD_HHMMSS.yaml
"""

import os
import time
import yaml
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String
from geometry_msgs.msg import PoseStamped

_LOG_DIR = Path(__file__).parent.parent / "docs" / "harvest_logs"

# KPI targets (논문 기준 40-50s, 목표 35s 이하)
KPI_CYCLE_TARGET_SEC = 35.0
KPI_SUCCESS_RATE_TARGET = 0.80


class HarvestSessionLogger(Node):

    def __init__(self):
        super().__init__("harvest_session_logger")
        self._session_start = time.time()
        self._attempts: list = []
        self._current: dict = {}
        self._pick_trigger_time: float = 0.0
        self._total_success = 0
        self._total_vla = 0
        self._total_timeout = 0

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        _LOG_DIR.mkdir(parents=True, exist_ok=True)
        self._log_path = _LOG_DIR / ("session_%s.yaml" % ts)

        self.create_subscription(String, "/strawberry/scan/status", self._status_cb, 10)
        self.create_subscription(Empty, "/dsr01/curobo/pick_complete", self._complete_cb, 10)
        self.create_subscription(PoseStamped, "/strawberry/vla/request", self._vla_cb, 10)

        self.get_logger().info("HarvestSessionLogger → %s" % self._log_path)

    def _status_cb(self, msg: String) -> None:
        text = msg.data
        if text.startswith("PICK_TRIGGER"):
            parts = text.split()
            cell_id = parts[1] if len(parts) > 1 else "?"
            self._pick_trigger_time = time.time()
            self._current = {
                "cell_id": cell_id,
                "trigger_time": datetime.now().isoformat(),
                "result": "IN_PROGRESS",
                "cycle_sec": None,
                "vla_handoff": False,
            }
        elif text.startswith("PICK_COMPLETE"):
            if self._current:
                elapsed = time.time() - self._pick_trigger_time
                self._current["cycle_sec"] = round(elapsed, 2)
                self._current["result"] = "SUCCESS"
                self._total_success += 1
                self._flush_attempt()
        elif text.startswith("PICK_TIMEOUT"):
            if self._current:
                elapsed = time.time() - self._pick_trigger_time
                self._current["cycle_sec"] = round(elapsed, 2)
                self._current["result"] = "TIMEOUT"
                self._total_timeout += 1
                self._flush_attempt()

    def _complete_cb(self, _msg: Empty) -> None:
        pass  # handled via PICK_COMPLETE status string

    def _vla_cb(self, msg: PoseStamped) -> None:
        if self._current:
            self._current["vla_handoff"] = True
            self._current["result"] = "VLA"
            self._total_vla += 1
            if self._current.get("cycle_sec") is None:
                elapsed = time.time() - self._pick_trigger_time
                self._current["cycle_sec"] = round(elapsed, 2)
            self._flush_attempt()

    def _flush_attempt(self) -> None:
        if not self._current:
            return
        self._attempts.append(dict(self._current))
        self._current = {}
        self._save()
        self._print_summary()

    def _save(self) -> None:
        total = len(self._attempts)
        success_rate = self._total_success / total if total > 0 else 0.0
        avg_cycle = (
            sum(a["cycle_sec"] for a in self._attempts if a["cycle_sec"] is not None)
            / max(1, sum(1 for a in self._attempts if a["cycle_sec"] is not None))
        )
        session = {
            "session_start": datetime.fromtimestamp(self._session_start).isoformat(),
            "kpi": {
                "total_attempts": total,
                "success": self._total_success,
                "vla_handoff": self._total_vla,
                "timeout": self._total_timeout,
                "success_rate": round(success_rate, 3),
                "success_rate_target": KPI_SUCCESS_RATE_TARGET,
                "avg_cycle_sec": round(avg_cycle, 2),
                "cycle_target_sec": KPI_CYCLE_TARGET_SEC,
                "kpi_met": success_rate >= KPI_SUCCESS_RATE_TARGET and avg_cycle <= KPI_CYCLE_TARGET_SEC,
            },
            "attempts": self._attempts,
        }
        with self._log_path.open("w") as fh:
            yaml.dump(session, fh, allow_unicode=True, default_flow_style=False)

    def _print_summary(self) -> None:
        total = len(self._attempts)
        rate = self._total_success / total if total > 0 else 0.0
        self.get_logger().info(
            "KPI  success=%d/%d (%.0f%%)  vla=%d  timeout=%d  target=%.0f%%"
            % (self._total_success, total, rate * 100, self._total_vla,
               self._total_timeout, KPI_SUCCESS_RATE_TARGET * 100)
        )


def main(args=None):
    rclpy.init(args=args)
    node = HarvestSessionLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
