#!/usr/bin/env bash
set -euo pipefail

# Clean only host-side gripper service processes.
# This is intentionally narrower than clean_robot_runtime.sh: it does not kill
# bringup, controller_manager, scan, planner, or RViz.

patterns=(
  "ros2 launch dsr_gripper_tcp gripper_service_node.launch.py"
  "dsr_gripper_tcp/lib/dsr_gripper_tcp/gripper_service_node"
  "gripper_service_node --ros-args -r __node:=gripper_service"
)

echo "[clean_gripper_runtime] before:"
pgrep -af 'gripper_service_node|dsr_gripper_tcp' || true

for pat in "${patterns[@]}"; do
  pkill -f "$pat" 2>/dev/null || true
done

sleep 2
ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1 || true
sleep 1

echo "[clean_gripper_runtime] after:"
pgrep -af 'gripper_service_node|dsr_gripper_tcp' || true
ros2 topic info /gripper_service/state -v 2>/dev/null || true
echo "[clean_gripper_runtime] done"
