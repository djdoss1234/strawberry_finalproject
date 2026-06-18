#!/usr/bin/env bash
set -euo pipefail

patterns=(
  "ros2 launch e0509_gripper_description bringup.launch.py"
  "bringup.launch.py"
  "ros2 launch dsr_gripper_tcp gripper_service_node.launch.py"
  "dsr_bringup2"
  "ros2_control_node"
  "controller_manager"
  "dsr_controller2"
  "joint_state_broadcaster"
  "robot_state_publisher"
  "rviz2"
  "gripper_joint_publisher"
  "ros2 run e0509_gripper_description curobo_planner_node.py"
  "ros2 launch e0509_gripper_description curobo_planner.launch.py"
  "ros2 launch strawberry_motion workspace_scan.launch.py"
  "workspace_scan.launch.py"
  "gripper_service_node"
  "curobo_planner_node.py"
  "joint_jog_control.py"
  "scan_executor_node"
  "strawberry_fusion_node"
)

for pat in "${patterns[@]}"; do
  pkill -f "$pat" 2>/dev/null || true
done

sleep 2
ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1 || true
sleep 1

echo "clean_robot_runtime: done"
