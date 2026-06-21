#!/usr/bin/env bash
set -euo pipefail

HOST="${1:-110.120.1.66}"
NAMESPACE="${2:-dsr01}"

if [[ -f "${HOME}/doosan_ws/install/setup.bash" ]]; then
  # ROS/colcon setup files may reference unset variables such as COLCON_TRACE.
  # Keep this wrapper strict, but relax nounset only while sourcing setup.bash.
  set +u
  # shellcheck source=/dev/null
  source "${HOME}/doosan_ws/install/setup.bash"
  set -u
fi

echo "[start_gripper_service_stable] host=${HOST} namespace=${NAMESPACE}"
echo "[start_gripper_service_stable] stop_existing_drl=false"
echo "[start_gripper_service_stable] If INITIALIZE status 3 repeats after this, check gripper power/RS-485 or restart controller."

exec ros2 launch dsr_gripper_tcp gripper_service_node.launch.py \
  controller_host:="${HOST}" \
  namespace:="${NAMESPACE}" \
  stop_existing_drl:=false \
  init_attempts:=10 \
  init_timeout_sec:=30.0 \
  init_retry_delay_sec:=2.0 \
  drl_start_retry_count:=5 \
  drl_start_retry_delay_sec:=2.0 \
  post_drl_start_sleep_sec:=2.0
