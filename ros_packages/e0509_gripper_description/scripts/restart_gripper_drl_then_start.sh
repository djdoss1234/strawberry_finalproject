#!/usr/bin/env bash
set -euo pipefail

HOST="${1:-110.120.1.66}"
NAMESPACE="${2:-dsr01}"

if [[ -f "${HOME}/doosan_ws/install/setup.bash" ]]; then
  set +u
  # shellcheck source=/dev/null
  source "${HOME}/doosan_ws/install/setup.bash"
  set -u
fi

echo "[restart_gripper_drl_then_start] host=${HOST} namespace=${NAMESPACE}"
echo "[restart_gripper_drl_then_start] This is the fallback path for: DRL already running but TCP 20002 refuses connection."

echo "[restart_gripper_drl_then_start] stopping local gripper processes first..."
"$(dirname "$0")/clean_gripper_runtime.sh"

echo "[restart_gripper_drl_then_start] requesting /${NAMESPACE}/drl/drl_stop..."
ros2 service call "/${NAMESPACE}/drl/drl_stop" dsr_msgs2/srv/DrlStop "{stop_mode: 1}" || true

echo "[restart_gripper_drl_then_start] waiting for controller DRL state to settle..."
sleep 5

echo "[restart_gripper_drl_then_start] starting gripper service from clean DRL state..."
exec ros2 launch dsr_gripper_tcp gripper_service_node.launch.py \
  controller_host:="${HOST}" \
  namespace:="${NAMESPACE}" \
  stop_existing_drl:=false \
  init_attempts:=10 \
  init_timeout_sec:=30.0 \
  init_retry_delay_sec:=2.0 \
  drl_start_retry_count:=5 \
  drl_start_retry_delay_sec:=2.0 \
  post_drl_start_sleep_sec:=2.0 \
  connect_timeout_sec:=30.0
