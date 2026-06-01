#!/usr/bin/env bash
set -euo pipefail

# Dashboard runner for djdoss1234 local Doosan workspace.
#
# - ROS2 bridge runs on the host so it can read /dsr01/* and /strawberry/* topics.
# - Web dashboard runs locally if FastAPI is installed; otherwise it uses Docker.
# - This script does not start RealSense nodes. Use the existing project launch/Yolo nodes.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
WS_DIR="${DOOSAN_WS_DIR:-/home/user/doosan_ws}"
STATE_FILE="${HARVEST_STATE_FILE:-$SCRIPT_DIR/data/harvest_state.json}"

export HARVEST_STATE_FILE="$STATE_FILE"
export CAM0_TOPIC="${CAM0_TOPIC:-/camera/camera/color/image_raw}"
export CAM1_TOPIC="${CAM1_TOPIC:-/camera2/camera2/color/image_raw}"
export MJPEG_PORT="${MJPEG_PORT:-8766}"
export USB_FALLBACK="${USB_FALLBACK:-false}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export DASHBOARD_SYNC_TELEOP_API="${DASHBOARD_SYNC_TELEOP_API:-false}"
export DASHBOARD_ENABLE_JOG="${DASHBOARD_ENABLE_JOG:-true}"
export DASHBOARD_ENABLE_MOVEJ="${DASHBOARD_ENABLE_MOVEJ:-true}"
export DASHBOARD_ENABLE_GRIPPER="${DASHBOARD_ENABLE_GRIPPER:-true}"
export DASHBOARD_JOG_MAX_PERCENT="${DASHBOARD_JOG_MAX_PERCENT:-20.0}"
export DASHBOARD_MOVEJ_MAX_VEL="${DASHBOARD_MOVEJ_MAX_VEL:-30.0}"
export DASHBOARD_MOVEJ_MAX_ACC="${DASHBOARD_MOVEJ_MAX_ACC:-40.0}"

mkdir -p "$(dirname "$STATE_FILE")"

if [[ -f "$WS_DIR/install/setup.bash" ]]; then
  set +u
  # shellcheck disable=SC1091
  source "$WS_DIR/install/setup.bash"
  set -u
elif [[ -f /opt/ros/humble/setup.bash ]]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  set -u
fi

cleanup() {
  echo "[dashboard] stopping..."
  if [[ -n "${BRIDGE_PID:-}" ]]; then
    kill "$BRIDGE_PID" 2>/dev/null || true
  fi
  if [[ -n "${DASH_PID:-}" ]]; then
    kill "$DASH_PID" 2>/dev/null || true
  fi
  if [[ "${DASHBOARD_DOCKER_STARTED:-false}" == "true" ]]; then
    docker rm -f strawberry-harvest-dashboard >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

echo "[dashboard] project: $PROJECT_DIR"
echo "[dashboard] state:   $STATE_FILE"
echo "[dashboard] cam0:    $CAM0_TOPIC"
echo "[dashboard] cam1:    $CAM1_TOPIC"
echo "[dashboard] usb fallback: $USB_FALLBACK"
echo "[dashboard] teleop-api sync: $DASHBOARD_SYNC_TELEOP_API"
echo "[dashboard] jog enabled: $DASHBOARD_ENABLE_JOG max=${DASHBOARD_JOG_MAX_PERCENT}%"
echo "[dashboard] movej enabled: $DASHBOARD_ENABLE_MOVEJ max_vel=${DASHBOARD_MOVEJ_MAX_VEL} max_acc=${DASHBOARD_MOVEJ_MAX_ACC}"
echo "[dashboard] gripper enabled: $DASHBOARD_ENABLE_GRIPPER"

python3 "$SCRIPT_DIR/ros2_bridge.py" &
BRIDGE_PID=$!

sleep 2

if python3 -c "import fastapi, uvicorn" >/dev/null 2>&1; then
  echo "[dashboard] starting FastAPI dashboard on host: http://localhost:8765"
  python3 "$SCRIPT_DIR/harvest_dashboard.py" \
    --host 0.0.0.0 \
    --port 8765 \
    --camera-url-0 "http://localhost:${MJPEG_PORT}/cam0" \
    --camera-url-1 "http://localhost:${MJPEG_PORT}/cam1" &
  DASH_PID=$!
  wait "$DASH_PID"
else
  echo "[dashboard] FastAPI not found on host; using Docker dashboard image."
  echo "[dashboard] if build fails, install network/dependency access or run:"
  echo "            python3 -m pip install fastapi 'uvicorn[standard]' websockets"
  docker build -t strawberry-harvest-dashboard "$SCRIPT_DIR"
  DASHBOARD_DOCKER_STARTED=true
  docker rm -f strawberry-harvest-dashboard >/dev/null 2>&1 || true
  echo "[dashboard] starting Docker dashboard on: http://localhost:8765"
  echo "[dashboard] this command stays in the foreground; press Ctrl+C to stop."
  echo "[dashboard] if the browser does not open, check: docker logs strawberry-harvest-dashboard"
  docker run --rm \
    --name strawberry-harvest-dashboard \
    --network host \
    -e HARVEST_STATE_FILE=/data/harvest_state.json \
    -e CAMERA_URL_0="http://localhost:${MJPEG_PORT}/cam0" \
    -e CAMERA_URL_1="http://localhost:${MJPEG_PORT}/cam1" \
    -e DASHBOARD_SYNC_TELEOP_API="$DASHBOARD_SYNC_TELEOP_API" \
    -v "$SCRIPT_DIR/data:/data" \
    strawberry-harvest-dashboard
fi
