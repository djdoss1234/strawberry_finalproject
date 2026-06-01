# Local Harvest Dashboard

팀원 dashboard를 djdoss1234 Doosan workspace에서 바로 쓰기 위한 로컬 래퍼입니다.

## 구조

```text
ros2_bridge.py
  /dsr01/joint_states
  /dsr01/system/get_current_pose 또는 /dsr01/tcp_pose
  /strawberry/scan/status
  /dsr01/curobo/pick_complete
  /camera/camera/color/image_raw
  /camera2/camera2/color/image_raw
    -> tools/dashboard/data/harvest_state.json
    -> MJPEG http://localhost:8766/cam0, /cam1

harvest_dashboard.py
  harvest_state.json + MJPEG stream
    -> http://localhost:8765
```

## 실행

먼저 평소처럼 로봇/카메라/스캔 노드를 띄웁니다.

```bash
cd ~/doosan_ws
source install/setup.bash
```

대시보드:

```bash
cd ~/doosan_ws/src/strawberry_finalproject
bash tools/dashboard/start_local_dashboard.sh
```

브라우저:

```text
http://localhost:8765
```

## 현재 환경 기본값

```bash
CAM0_TOPIC=/camera/camera/color/image_raw
CAM1_TOPIC=/camera2/camera2/color/image_raw
MJPEG_PORT=8766
USB_FALLBACK=false
DASHBOARD_SYNC_TELEOP_API=false
HARVEST_STATE_FILE=tools/dashboard/data/harvest_state.json
```

`USB_FALLBACK=false`가 중요합니다. RealSense를 dashboard가 직접 열면 기존 YOLO/RealSense
노드와 충돌해서 `Device or resource busy`가 날 수 있습니다.

`DASHBOARD_SYNC_TELEOP_API=false`도 현재 환경에서는 중요합니다. 팀원 dashboard는 기본적으로
별도 teleop API `http://localhost:8767/status`를 조회하도록 만들어져 있었지만,
이 프로젝트에서는 `ros2_bridge.py`가 ROS2 토픽을 직접 읽어 state file을 갱신합니다.

## FastAPI 의존성

현재 호스트 기본 Python에 `fastapi`가 없으면 `start_local_dashboard.sh`가 웹 대시보드만
Docker로 실행합니다. ROS2 bridge는 호스트에서 실행합니다.

호스트에서 직접 실행하고 싶으면:

```bash
python3 -m pip install fastapi "uvicorn[standard]" websockets
```

## 안전 상태

- 이 dashboard는 현재 모니터링 중심입니다.
- UI의 joint command는 `harvest_state.json`에 pending command를 쓰는 수준이며,
  Doosan motion service로 직접 연결하지 않습니다.
- 실제 motion은 기존 `workspace_scan.launch.py`, `scan_executor_node.py`,
  `curobo_planner_node.py` 경계를 통해 실행합니다.

## 2026-06-01 TODO

- YOLO 재학습 모델 출력이 확정되면 detection count / target count를 dashboard에 연결.
- `harvest_session_logger.py`의 KPI YAML과 dashboard state를 동기화.
- gripper actual feedback이 구현되면 `/gripper/position` 또는 새 feedback topic에 연결.
