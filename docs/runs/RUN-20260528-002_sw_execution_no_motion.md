# RUN-20260528-002: SW Execution No-Motion 분리 실험

## 기본 정보

| 항목 | 내용 |
| --- | --- |
| 날짜/시간 | 2026-05-28 |
| 담당자 | djdoss1234 |
| 단계 | motion / execution |
| scene | refit panel quadtree workspace |
| commit | `3219f31`, `35767bb`, `98367cb` |
| 관련 issue | `docs/issues/ISSUE-20260528-008_sw_plan_valid_execution_no_motion.md` |

## 목적과 완료 기준

- 목적:
  - `root/sw`가 cuRobo `PLAN_VALID`임에도 실제 로봇이 움직이지 않는 원인을
    planner, service response, controller execution 단계로 분리한다.
- 완료 기준:
  - `MoveSplineJoint`, direct `MoveJoint`, staged `MoveJoint` 중 어느 단계에서
    실제 joint motion이 발생하거나 멈추는지 기록한다.

## 입력 조건

- 하드웨어/환경:
  - Doosan E0509
  - RealSense eye-in-hand
  - whiteboard + 4분할 paper workspace
- node/launch:

```bash
ros2 launch strawberry_motion workspace_scan.launch.py \
  enable_robot_execution:=true target_cell:=root/sw
```

- config:
  - `config/panel_registration.yaml`
  - `config/scan_collision_world.yaml`
  - `config/scan_pose_candidates_refit_candidate.yaml`
- target:

```text
root/sw endpoint_deg = [149.6, 1.1, 142.6, -124.0, 93.6, 58.7]
```

## 실행 명령 또는 절차

1. overview pose에서 시작한다.
2. `target_cell:=root/sw`로 scan executor를 실행한다.
3. `/strawberry/scan/start` trigger를 보낸다.
4. terminal log와 joint-state arrival result를 확인한다.

## 관찰 결과

- 1차:
  - `MoveSplineJoint response: success=True`
  - 실제 motion 없음
  - `EXEC_TIMEOUT root/sw`
- 2차:
  - slow spline probe `vel=20`, `time=15s`
  - `success=True`지만 motion 없음
  - no-motion 감지 후 fallback MoveJoint 시도
  - endpoint 도착 실패
- 3차:
  - direct MoveJoint endpoint test
  - `MoveJoint response: success=True`
  - endpoint 도착 실패
- 다음 실험:
  - staged MoveJoint diagnostic 결과 stage4까지 도달하고 stage5에서 no-arrival.
  - stage4를 임시 SW scan pose로 채택.
  - 2026-05-28 12:23 재실행에서 stage4 임시 scan pose 도달 후 scan sequence
    완료를 확인.

```text
stage4 accepted = [133.5, -26.6, 117.8, -88.0, 94.6, 10.2]
stage5 failed   = [145.8, -5.4, 136.6, -115.5, 93.8, 47.9]
```

성공 run:

```text
AT_SCAN_POSE root/sw joints_deg=[133.5 -26.6 117.8 -88.1 94.6 10.3]
SCANNED_EMPTY root/sw no detection in dwell window
RETURNING_TO_OVERVIEW
AT_OVERVIEW joints_deg=[97.8 -94.4 65.9 -10.9 95.5 -94.8]
SCAN_COMPLETE
```

## 시각자료 계획 및 확보 상태

| 자료 | 필요 장면 | 상태 | 원본 위치 | 공개 위치/사용처 |
| --- | --- | --- | --- | --- |
| terminal log | `success=True` + `EXEC_TIMEOUT` | `CAPTURED_TEXT_ONLY` | 사용자 채팅 로그 | Notion/portfolio text |
| RViz | SW cell `SCANNING -> PLANNING_FAIL` | `NOT_CAPTURED` | `artifacts/RUN-20260528-002/raw/` | `docs/assets/motion/` |
| 영상 | SW no-motion 및 staged waypoint 결과 | `NOT_CAPTURED` | `artifacts/RUN-20260528-002/raw/` | 포트폴리오 |

<!-- VISUAL TODO
asset_id: RUN-20260528-002_sw_execution_no_motion
capture: SW 명령 성공 응답 후 미동작, 그리고 staged waypoint 진단 결과
source_path: artifacts/RUN-20260528-002/raw/
public_path: docs/assets/motion/
use_in: GitHub README, 포트폴리오, Notion Run page
status: NOT_CAPTURED
-->

## 판정

```text
PARTIAL_STAGE4_ACCEPTED
```

판정 근거:

- `root/sw` planning validity는 확보됐지만, controller execution validity는
  최종 endpoint에서는 확보되지 않았다.
- staged diagnostic에서 stage4까지는 실기 도달 가능함을 확인했다.
- service response `success=True`만으로는 execution success를 보장하지
  않는다는 중요한 통합 이슈를 확인했다.

## 배운 점과 다음 행동

- 배운 점:
  - motion planning 결과가 valid여도 실제 controller가 명령을 실행하지
    않을 수 있다.
  - ROS service 성공 응답과 실제 robot arrival은 반드시 분리해서 검증해야 한다.
- 다음 작업:
  - stage4 이후 stage5/final endpoint로 향하는 구간의 joint delta와
    singularity/limit proximity를 분석한다.
  - stage4 pose에서 실제 camera FOV가 SW cell 관측에 충분한지 확인한다.

## 포트폴리오/자소서 후보 문장

실전 프로젝트에서 `root/sw` 영역은 cuRobo 기준으로는 `PLAN_VALID`였지만,
Doosan execution layer에서는 `MoveSplineJoint`와 `MoveJoint`가 모두
`success=True`를 반환하고도 실제 관절이 움직이지 않는 문제가 발생했습니다.
이를 단순 planner 실패로 보지 않고, service response와 joint-state arrival을
분리해 검증하면서 controller execution validity 문제로 재정의했습니다.
