# ISSUE-20260528-008: SW PLAN_VALID but Execution No-Motion

## 상태

```text
INVESTIGATING
```

## 문제 현상

- 언제 발견했는가: 2026-05-28 `root/sw` 단일 cell scan 실기 중.
- 어떤 동작에서 발생했는가:
  - refit panel 기준 `root/sw` scan target.
  - cuRobo는 `PLAN_VALID`를 반환.
  - Doosan `/dsr01/motion/move_spline_joint`와 `/dsr01/motion/move_joint`
    모두 service response는 `success=True`.
- 실제 출력/증상:
  - 로봇 관절이 endpoint에 도착하지 않음.
  - executor의 joint-state arrival check에서 `EXEC_TIMEOUT` 발생.
  - 대표 endpoint:

```text
root/sw endpoint_deg = [149.6, 1.1, 142.6, -124.0, 93.6, 58.7]
start overview_deg   = [97.84, -94.40, 65.95, -10.93, 95.49, -94.79]
```

## 영향

- 기능 영향:
  - 4분할 quadtree scan 중 `root/sw`를 신뢰성 있게 방문하지 못함.
  - 전체 coverage와 이후 harvest target discovery가 불완전해짐.
- 안전/실험 영향:
  - `success=True` service response만으로 실제 실행 성공을 판단하면 위험함.
  - 반드시 joint-state 기반 도착 확인이 필요하다는 근거가 됨.

## 원인 분석

- 확인된 사실:
  - `root/sw`는 refit 이후 최초엔 `IK_FAIL`이었으나, standoff/approach
    재탐색으로 `PLAN_VALID` 후보를 확보했다.
  - `MoveSplineJoint`는 `success=True`를 반환하지만 실제 motion이 시작되지
    않는 silent no-motion이 관찰됐다.
  - `MoveSplineJoint.srv` response에는 `msg` 필드가 없다. 로그의 `msg='N/A'`
    는 내부 rejection reason이 아니라 코드에서 찍은 fallback 문자열이다.
  - direct `MoveJoint` endpoint도 `success=True` 이후 endpoint 도착 실패.
- 현재 가설:
  1. SW endpoint 또는 큰 joint delta가 Doosan controller 내부 조건에서
     silent reject/hold된다.
  2. 한 번에 보내는 큰 joint move가 문제이며, 중간 waypoint로 쪼개면
     실행 가능할 수 있다.
  3. cuRobo의 planning validity와 Doosan controller의 executable command
     validity가 불일치한다.

## 시도한 해결 방법

| 시도 | 결과 | 판단 |
| --- | --- | --- |
| SW `panel_normal 0.55 m` 후보 | `PLAN_VALID`, 그러나 overview와 유사하거나 실행성 불안정 | 후보 재탐색 |
| SW `panel_normal 0.30 m` 후보 | `PLAN_VALID`, J1 swing 감소 | 현재 후보 |
| `MoveSplineJoint` slow probe `vel=20`, `time=15s` | `success=True`, 실제 motion 없음 | spline silent no-motion |
| spline no-motion 후 MoveJoint fallback | endpoint 도착 실패 | queue/busy 또는 endpoint 한방 문제 분리 필요 |
| direct MoveJoint endpoint | `success=True`, endpoint 도착 실패 | endpoint 한방 silent reject 의심 |
| staged MoveJoint diagnostic | 구현 완료, 실기 결과 대기 | 다음 실험 |

## 최종 해결 또는 다음 조치

- 다음 조치:
  - cuRobo trajectory를 6개 이하 coarse waypoint로 샘플링해 `MoveJoint`를
    순차 실행한다.
  - 각 stage마다 target, service response, current joint를 기록해 어느
    구간에서 controller가 멈추는지 식별한다.
  - 첫 stage부터 움직이지 않으면 SW 접근 정책 자체를 더 보수적으로
    재설계한다.
  - 중간 stage에서 멈추면 해당 구간의 joint delta, singularity, limit
    proximity를 분석한다.
- 남은 위험:
  - staged `MoveJoint`는 cuRobo spline path를 완전히 그대로 실행하는 것이
    아니므로 collision 검증의 의미가 약해진다.
  - 실기 검증은 저속, clear-space, E-stop 준비 상태에서만 수행한다.

## 검증 근거

- 발견 run:
  - terminal log `2026-05-28 11:30`: `MoveSplineJoint success=True`, no motion,
    `EXEC_TIMEOUT`.
  - terminal log `2026-05-28 11:53`: slow spline no-motion 후 fallback,
    endpoint 도착 실패.
  - terminal log `2026-05-28 12:02`: direct MoveJoint endpoint `success=True`,
    endpoint 도착 실패.
- 관련 run/config:
  - `docs/runs/RUN-20260528-001_sw_candidate_search_v2.yaml`
  - `config/scan_pose_candidates_refit_candidate.yaml`
- 관련 commit:
  - `3219f31 fix: add SW spline no-motion fallback`
  - `35767bb debug: test SW direct MoveJoint endpoint`
  - `98367cb debug: stage SW MoveJoint execution`

## 수정 전후 시각자료

| 구분 | 필요 장면 | 상태 | 경로/사용처 |
| --- | --- | --- | --- |
| 수정 전 | `success=True` 이후 로봇 미동작 + `EXEC_TIMEOUT` terminal log | `CAPTURED_TEXT_ONLY` | 사용자 채팅 로그, worklog |
| 수정 후 | staged MoveJoint 각 waypoint 통과 여부 영상/terminal | `NOT_CAPTURED` | 포트폴리오 troubleshooting 카드 |

<!-- VISUAL TODO
asset_id: ISSUE-20260528-008_sw_execution_no_motion
capture: SW execution no-motion terminal log, RViz cell state, staged waypoint 성공/실패 영상
source_path: artifacts/RUN-20260528-002/raw/
public_path: docs/assets/motion/
use_in: 포트폴리오 문제 해결 카드, Notion Issue page
status: NOT_CAPTURED
-->

## 포트폴리오/면접에서 설명할 포인트

- 문제를 어떻게 분리했는가:
  - 처음에는 `root/sw`가 가깝기 때문에 planning 문제로 보였지만, cuRobo
    `PLAN_VALID`, service `success=True`, joint-state no-arrival을 분리해
    planner validity와 controller execution validity가 다르다는 점을 확인했다.
  - service response가 아니라 실제 joint feedback으로 성공 여부를 판정하도록
    구조를 강화했다.
- 왜 이 해결책을 선택했는가:
  - 한 번에 endpoint를 보내는 명령이 silent no-motion을 보였기 때문에,
    cuRobo trajectory를 coarse waypoint로 나누어 controller가 거부하는
    구간을 찾는 staged diagnostic을 선택했다.
