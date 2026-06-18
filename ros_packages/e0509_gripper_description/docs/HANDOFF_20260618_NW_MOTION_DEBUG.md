# 2026-06-18 NW 셀 수확 모션 디버그 인계

## 현재 목표

SW 단일 딸기에서 검증됐던 수확 모션을 NW 잎/줄기 가림 셀로 확장한다.
현재 place는 잠시 중단하고, NW 셀에서 딸기 접근/파지 모션 안정화가 우선이다.

## 2026-06-18 추가 구현: collect-then-pick

NW 세부 scan pose에서 바로 pick을 시작하면, perception에는 유리하지만 pick
branch로는 불리한 자세에서 수확이 시작되어 final approach가 계속 막혔다.
이를 분리하기 위해 `strawberry_motion` scan executor에 collect-then-pick 모드를
추가했다.

구현 흐름:

```text
root/nw/nw scan
 -> root/nw/ne scan
 -> root/nw/se scan
 -> root/nw/sw scan
 -> 후보 PoseStamped 전체 수집/중복 제거
 -> root/nw 중앙 pick-ready pose로 이동
 -> best target 1개를 /dsr01/curobo/pick_pose로 forward
```

사용된 중앙 pick-ready pose:

```text
source: config/scan_pose_candidates_refit_candidate.yaml / root/nw
TCP BASE [mm,deg] = [-225.46, 338.93, 902.31, 88.42, 87.31, -89.88]
joints_deg = [144.09, 22.90, -1.00, -238.52, -75.31, 108.68]
```

새 launch parameter:

```text
collect_then_pick:=true             # workspace_scan.launch.py 기본값 true
collect_pick_ready_cell:=root/nw    # 생략 시 target_cell 사용
```

다음 실기에서 기대 로그:

```text
COLLECT_THEN_PICK_ENABLED scan_cells=[...] pick_ready_cell=root/nw
COLLECT_TARGETS root/nw/nw kept=... total_buffer=...
COLLECT_TARGETS root/nw/ne kept=... total_buffer=...
COLLECT_TARGETS root/nw/se kept=... total_buffer=...
COLLECT_TARGETS root/nw/sw kept=... total_buffer=...
COLLECT_THEN_PICK_READY_MOVE root/nw candidates=... best=(x,y,z)mm
PICK_TRIGGER root/nw/best 1/...
```

주의:

- `max_total_picks:=1`이면 수집 후보 중 첫 best target 1개만 시도한다.
- 한 개를 따면 줄기/잎이 움직일 수 있으므로 NW 안정화 단계에서는
  `1 pick -> 재스캔 -> 다음 pick`을 기본으로 한다.
- 이 구현은 scan executor의 target forwarding 방식 변경이며,
  `curobo_planner_node.py`의 SW 수확 모션 자체를 직접 바꾸지 않는다.

## 오늘 확인한 문제

### 1. NW에서 딸기까지 충분히 접근하지 못함

최근 실행 로그:

```text
logs/runtime/2026-06-18/
curobo_planner_node_20260618T145519-fe017af0.jsonl
```

관찰:

- target 예시: raw `(-238, 672, 819)mm`
- measured TCP direct final approach 사용
- cuRobo final approach probing 결과:
  - `150mm` IK_FAIL
  - `130mm` IK_FAIL
  - `110mm` IK_FAIL
  - `90mm` Plan OK
- 이후 남은 `60mm`를 SW baseline처럼 `TOOL +Z MoveLine`으로 실행하려 했으나,
  Doosan MoveLine 서비스가 success처럼 빠르게 반환하고 실제 joint 변화가
  `max_delta=0.01deg` 수준이라 실패로 판정됨.

결론:

```text
물리적으로 절대 못 가는 거리라기보다,
현재 선택된 measured-TCP 자세에서 cuRobo deep final approach가 막히고,
MoveLine fallback도 실제 이동 없이 success를 반환하는 것이 핵심 병목이다.
```

### 2. SW와 NW가 같은 방식이 아니었음

SW 성공 기준 로그:

```text
logs/runtime/2026-06-09/
curobo_planner_node_20260609T160052-da5edd5a.jsonl
```

SW는 다음 방식이었다.

```text
cuRobo pre-approach
 -> Doosan TOOL +Z MoveLine 20mm
 -> Doosan TOOL +Z extra advance 65mm
 -> close
 -> TOOL -Z retreat
```

NW measured TCP 실험은 다음 방식이었다.

```text
cuRobo pre-approach
 -> measured TCP 기준 final approach를 cuRobo로 깊게 계획
 -> 실패 시 얕은 cuRobo fallback
 -> 필요 시 TOOL MoveLine fallback
```

따라서 "SW에서는 됐는데 NW는 왜 안 되냐"의 직접 원인은 SW baseline
motion policy와 NW measured-TCP policy가 다르기 때문이다.

### 3. 속도 30% 증속이 실제로 반영되지 않았음

`SPLINE_TIME_SCALE`은 줄였지만 `MoveSplineJoint` 요청값은 여전히
`req.vel = [36.0] * 6`, `req.acc = [54.0] * 6`로 하드코딩되어 있었다.

수정:

```text
SPLINE_VEL_DEG_S  = 47.0
SPLINE_ACC_DEG_S2 = 70.0
SPLINE_TIME_SCALE = 0.87
SPLINE_MIN_TIME   = 0.58
```

다음 로그에서 `velocity_deg_s=[47.0, ...]`가 찍히는지 확인해야 한다.

### 4. NW/SW scan pose가 먼저 돌던 이유

`strawberry_motion`의 `scan_executor_node.py`에서 NW occlusion 실험 중
위쪽 잎 후보를 먼저 잡는 것을 피하려고 subcell scan order를 한때
`sw -> se -> nw -> ne`로 변경했었다.

현재는 원인 분리를 위해 원래 visual order로 되돌렸다.

```text
nw -> ne -> sw -> se
```

단, candidate sorting/grouping 쪽에는 lower stem-level 후보를 우선하려는
변경이 남아 있다. 이는 `max_total_picks=1` 조건에서 높은 잎/꼭지 후보를
먼저 소비하는 문제를 줄이기 위한 것이다.

### 5. 현재 scan executor는 "4개 포즈 collect 후 pick" 구조가 아님

사용자 가정:

```text
NW 세부 scan pose 4개를 전부 훑음
 -> 후보를 기억함
 -> 해당 pose 또는 안정적인 중앙 pose로 돌아감
 -> pick 실행
```

실제 코드 흐름:

```text
각 sub-scan pose로 이동
 -> dwell 중 detection buffer 수집
 -> 해당 pose에서 target이 잡히면 즉시 /dsr01/curobo/pick_pose로 forward
 -> curobo_planner가 그 scan pose의 현재 joint branch에서 바로 pick 시작
 -> pick_complete 대기
 -> 다음 scan pose 이동
```

즉 현재는 `보고 즉시 따기`에 가깝다. `max_total_picks:=1`이면 첫 번째로
잡힌 후보 하나에 바로 수확 시도를 소비한다. NW에서는 이 때문에 scan view로는
좋지만 pick branch로는 나쁜 세부 scan pose에서 바로 접근을 시작하게 된다.

최근 로그 예:

```text
target raw=(-374,672,765)mm
start_J1=-30.1deg
pre-approach end_J=[-25.6, -49.7, 29.2, 294.2, -81.3, 109.3]deg
final approach 150/130/110/90mm IK_FAIL, 70mm만 성공
남은 80mm TOOL MoveLine no-motion으로 실패
```

결론:

```text
perception target은 이전보다 정상화됐지만,
pick을 시작하는 joint branch가 NW 세부 scan pose에 묶여 있어 final approach가 막힌다.
```

### 6. 다음 권장 구조: collect-then-pick + NW 중앙 pick-ready pose

새 티칭을 당장 추가하지 않고, 이미 검증/기록된 gripper-centered NW 중앙 pose를
pick-ready pose로 활용한다.

NW 중앙 gripper-centered pose:

```text
source: config/scan_pose_candidates_refit_candidate.yaml / root/nw
TCP BASE [mm,deg] = [-225.46, 338.93, 902.31, 88.42, 87.31, -89.88]
joints_deg = [144.09, 22.90, -1.00, -238.52, -75.31, 108.68]
```

권장 시퀀스:

```text
1. root/nw/nw, root/nw/ne, root/nw/sw, root/nw/se를 순서대로 scan
2. 각 pose에서는 pick을 실행하지 않고 후보 PoseStamped만 buffer에 저장
3. 전체 후보 중 best target 1개 선택
   - z 너무 높은 leaf/top 후보 제외
   - stem keypoint confidence/geometry 정상
   - 낮은 stem-level 후보 우선
   - 필요 시 x/z 위치와 subcell 정보를 함께 기록
4. 로봇을 root/nw 중앙 pick-ready joints로 이동
5. 저장해 둔 best target pose를 /dsr01/curobo/pick_pose로 publish
6. cuRobo planner는 중앙 pick-ready branch에서 수확 시작
7. pick/detach/retreat 후에는 재스캔 권장
```

재스캔 판단:

- 모형 딸기라도 한 개를 따면 줄기/잎이 움직이고, 인접 딸기 keypoint가 조금 변할 수 있다.
- 따라서 NW 안정화 단계에서는 `1 pick -> 재스캔 -> 다음 pick`이 안전하다.
- 향후 충분히 안정되면 같은 scan 후보 묶음에서 2개 이상 연속 수확하는 최적화를 검토한다.

위에서 아래로 훑는 대안:

```text
top -> mid -> low 순서로 scan만 수행
 -> 후보를 누적
 -> 중앙 pick-ready pose에서 best target 수확
```

이는 지금의 2x2 subcell 순회보다 perception 관점에서 자연스럽고, 잎/상단 후보를
먼저 소비하지 않게 만들 수 있다. 단, 구현은 동일하게 `scan은 저장만, pick은
중앙 pose에서`라는 원칙을 따른다.

## 오늘 코드 수정

### `scripts/curobo_planner_node.py`

수정 내용:

- measured TCP final approach 관련 ROS parameter 추가
  - `direct_curobo_final_approach_for_measured_tcp`
  - `measured_tcp_max_approach_m`
  - `measured_tcp_tool_line_after_curobo_fallback`
- measured TCP에서 첫 pre-approach 성공 자세를 바로 확정하지 않도록 변경
- 각 orientation 후보에 대해 final depth reachability를 미리 probing
  - `150 -> 130 -> 110 -> 90 -> 70 -> 60mm`
  - 가장 깊게 들어갈 수 있는 자세를 선택
- cuRobo deep approach 실패 시 얕은 cuRobo fallback 후, 남은 거리만
  `TOOL +Z MoveLine`으로 보강하는 경로 추가
- `trajectory_has_reasonable_swing()`이 scalar max delta를 받으면 6축 리스트로
  확장하도록 수정
- MoveSplineJoint 실제 velocity/acceleration 요청값을 30% 증속
- z guard 상향:
  - `MEASURED_TCP_TARGET_Z_MAX_M = 1.050`

### `strawberry_motion` 쪽 변경

파일:

```text
/home/user/doosan_ws/src/strawberry_finalproject/
src/strawberry_motion/execution/scan_executor_node.py
```

확인된 변경:

- overview pose 판정을 wrap-aware joint tolerance로 변경
- `max_total_picks` parameter 추가
- scan target joint wrapping에서 J1/J4/J6 continuity 개선
- candidate sorting은 lower stem-level 후보를 우선하도록 조정
- subcell physical scan order는 다시 `nw -> ne -> sw -> se`로 복구

주의:

`strawberry_finalproject`에는 이 파일 외에도 `workspace_scan.launch.py`,
`config/scan_pose_candidates_depth2.yaml` 변경이 남아 있다. 이 둘은 이번
인계 기준으로 별도 변경이며, 무심코 되돌리거나 함께 커밋하지 말 것.

## 검증 완료

```bash
python3 -m py_compile scripts/curobo_planner_node.py
python3 -m py_compile /home/user/doosan_ws/src/strawberry_finalproject/src/strawberry_motion/execution/scan_executor_node.py
git diff --check
colcon build --packages-select e0509_gripper_description
colcon build --packages-select strawberry_motion
```

위 항목 통과.

## 다음 실행 커맨드

Planner:

```bash
source ~/doosan_ws/install/setup.bash

ros2 run e0509_gripper_description curobo_planner_node.py --ros-args \
  -p measured_tcp_plan_only:=false \
  -p direct_curobo_final_approach_for_measured_tcp:=true \
  -p measured_tcp_max_approach_m:=0.150 \
  -p measured_tcp_tool_line_after_curobo_fallback:=true
```

Scan:

```bash
source ~/doosan_ws/install/setup.bash

ros2 launch strawberry_motion workspace_scan.launch.py \
  enable_robot_execution:=true \
  target_cell:=root/nw \
  enable_fusion_detection:=true \
  enable_pick_integration:=true \
  max_total_picks:=1 \
  scan_movej_vel:=45.0 \
  scan_movej_acc:=60.0
```

Trigger:

```bash
ros2 service call /strawberry/scan/start std_srvs/srv/Trigger "{}"
```

## 다음 실기에서 반드시 볼 로그

Planner startup:

```text
MEASURED_TCP_FINAL_APPROACH direct_curobo=True max=150mm tool_line_after_fallback=True
```

후보 선택:

```text
MEASURED_TCP_FINAL_PROBE_BEST depth=...mm variant=...
```

판단 기준:

- `depth=150mm` 또는 `130mm`가 선택되면 이번 수정이 효과 있음
- 계속 `90mm` 이하만 선택되면 현재 NW scan pose / target / measured TCP
  orientation 조합에서는 깊은 접근이 어렵다는 뜻
- `FINAL_APPROACH_TOOL_FINISH`가 실행됐는데 joint가 안 움직이면,
  Doosan MoveLine이 해당 자세에서 실제 이동을 수행하지 않는 문제로 분리됨

## 아직 못 해결한 것

- NW measured TCP 경로의 실기 성공은 아직 검증되지 않았다.
- MoveLine이 success처럼 반환하지만 실제 관절 변화가 없는 원인은 미해결이다.
- SafeGrasp/gripper state timeout이 여전히 간헐적으로 발생한다.
- NW 검증 중 일부 target은 높은 leaf/top 후보일 수 있어, perception target
  품질을 사람이 화면으로 함께 확인해야 한다.
- SW regression 테스트는 아직 이번 변경 후 재실행하지 않았다.

## 바로 다음 의사결정

1. 이번 measured TCP probing 수정으로 `150/130mm` 접근 자세가 선택되는지 확인
2. 그래도 `90mm` 이하만 가능하면 NW는 measured TCP direct approach를 중단하고,
   SW와 동일한 `legacy_160mm` policy로 비교 실행
3. MoveLine no-motion이 계속 나오면 Doosan native MoveLine fallback을 신뢰하지
   말고, final approach 전체를 cuRobo reachable pose 또는 다른 taught approach
   branch로 재설계
4. SW 셀에서 기존 동작이 깨지지 않았는지 별도 regression run 수행
