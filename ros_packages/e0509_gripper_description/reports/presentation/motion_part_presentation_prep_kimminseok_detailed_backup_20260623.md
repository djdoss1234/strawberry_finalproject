# 딸기 수확 로봇 - 모션 파트 발표 준비 상세본

> 용도: 최신 PPT 페이지별 발표본과 별도로, 모듈 입출력/노드 구조/질문 대비를 길게 보존한 상세 참고 파일
> 기준: 실제 프로젝트 로컬 코드와 2026-06-22~23 발표 준비 내용

---

## 핵심 메시지

> 인식만으로는 수확이 끝나지 않는다. 3D target을 실제 로봇 동작으로 바꾸는 과정에서 접근 방향, 깊이, 관절 branch, 주변 줄기 간섭이 성패를 결정한다.

정상 노출 딸기에서는 현재 rule-based motion으로 연속 수확 및 place까지 가능함을 확인했다. 반면 잎/줄기 가림, Y자 줄기, 군집 딸기에서는 target이 보이더라도 줄기 방향과 주변 간섭을 motion layer가 충분히 반영하지 못해 실패가 많았다.

---

## 외워둘 숫자

| 항목 | 값 | 비고 |
|---|---:|---|
| 정상 노출 딸기 반복 실험 | 10회 x 3개 = 30개 | 수기 라벨 기준 |
| 성공 | 27/30개 | 성공률 90.0% |
| 실패 | 3/30개 | 실패율 10.0% |
| 회당 평균 성공 개수 | 2.7/3개 | 10회 평균 |
| pick sequence 평균 | 약 34.2초/개 | `/pick_pose` trigger -> `PICK COMPLETE` 로그 기준 |
| 3개 run 평균 | 약 118초/run | 3개 정상 완료 run 기준 |
| final straight approach 평균 | 약 3.7초 | 직선 진입 자체 |
| 접근-파지 구간 개선 | 36.4초 -> 15.1초 | 후보 탐색/IK/대기시간 최적화 |
| 대표 성공 | 정상 노출 딸기 3개 연속 pick-place | 영상 확보 |

주의:

- 90%는 전체 환경 성공률이 아니라 정상 노출 조건 반복 실험 결과다.
- `pick_complete`는 성공률이 아니라 sequence 종료 신호다.
- SafeGrasp/current 기반 성공 판정은 아직 완전 신뢰 단계가 아니다.
- VLA는 적용 완료가 아니라 rule-based 한계를 보완할 다음 방향이다.

---

## 발표 스크립트 요약

### 비전 -> 모션 경계

```text
비전 결과는 처음에는 이미지 안의 딸기와 keypoint입니다.
모션에서는 이것을 바로 쓸 수 없고, RealSense depth와 eye-in-hand 좌표 변환을 거쳐
로봇 base 기준 3D target으로 바꿔야 합니다.

현재는 strawberry_fusion_node가 RGB-D와 joint state를 이용해 안정적인 pick 후보를 만들고,
scan_executor_node가 그 후보를 /dsr01/curobo/pick_pose로 planner에 넘깁니다.
이후 curobo_planner_node가 접근 경로, 직선 진입, 파지, 후퇴, place까지 실행합니다.
```

짧은 파이프라인:

```text
RGB-D detection
 -> 3D target fusion
 -> scan executor target selection
 -> cuRobo pre-approach planning
 -> Doosan straight approach / gripper sequence
 -> retreat / place / next scan
```

### 왜 모션이 어려웠나

```text
딸기를 인식했다고 바로 딸 수 있는 것이 아니었습니다.
실제 실패는 target이 보였는데도 줄기 방향이 틀리거나,
깊이가 1~2cm 모자라거나, 주변 줄기와 같이 잡히거나,
특정 관절 branch에서 직선 진입이 막히는 식으로 발생했습니다.

그래서 모션 파트의 핵심은 '어디에 있다'가 아니라
'어떤 자세로, 얼마나 깊게, 어떤 경로로 들어가야 안전하게 잡히는가'였습니다.
```

핵심 키워드:

- target 좌표 정확도
- stem direction과 gripper orientation
- final approach depth
- IK branch / joint limit
- 주변 줄기/잎 collision 미반영
- 실패 후 재스캔 / 다음 target 전환

---

## 파지 모션 변화

| 방식 | 장점 | 문제 | 결론 |
|---|---|---|---|
| Spline 후 바로 close | 구현 단순 | 줄기 정렬 전 닫힘, 빈 파지 | 실패 많음 |
| Spline + 직선 진입 후 close | 접근 방향 개선 | 15도 기울어진 자세/깊이 오차, 후퇴 불안정 | 부분 개선 |
| 직선 진입 + open descent + close + pull-down | 위치 오차 흡수, 줄기 감싸기 | 가림/군집에서는 여전히 한계 | 현재 채택 |

발표 멘트:

```text
처음에는 spline으로 target까지 접근한 뒤 바로 그리퍼를 닫았습니다.
하지만 줄기와 그리퍼가 정렬되기 전에 닫히면서 줄기를 치거나 빈 파지가 발생했습니다.

두 번째는 spline 후 직선 진입을 추가했습니다.
접근 방향은 개선됐지만, 여전히 그리퍼가 약 15도 기울어진 branch를 선택하거나,
파지 후 후퇴 과정에서 놓치는 문제가 있었습니다.

최종적으로는 열린 그리퍼로 줄기 근처까지 접근한 뒤,
KP1 부근에서 닫고 아래로 당겨 분리하는 구조를 사용했습니다.
```

---

## 성공 사례 설명

```text
주변 잎과 줄기 간섭이 적고, 줄기 방향이 명확한 정상 노출 딸기에서는
현재 rule-based motion으로 연속 pick-place가 가능함을 확인했습니다.

정상 노출 딸기 30개 중 27개를 수확했으며,
3개 연속 pick-place 시퀀스도 영상으로 확보했습니다.
이는 3D target 변환, cuRobo 기반 pre-approach, 직선 진입,
open descent, pull-down detach, tray place까지 전체 파이프라인이
연결되어 동작함을 보여줍니다.
```

성과:

- 정상 노출 딸기 27/30 성공
- 3개 연속 pick-place 시연
- tray place까지 연결
- runtime JSONL 기반 시간 측정 가능

---

## 실패 케이스 분석

| 케이스 | 관찰된 실패 | 원인 | 다음 개선 |
|---|---|---|---|
| 꺾인 줄기 / 대각선 줄기 | 옆으로 빗겨 접근 | fusion이 보낸 stem orientation을 planner가 충분히 활용하지 못함 | stem direction 기반 grasp orientation |
| 잎/줄기 가림 | keypoint depth invalid, target 불안정 | 단일 시점 depth와 keypoint가 가림에 취약 | multi-view re-scan, 후보 안정성 점수 |
| 군집 / 주변 줄기 많음 | 다른 줄기와 같이 잡힘 | 주변 얇은 줄기/잎이 collision world에 없음 | 주변 줄기 obstacle proxy |
| 높은 위치 target | 직선 진입 실패, IK_FAIL | 특정 자세에서 Cartesian line과 joint limit 충돌 | workspace-aware target filtering |
| row2 tray place | 수직 descent deviation 초과 | cuRobo joint path가 tray 수직선에서 벗어남 | Cartesian constraint 또는 tray geometry 보강 |

핵심 멘트:

```text
딸기를 봤는데도 못 따는 이유는, 3D 위치 하나만으로는
줄기 방향과 주변 간섭을 표현하기 부족하기 때문입니다.
```

---

## Rule-based Motion의 한계와 다음 방향

Rule-based 한계:

- 고정 orientation 후보만으로는 꺾인 줄기 대응이 어렵다.
- 한 시점에서 보이는 keypoint/depth는 잎 가림에 취약하다.
- 얇은 잎/줄기는 collision object로 들어가지 않아, 같이 잡히거나 밀린다.
- 실패 원인이 다양해서 모든 경우를 if문으로 막기 어렵다.

Next:

- target별 stem direction을 실제 grasp orientation에 반영
- 실패 target은 같은 자리에서 반복하지 않고 다른 후보/다른 시점으로 전환
- 주변 줄기/딸기를 간단한 obstacle로 등록
- VLA는 low-level 제어가 아니라 "이 target을 지금 따도 되는가"를 판단하는 supervisor로 사용

쉽게 말하는 버전:

```text
지금 방식은 정상 케이스를 안정적으로 따는 데는 효과가 있었지만,
줄기가 꺾이거나 가려지거나 주변 줄기가 많은 상황은 규칙만으로 다 커버하기 어렵습니다.
그래서 다음 단계는 로봇이 바로 움직이기 전에,
target의 줄기 방향과 주변 상황을 보고
'지금 따도 되는지 / 다시 봐야 하는지 / 다른 딸기를 먼저 딸지'를 판단하는 구조입니다.
```

---

## 모듈별 입출력 / 노드 구조

### 실제 실행 파이프라인

```text
RealSense RGB-D
 -> strawberry_fusion_node.py
 -> /strawberry/detection/pick_pose
 -> scan_executor_node.py
 -> /dsr01/curobo/pick_pose
 -> curobo_planner_node.py
 -> PickSequenceExecutor / FinalApproachExecutor / GraspSearchExecutor
 -> Doosan motion services + gripper_service
 -> /dsr01/curobo/pick_complete
 -> scan_executor_node.py continues or rescans
```

### 모듈 입출력 표

| 모듈 | Input | Process | Output |
|---|---|---|---|
| `strawberry_fusion_node.py` | RealSense RGB-D, `/dsr01/joint_states` | YOLO/pose/keypoint, depth, stem geometry, stable target selection | `/strawberry/detection/pick_pose`, `/strawberry/detection/scene_positions` |
| `scan_executor_node.py` | scan pose config, detection pick_pose, `/pick_complete` | scan pose 이동, dwell 중 후보 수집, 중복/실패 target 관리 | `/dsr01/curobo/pick_pose`, `/strawberry/scan/status` |
| `curobo_planner_node.py` | `/dsr01/curobo/pick_pose`, joint state, collision world | state machine wiring, planning/execution orchestration | Doosan motion service calls, `/pick_complete` |
| `GraspSearchExecutor` | target pose, grasp variant policy | cuRobo IK/depth probe, variant selection | pre-approach plan + selected approach config |
| `FinalApproachExecutor` | pre-approach pose, selected depth | straight approach, cuRobo fallback, tool finish | final grasp pose reached or abort |
| `PickSequenceExecutor` | one target PoseStamped | open, approach, close, verify, detach, retreat, place gate | complete/hold/failure result |
| `TrayPlaceExecutor` | grasp result, slot sequence, place slots | tray above/release planning, release, ascend | place complete/fail |
| `gripper_service` | set_position / get_state / SafeGrasp | gripper open/close and state read | position/current/result |

### 주요 ROS 인터페이스

Subscriptions:

- `/dsr01/joint_states` (`sensor_msgs/JointState`)
- `/strawberry/detection/pick_pose` (`geometry_msgs/PoseStamped`)
- `/dsr01/curobo/pick_pose` (`geometry_msgs/PoseStamped`)
- `/strawberry/detection/scene_positions` (`std_msgs/Float64MultiArray`)
- `/strawberry/scan/status` (`std_msgs/String`)

Publishers:

- `/dsr01/curobo/pick_pose` (`geometry_msgs/PoseStamped`)
- `/dsr01/curobo/pick_complete` (`std_msgs/Empty`)
- `/strawberry/scan/status` (`std_msgs/String`)
- `/strawberry/exploration/set_cell_state` (`std_msgs/String`)

Services:

- `/strawberry/scan/start` (`std_srvs/srv/Trigger`)
- `/dsr01/motion/move_spline_joint`
- `/dsr01/motion/move_joint`
- `/dsr01/motion/move_line`
- `/dsr01/motion/change_operation_speed`
- `/gripper_service/set_position`
- `/gripper_service/get_state`

Action:

- `/gripper_service/safe_grasp` (`dsr_gripper_tcp_interfaces/action/SafeGrasp`)

---

## 주요 파일 책임

| 파일 | 역할 |
|---|---|
| `scripts/curobo_planner_node.py` | planner node entrypoint, ROS wiring, pick/place orchestration |
| `scripts/pick_sequence_executor.py` | target 1개에 대한 전체 pick sequence 실행 |
| `scripts/final_approach_executor.py` | final approach 거리 계산, 직선 진입, fallback |
| `scripts/grasp_search_executor.py` | grasp variant/depth probe 탐색 |
| `scripts/grasp_candidate_policy.py` | 후보 선택/tie-break 정책 |
| `scripts/open_stem_descent_policy.py` | open 상태에서 KP1까지 내려가는 거리 계산 |
| `scripts/tray_place_executor.py` | tray above/release/place 실행 |
| `scripts/gripper_client.py` | gripper set_position/get_state/SafeGrasp 호출 |
| `scripts/doosan_motion_client.py` | Doosan motion service wrapper |
| `scripts/curobo_planning_adapter.py` | cuRobo planning 호출과 fail logging |
| `scripts/scene_obstacle_manager.py` | neighbor obstacle/world update 관리 |
| `scripts/strawberry_fusion_node.py` | RGB-D/keypoint/depth 기반 target fusion |
| `src/strawberry_motion/execution/scan_executor_node.py` | scan pose 순회, 후보 수집, pick trigger |

---

## 예상 질문 & 답변

### Q1. 시뮬이 잘 안 됐다며? 코드만 있으면 되는 거 아닌가?

```text
코드만으로는 부족합니다.
실제 시스템은 카메라 depth, hand-eye 좌표, Doosan motion service,
gripper service, collision world가 동시에 맞아야 동작합니다.

특히 이 프로젝트는 단순 팔 이동이 아니라 줄기를 감싸고 당겨 분리하는 manipulation이라,
시뮬에서는 얇은 줄기/잎 접촉, gripper 접촉, depth noise까지 다시 구성해야 합니다.
따라서 시뮬은 후속 검증 환경으로 준비하고,
실제 로봇 실험에서는 runtime log와 영상으로 실패 원인을 먼저 분석했습니다.
```

### Q2. 모션 플래너가 로봇 구조를 모르는 것 아닌가?

```text
오히려 반대입니다.
cuRobo는 URDF와 joint limit 기반으로 IK와 collision을 계산하고,
안 되는 자세는 IK_FAIL이나 collision으로 거부합니다.

문제는 planner가 로봇을 모르는 것이 아니라,
target pose 품질, final approach에서의 branch 선택,
그리고 Doosan native MoveLine과 cuRobo planning 사이의 이음새입니다.
```

### Q3. 왜 cuRobo와 Doosan native motion을 섞었나?

```text
cuRobo는 후보 pose의 IK와 collision-aware pre-approach를 계산하는 데 유리합니다.
반면 줄기 직선 진입, 짧은 하강, release 같은 구간은 실제 제어기에서 직선성이 중요해서
Doosan MoveLine을 함께 사용했습니다.

다만 hybrid 구조에서는 branch mismatch가 생길 수 있어,
후속 개선은 final approach까지 더 일관된 Cartesian constraint로 묶는 방향입니다.
```

### Q4. 파지 성공은 자동으로 판단하나?

```text
현재는 gripper position 기반 판정과 SafeGrasp current 기반 판정을 실험했습니다.
하지만 실기에서는 position이 실제 파지 상태와 어긋나거나 serial read가 불안정한 경우가 있어,
아직 완전 자동 성공 판정이라고 말하기는 어렵습니다.

그래서 이번 발표에서는 영상/육안 라벨과 runtime log를 함께 사용했고,
자동 KPI는 보완 중인 항목으로 설명하는 것이 맞습니다.
```

### Q5. 정상 조건은 90%인데 왜 가림/군집은 안 되나?

```text
정상 조건에서는 target과 줄기 위치가 명확해서 rule-based motion이 잘 맞습니다.
하지만 가림/군집에서는 target은 보여도 파지점 depth가 불안정하거나,
주변 줄기가 collision world에 없어서 같이 잡히는 문제가 발생합니다.

즉 실패 원인은 인식 여부 하나가 아니라,
줄기 방향, 접근 깊이, 주변 간섭, 관절 branch가 함께 맞지 않기 때문입니다.
```

### Q6. VLA는 어디에 쓰려는 건가?

```text
VLA가 로봇 trajectory를 직접 만드는 방향은 아닙니다.
low-level motion은 기존 planner와 controller가 담당하고,
VLA는 현재 target을 따도 되는지, 다시 봐야 하는지,
다른 target을 먼저 따야 하는지를 판단하는 supervisor로 쓰는 방향입니다.
```

---

## 발표 톤 가이드

좋은 프레임:

```text
정상 노출 조건에서는 실제 로봇 pick-place까지 연결했다.
하지만 농장형 조건에 가까워질수록 인식만으로는 부족했고,
실패 원인을 줄기 방향, 접근 깊이, 주변 간섭, 관절 branch로 분해했다.
```

피해야 할 표현:

- 전체 자동 수확 완성
- 모든 딸기 조건에서 90% 성공
- VLA 적용 완료
- SafeGrasp 자동 판정 완료
- 충돌 회피 완성
