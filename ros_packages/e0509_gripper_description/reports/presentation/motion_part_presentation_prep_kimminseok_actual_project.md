# 딸기 수확 로봇 - 모션 파트 발표 준비 (김민석, 최신 PPT 기준)

기준 PPT:

```text
/home/user/Downloads/딸기 수확 로봇 프로젝트 - 정밀 제어를 활용한 스마트 농업 솔루션(3).pptx
```

핵심 메시지:

> 딸기를 인식하는 것만으로는 수확이 끝나지 않는다. 실제 수확 성공은 줄기 방향, 접근 깊이, 주변 잎/줄기 간섭, 로봇 관절 자세를 함께 만족해야 가능하다.

발표에서 절대 조심할 것:

- 특정 구역 이름(SW/NW 등)을 성능 지표처럼 직접 말하지 않는다.
- "정상 노출 조건", "가림/군집 조건", "높은 위치 조건"처럼 상황 중심으로 설명한다.
- `pick_complete`는 성공 신호가 아니라 sequence 종료 신호다.
- VLA는 아직 적용 완료가 아니라, rule-based 한계를 보완할 다음 방향이다.

---

## 0. 숫자 치트시트

| 항목 | 발표용 표현 | 비고 |
|---|---:|---|
| 정상 노출 딸기 반복 검증 | 10회 반복, 총 30개 | 회당 정상 딸기 3개 |
| 정상 노출 딸기 성공 | 27/30개 | 성공률 90.0% |
| 회당 평균 성공 개수 | 2.7/3개 | 수기 라벨 기준 |
| pick sequence 평균 | 약 34.2초/개 | `/pick_pose` trigger -> `PICK COMPLETE` 로그 기준 |
| 3개 연속 작업 평균 | 약 118초/run | 3개 정상 완료 run 기준 |
| final straight approach 평균 | 약 3.7초 | 직선 진입 구간 자체 |
| 접근-파지 구간 개선 | 36.4초 -> 15.1초 | 후보 탐색/IK/대기시간 최적화 효과 |
| 대표 성공 시연 | 정상 노출 딸기 3개 연속 pick-place | 영상 확보 |
| 로봇/환경 | Doosan E0509, ROS2 Humble, RealSense RGB-D, RH-P12-RN(A), cuRobo | 실제 프로젝트 기준 |

발표 문장:

```text
정상 노출 조건에서는 10회 반복, 총 30개 중 27개를 수확해 90.0% 성공률을 보였다.
다만 이는 전체 환경 성공률이 아니라, 주변 간섭이 적고 줄기 방향이 명확한 조건에서의 결과다.
```

---

## Slide 18 - 모션 플래닝 파트 도입

현재 슬라이드:

```text
03 모션 플래닝
```

발표 멘트:

```text
이제 모션 플래닝 파트입니다.
비전이 딸기를 찾아도 로봇이 실제로 수확하려면,
그 위치로 안전하게 접근하고, 줄기를 감싸 잡고, 아래로 당겨 분리한 뒤,
다시 배치 위치로 이동해야 합니다.

저는 이 과정에서 target을 실제 로봇 동작으로 바꾸는 부분,
즉 scan, path planning, final approach, grasp, retreat, place를 담당했습니다.
```

핵심 키워드:

- 3D target
- cuRobo planning
- Doosan MoveSplineJoint / MoveLine
- open descent grasp
- detach pull-down
- place sequence

---

## Slide 19 - 실험 대상 상황별 정의

현재 슬라이드:

```text
MOTION: 실험 대상 상황별 정의
1. 딸기 On position
2. 딸기 Occlusion
3. 딸기 Cluster
```

슬라이드에 넣을 짧은 문구:

```text
실제 농장 환경에서는 딸기가 항상 깔끔하게 노출되어 있지 않음

대표 상황
1. 정상 노출: 줄기와 과실이 명확하게 보이는 경우
2. 가림: 잎/줄기 때문에 파지점이 부분적으로 가려진 경우
3. 군집: 여러 딸기와 줄기가 가까이 붙어 있는 경우
```

발표 멘트:

```text
실험은 크게 세 가지 상황을 기준으로 봤습니다.
첫 번째는 주변 간섭이 적고 줄기가 잘 보이는 정상 노출 딸기입니다.
두 번째는 잎이나 줄기 때문에 파지점이 가려진 경우입니다.
세 번째는 딸기와 줄기가 서로 가까이 붙어 있어서 그리퍼가 다른 줄기를 같이 잡을 수 있는 군집 상황입니다.

중요한 점은, 같은 딸기 인식 결과라도 모션 난이도는 이 세 상황에서 완전히 달라진다는 것입니다.
```

질문 대비:

```text
Q. 왜 이 세 상황으로 나눴나?
A. 모션 실패 원인이 크게 target이 명확한지, 파지점이 가려졌는지, 주변 줄기와 간섭되는지로 갈렸기 때문이다.
```

---

## Slide 20 - 4분할 영역 스캔 구조 구성

현재 슬라이드:

```text
MOTION: 4분할 영역 스캔 구조 구성
SW / NW / NE / SE
```

주의:

- 발표에서 구역명을 성능 결과와 직접 연결하지 않는다.
- "작업 영역을 여러 관측 자세로 나눴다" 정도로 설명한다.

슬라이드 문구 수정안:

```text
목적
- 한 시야에서 모든 딸기를 안정적으로 보기 어려움
- 위치에 따라 가림, 깊이 오차, 접근 가능성이 달라짐
- 따라서 작업 영역을 여러 관측 자세로 나누어 순차 관측

현재 구현
- 각 관측 자세에서 detection 후보 수집
- 안정적인 target 후보를 선별
- pick motion으로 연결
- pick 이후 필요 시 재스캔하여 남은 target 갱신
```

발표 멘트:

```text
한 번의 카메라 자세로 전체 작업 영역을 안정적으로 보기 어렵기 때문에,
작업 영역을 여러 관측 자세로 나누어 스캔했습니다.
각 자세에서 보이는 딸기 후보를 수집하고,
그중 depth와 keypoint가 안정적인 target만 모션 플래너에 전달합니다.

초기에는 여러 자세를 모두 돌면서 후보를 모으는 방식도 시도했지만,
관측 수가 늘어도 실제 pick 정확도가 좋아지는 것은 아니었습니다.
그래서 최종적으로는 '많이 보는 것'보다 '실제로 딸 수 있는 안정 후보를 고르는 것'이 중요하다고 판단했습니다.
```

질문 대비:

```text
Q. 여러 번 보면 무조건 좋아지는 것 아닌가?
A. 아니었다. raw detection은 늘 수 있지만, 자세가 바뀌면 target 좌표와 접근 branch도 바뀌어 오히려 pick 정확도가 떨어지는 경우가 있었다.
```

---

## Slide 21 - 기존 파지 모션의 한계

현재 슬라이드:

```text
① Spline 후 바로 파지
② Spline + 직선진입 후 바로 파지
```

슬라이드 표 수정안:

| 구분 | 모션 특징 | 문제점 |
|---|---|---|
| Spline 후 바로 파지 | 목표점까지 곡선 접근 후 즉시 close | 줄기 정렬 전 파지, 빈 파지/옆 파지 발생 |
| Spline + 직선진입 후 바로 파지 | 목표점 근처까지 spline, 이후 직선 접근 후 close | 접근은 개선됐지만 파지 안정성과 후퇴가 불안정 |

추가로 말할 점:

```text
두 방식 모두 일부 케이스에서는 그리퍼가 수평이 아니라 약 15도 기울어진 자세로 접근했다.
이 경우 줄기를 감싸기보다 위쪽 또는 옆쪽으로 빗겨 잡는 문제가 있었다.
```

발표 멘트:

```text
처음에는 목표점까지 spline으로 접근한 뒤 바로 그리퍼를 닫았습니다.
하지만 줄기와 그리퍼가 정렬되기 전에 닫히면서 줄기를 치거나 빈 파지가 발생했습니다.

두 번째로는 spline 후 직선 진입을 추가했습니다.
접근 방향은 개선됐지만, 여전히 그리퍼가 기울어진 branch를 선택하거나,
파지 후 후퇴 과정에서 놓치는 문제가 있었습니다.

그래서 단순히 목표점에 도달하는 것보다,
줄기를 안정적으로 감싸는 파지 시퀀스가 필요하다고 판단했습니다.
```

---

## Slide 22 - 파지 모션 변화

현재 슬라이드:

```text
열린 그리퍼 상태로 줄기 근처까지 하강
KP1 부근에서 close
줄기를 감싼 뒤 아래로 당겨 분리
```

슬라이드 문구 수정안:

```text
현재 모션
1. cuRobo로 pre-approach 계산
2. 줄기 방향으로 직선 진입
3. 그리퍼 open 상태 유지
4. KP1 근처까지 짧게 하강
5. 줄기 위치에서 close
6. BASE -Z 방향으로 당겨 분리
7. 후퇴 후 place 또는 다음 scan
```

현재 모션 선택 이유:

```text
- 한 점을 정확히 찍는 방식보다 줄기를 감싸는 방식이 위치 오차에 강함
- open 상태로 내려오기 때문에 줄기 주변을 자연스럽게 포획 가능
- close 후 아래로 당겨 detach 하므로 단순 close보다 분리 성공률이 높음
```

발표 멘트:

```text
최종적으로 선택한 방식은 열린 그리퍼로 줄기 근처까지 접근한 뒤,
KP1 부근에서 닫고 아래로 당겨 분리하는 구조입니다.

이 방식은 파지점 한 점을 완벽히 맞히는 방식이 아니라,
줄기 주변을 감싸며 잡는 방식이라 몇 mm 정도의 target 오차에 더 강했습니다.
정상 노출 딸기에서는 이 구조가 가장 안정적으로 동작했습니다.
```

---

## Slide 23 - 성공 사례 분석

현재 슬라이드:

```text
MOTION: 성공 사례 분석
```

역할:

- 대표 성공 run 설명
- 그래프는 "정상 노출 딸기 3개 연속 성공 run"의 시간 분석으로 설명
- 전체 환경 성공률처럼 말하지 않는다.

슬라이드 본문 수정안:

```text
주변 간섭이 적고 줄기 방향이 명확한 경우,
현재 모션 구조로 연속 수확 및 place까지 가능함을 확인

성공 조건
- 타겟과 실제 줄기 위치가 명확함
- 잎/주변 줄기 간섭이 적음
- 그리퍼 접근 방향과 줄기 방향이 크게 어긋나지 않음

대표 성과
- 정상 노출 딸기 3개 연속 pick-place sequence 완료
- 격자 tray place 3/3 완료
- 접근-파지 구간 36.4초 -> 15.1초
- 약 58.6% 단축

시간 단축은 모션 형태 변경보다
후보 탐색, IK 계산, 대기시간을 줄인 최적화의 결과
```

23페이지 그래프 설명:

```text
이 그래프는 정상 노출 딸기 3개를 연속 수확한 대표 run의 시간 분해입니다.
전체 환경 통계라기보다, 현재 motion pipeline이 정상 조건에서 pick-place까지 연결될 수 있음을 보여주는 예시입니다.
```

발표 멘트:

```text
정상 노출 딸기에서는 현재 모션 구조로 3개 연속 pick-place까지 완료했습니다.
여기서 중요한 점은 시간 단축이 단순히 모션 형태를 바꿔서 생긴 게 아니라는 점입니다.
후보 탐색 순서를 정리하고, IK seed와 불필요한 대기시간을 줄이면서 접근-파지 구간을 36.4초에서 15.1초까지 줄였습니다.
```

주의:

- `전체 task 평균 약 61.7초` 문구는 최신 반복 평균과 섞이면 혼란이 생길 수 있다.
- 공간이 부족하면 23페이지에서는 61.7초를 빼고, 24페이지에서 반복 결과를 말한다.

---

## Slide 24 - 복잡한 줄기/잎 환경에서의 수확 실패 분석

현재 슬라이드:

```text
MOTION: 복잡한 줄기/잎 환경에서의 수확 실패 분석
```

역할:

- 모든 딸기를 시도했을 때 성공이 어떤 조건에 집중됐는지 설명
- 10회 반복 표 삽입
- 가림/군집/높은 위치는 정량 성공률보다 실패 원인 중심으로 설명

### 24페이지에 넣을 핵심 표

추천 표:

| 딸기 상태 | 반복 시도 결과 | 주요 실패 원인 | 개선 방향 |
|---|---|---|---|
| 줄기가 곧고 주변 간섭이 적은 딸기 | 10회 반복, 총 30개 중 27개 성공 | target과 실제 줄기 위치가 잘 맞는 경우 안정적 | 현재 모션 유지 + 반복 검증 |
| Y자/꺾인 줄기 | 빗겨 접근하거나 줄기 위쪽을 놓침 | 실제 줄기 방향을 gripper 각도에 충분히 반영하지 못함 | stem direction 기반 grasp orientation |
| 주변 줄기/딸기가 겹친 경우 | 다른 줄기와 같이 잡거나 파지 실패 | 주변 얇은 줄기/잎이 collision world에 없음 | 주변 줄기/딸기를 obstacle로 반영 |
| 잎/줄기 가림 또는 높은 위치 | 인식은 되어도 직선 진입/IK 실패 | depth/keypoint 불안정, 관절 branch 제약 | multi-view 재관찰 + 도달 가능 target 우선 |

### 24페이지 작은 KPI 박스

```text
정상 노출 딸기 반복 검증
- 10회 반복
- 회당 정상 딸기 3개
- 총 30개 중 27개 성공
- 성공률 90.0%
- 평균 2.7개/회 수확
```

### 하단 결론 문구

```text
결론: “딸기를 인식했다”는 것만으로는 수확 성공을 보장할 수 없음
실패는 주로 줄기 방향, 접근 깊이, 주변 간섭, 관절 branch 제약에서 발생
```

발표 멘트:

```text
24페이지는 조건별 실패 분석입니다.
모든 후보를 같은 모션 구조로 시도했을 때, 성공은 대부분 줄기가 곧고 주변 간섭이 적은 딸기에 집중됐습니다.
이 조건에서는 10회 반복, 총 30개 중 27개를 성공했습니다.

반대로 Y자처럼 갈라진 줄기나 주변 줄기가 겹치는 경우에는,
딸기가 인식되어도 gripper가 줄기 방향을 제대로 따라가지 못하거나 다른 줄기와 같이 잡는 문제가 발생했습니다.
또 잎이나 줄기에 가려진 딸기는 keypoint와 depth가 불안정했고,
높은 위치의 딸기는 직선 진입 과정에서 IK나 관절 branch 문제가 발생했습니다.

즉 실패 원인은 단순한 인식 실패가 아니라,
줄기 방향, 접근 깊이, 주변 간섭, 관절 자세가 함께 맞지 않았기 때문입니다.
```

주의:

- "SW에서 성공했다"라고 말하지 말고 "정상 노출 조건에서 성공했다"라고 말한다.
- "NW를 10번 돌렸다"처럼 말하지 않는다. 실제로는 모션 실패가 많아 정량 반복 검증 전 단계다.

---

## Slide 25 - 룰베이스 모션의 한계

현재 슬라이드:

```text
MOTION: 룰베이스 모션의 한계
딸기 Cluster / Occlusion
```

슬라이드 문구 수정안:

```text
Rule-based motion의 한계
- 꺾인 줄기에서는 gripper가 줄기 방향을 따라가지 못하고 옆으로 빗겨감
- 잎/줄기에 가려진 경우 파지점 좌표와 depth가 불안정함
- 주변 줄기와 딸기가 가까우면 다른 줄기까지 함께 잡는 문제가 발생
- 모든 실패 상황을 고정 규칙으로 처리하기 어려움

개선 방향
- 실제 줄기 방향을 반영한 gripper 각도 생성
- 가려진 target은 다른 각도에서 재관찰
- 주변 줄기/딸기를 obstacle로 반영
- 실패 로그 기반으로 안정적인 target 우선 선택
- gripper 위치/전류 기반 파지 성공 판정 보정
```

발표 멘트:

```text
정리하면 rule-based motion은 정상 조건에서는 꽤 안정적이었지만,
가림이나 군집처럼 상황 판단이 필요한 조건에서는 한계가 분명했습니다.

그래서 다음 단계는 단순히 offset을 더 넣는 것이 아니라,
실제 줄기 방향을 gripper orientation에 반영하고,
가려진 target은 다른 각도에서 다시 보고,
주변 줄기와 딸기를 obstacle로 넣어 planning 단계에서 반영하는 것입니다.
```

VLA로 넘기는 연결 멘트:

```text
이 지점에서 VLA가 필요한 이유가 생깁니다.
VLA가 trajectory를 직접 만드는 것이 아니라,
현재 target이 따기 좋은지, 다시 봐야 하는지, 다른 target을 먼저 따야 하는지를 판단하는 supervisor 역할을 하는 방향입니다.
```

---

## Slide 26 - VLA 기반 파지 보완 연결 멘트

현재 슬라이드:

```text
04 VLA 기반 파지 보완
```

모션 파트에서 넘기는 멘트:

```text
모션 파트에서 확인한 한계는, 규칙 기반으로 모든 딸기 상태를 다 처리하기 어렵다는 점입니다.
특히 가림, 군집, 꺾인 줄기처럼 상황 해석이 필요한 경우에는
단순 좌표와 고정 grasp rule만으로는 부족했습니다.

그래서 다음 파트에서는 이런 상황에서 target을 다시 볼지,
지금 따도 되는지, 어떤 후보를 우선할지 판단하는 VLA 기반 보완으로 넘어갑니다.
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
| `strawberry_fusion_node.py` | RealSense RGB-D, `/dsr01/joint_states` | keypoint/depth/stem geometry 기반 안정 target 생성 | `/strawberry/detection/pick_pose`, `/strawberry/detection/scene_positions` |
| `scan_executor_node.py` | scan pose config, detection pick_pose, `/pick_complete` | 관측 자세 이동, 후보 수집, target 순차 전달 | `/dsr01/curobo/pick_pose`, `/strawberry/scan/status` |
| `curobo_planner_node.py` | `/dsr01/curobo/pick_pose`, joint state, collision world | planning/execution state machine 연결 | Doosan motion service calls, `/pick_complete` |
| `GraspSearchExecutor` | target pose, grasp variant policy | cuRobo IK/depth probe, variant 선택 | pre-approach plan |
| `FinalApproachExecutor` | pre-approach pose, selected depth | straight approach, fallback, tool finish | final grasp pose reached or abort |
| `PickSequenceExecutor` | one target PoseStamped | open, approach, close, verify, detach, retreat, place gate | complete/hold/failure |
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

## 예상 질문 & 답변

### Q1. 시뮬이 잘 안 됐다며? 코드만 있으면 되는 거 아닌가?

```text
코드만으로는 부족합니다.
실제 시스템은 카메라 depth, eye-in-hand 좌표, Doosan motion service,
gripper service, collision world가 동시에 맞아야 동작합니다.

특히 이 프로젝트는 단순 이동이 아니라 줄기를 감싸고 당겨 분리하는 manipulation이라,
시뮬에서는 얇은 줄기/잎 접촉, gripper 접촉, depth noise까지 다시 구성해야 합니다.
따라서 시뮬은 후속 검증 환경으로 준비하고, 실제 로봇 실험에서는 runtime log와 영상으로 실패 원인을 먼저 분석했습니다.
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
VLA는 현재 target을 따도 되는지, 다시 봐야 하는지, 다른 target을 먼저 따야 하는지를 판단하는 supervisor로 쓰는 방향입니다.
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

마무리 멘트:

```text
이번 모션 파트의 성과는 정상 노출 딸기에서 전체 pick-place 파이프라인을 실제 로봇으로 연결하고,
실패 케이스를 원인별로 분해한 것입니다.
다음 단계는 stem direction 기반 grasp orientation, multi-view 재관찰,
주변 줄기 obstacle 반영, 그리고 VLA supervisor로 확장하는 것입니다.
```
