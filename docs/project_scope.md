# 프로젝트 범위 및 협업 계획

## 1. 목표

실제에 가까운 딸기 모형 환경에서 반복적으로 pick & place motion을
수행하고, 이후 더 복잡한 재배 환경에서는 VLA의 high-level 판단과
연동할 수 있는 딸기 수확 로봇 시스템을 구현합니다.

첫 우선순위는 semantic complexity가 아니라, **측정 가능하고 디버깅
가능하며 안전하게 연동 가능한 모션 시스템**을 만드는 것입니다.

## 2. 역할 분담

### 모션 개발: djdoss1234

담당 내용:

- 수확 motion architecture와 구현
- 실행 가능한 harvest action에 대한 target pose 검증
- motion planning, trajectory execution, planner 평가
- grasp, retreat, transfer, place sequence 신뢰성 개선
- tray detection 연동과 자동 slot placement
- collision environment 관리
- 실행 log, failure taxonomy, recovery policy

산출물:

- 모듈화된 motion runtime code
- 재사용 가능한 motion/planner interface
- tray localization 및 자동 place demo
- benchmark 결과와 실험 log
- end-to-end motion 시연

### VLA 개발: 협업 팀원

담당 내용:

- 복잡한 장면에서의 visual-language reasoning
- occlusion 또는 clutter 상황의 후보 우선순위 판단
- 수확 가능성 및 다음 action 판단
- recovery에 도움이 되는 semantic 설명 또는 metadata

연동 산출물:

- 구조화된 target/action proposal
- confidence와 risk annotation
- 반복 평가 가능한 VLA test case

## 3. 연동 Interface 원칙

semantic 판단과 실제 하드웨어 motion을 다음과 같이 분리합니다.

```text
VLA / perception decision
  -> target proposal
  -> motion-side geometry and safety validation
  -> planner and executor
  -> structured execution result
  -> next decision
```

요청된 motion이 실제로 실행 가능하고 안전한지에 대한 최종 판단은
모션 시스템이 담당합니다.

## 4. Motion 기준 구현 개발

참조 저장소:

- https://github.com/djdoss1234/strawberry_miniproject

미니프로젝트에서 가져올 수 있는 기반:

- 딸기 target 생성 pipeline
- robot frame transform
- cuRobo planning 통합
- Doosan trajectory execution
- gripper 동작
- 티칭 기반 place workflow

최종 프로젝트에서는 안정된 기능만 선별하여, 명시적인 interface와
실험용 log 체계를 갖춘 모듈 구조로 이식합니다.

## 5. 첫 기능 목표

### 1차: Quadtree 기반 작업영역 탐색과 Scan Motion

목표:

> 재배 workspace를 quadtree cell로 관리하고, 미관찰 또는 재관찰 대상
> cell을 보기 위한 eye-in-hand camera scan pose를 생성한다.

구현 순서:

1. `workspace_frame` 기준 작업영역 크기와 최대 depth를 정의합니다.
2. cell 상태와 subdivision/revisit policy를 정의합니다.
3. RViz marker로 workspace와 cell 상태를 시각화합니다.
4. 관찰 대상 cell 중심을 기준으로 scan pose를 생성합니다.
5. planner/executor 연결 전 TF, workspace boundary, ROS topic을 검증합니다.
6. 이후 기존 detector와 motion baseline을 연결해 cell 상태를 갱신합니다.

이 기능을 먼저 선택하는 이유:

- eye-in-hand robot이 어디를 관찰할지 정하는 것이 모션 담당 범위와 직접 연결됨
- VLA와 겹치지 않게 공간 탐색과 motion target 생성 계층을 구축할 수 있음
- 이후 검출, 수확 결과, VLA 제안을 같은 workspace 상태에 반영할 수 있음

### 2차: 이동 가능한 Tray에 대한 자동 Place

수확 후 배치 단계에서는 AprilTag/ArUco 또는 RGB-D로 움직인 tray의 pose를
추정하고, `tray_frame` 기준으로 slot별 `above`/`release` pose를 자동
생성합니다. 이는 quadtree scan 및 기본 수확 motion이 연결된 뒤 진행합니다.

## 6. 평가 계획

| 지표 | 의미 |
| --- | --- |
| planning success rate | 유효 target에 대해 trajectory를 생성한 비율 |
| execution success rate | abort/collision stop 없이 motion이 완료된 비율 |
| grasp success rate | grasp 및 retreat 후 딸기를 유지한 비율 |
| place success rate | 목표 slot에 정상 배치한 비율 |
| tray relocation success | tray 위치 변경 후 place에 성공한 비율 |
| cycle time | target 승인부터 place 완료까지 걸린 시간 |
| failure distribution | failure code별 발생 횟수 |

초기 failure code:

```text
INVALID_TARGET
TF_UNAVAILABLE
TRAY_NOT_FOUND
TRAY_POSE_UNCERTAIN
SLOT_OCCUPIED
IK_FAIL
PLANNING_FAIL
COLLISION_RISK
EXECUTION_ABORT
GRASP_FAIL
PLACE_FAIL
SUCCESS
```

## 7. 초기 Sprint

1. 실제형 모형 workspace의 기준 frame과 치수를 기록합니다.
2. quadtree cell 상태, 최대 depth, scan/revisit policy를 결정합니다.
3. ROS topic/action과 experiment log schema를 정의합니다.
4. exploration, visualization, motion, planning, diagnostics 기준 package 구조를 잡습니다.
5. workspace marker와 quadtree cell visualization을 구현합니다.
6. cell 중심 기반 next scan pose 생성을 구현합니다.
7. `rqt_graph`, TF tree, RViz scene, label이 포함된 진행 결과를 저장합니다.

## 8. 이후 통합

tray 및 motion baseline을 측정 가능하게 만든 이후 다음 작업을 수행합니다.

- 동일 target/scene 입력에서 planner 동작 비교
- 실제형 장애물과 collision model 추가
- recovery/retry policy 추가
- versioned interface로 VLA target proposal 수신
- deterministic motion validation을 포함한 복잡 장면 수확 실험
