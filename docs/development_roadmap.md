# 전체 개발 순서 및 실행 Roadmap

## 0. 이 문서의 기준

이 프로젝트에서 본인의 핵심 담당은 **딸기 수확 전체 motion system**입니다.
팀원은 복잡한 장면에서의 VLA 판단을 담당하고, 양쪽은 정의된 interface로
통합합니다.

따라서 진행 순서는 다음 원칙을 따릅니다.

- 먼저 반복 실행 가능한 motion baseline을 만듭니다.
- 그 다음 환경 변화와 실패 상황에 대응하도록 고도화합니다.
- VLA는 motion 실행 구조가 안정된 뒤 연결합니다.
- 성공 횟수만 기록하지 않고 실패 원인과 조건도 함께 기록합니다.

최종 목표:

> 실제에 가까운 딸기 모형, 잎/줄기/장애물, 이동 가능한 수확 tray 환경에서
> 딸기 target을 받아 안전하게 접근, 파지, 이동, 자동 배치하고, 이후
> 팀원의 VLA 판단 결과와 연동해 복잡 장면의 수확 demo까지 완성한다.

---

## 1단계. 프로젝트 기준 확정 및 협업 약속

### 목적

구현을 시작하기 전에 무엇을 성공으로 볼지, 어느 부분을 누가 책임질지,
모션 시스템과 VLA가 어떤 데이터로 연결될지를 정합니다.

### 본인 할 일

- 실제 사용할 로봇, gripper, camera, PC/GPU, ROS 2 환경 버전을 기록합니다.
- 모션 담당 범위를 `approach -> grasp -> retreat -> transfer -> place -> recovery`로 확정합니다.
- 최종 demo 시나리오와 최소 성공 조건을 정합니다.
- 평가 지표와 failure code 초안을 확정합니다.

### 팀원과 같이 정할 일

- VLA가 넘겨줄 정보의 최소 형식:
  - `target_id`
  - target pose 또는 target region
  - action 제안: `PICK`, `SKIP`, `REOBSERVE`
  - confidence/risk 정보
- 모션 측이 돌려줄 결과 형식:
  - `SUCCESS`, `INVALID_TARGET`, `PLANNING_FAIL`, `COLLISION_RISK`,
    `GRASP_FAIL`, `PLACE_FAIL`, `REOBSERVE_REQUIRED`
- 실험 영상, log, issue 기록 방식

### 산출물

- `docs/project_scope.md` 보완
- `docs/interfaces.md` 초안
- `docs/experiment_protocol.md` 초안

### 완료 기준

- 본인과 팀원이 담당 경계와 interface 초안에 동의함
- 성공/실패를 같은 용어로 기록할 수 있음
- 최종 demo가 한 문장으로 설명됨

---

## 2단계. 실제형 테스트베드와 계측 환경 구축

### 목적

알고리즘을 개선하기 전에 매번 비슷한 조건에서 반복 실험할 수 있는 물리
환경을 만듭니다.

### 본인 할 일

- 실제에 가까운 딸기 모형을 준비합니다.
  - 크기와 높이가 다른 딸기
  - 좌우 끝, 중앙, 깊이 차이가 있는 배치
  - 잎/줄기 또는 단순 장애물이 있는 배치
- 이동 가능한 계란판/수확 tray를 준비합니다.
- tray에 AprilTag 또는 ArUco marker를 부착할 위치를 설계합니다.
- 작업대, 재배판, tray, camera의 기준 위치와 치수를 기록합니다.
- 속도 제한, emergency stop 확인, 충돌 위험 구간을 정합니다.

### 권장 실험 Scene

| Scene | 목적 |
| --- | --- |
| S0 | 장애물 없이 딸기 1개, 고정 tray: motion 기본 확인 |
| S1 | 딸기 3개, 고정 tray: 반복 수확 확인 |
| S2 | 딸기 3개, tray 위치 변경: 자동 place 확인 |
| S3 | 잎/줄기 모형 또는 벽 장애물 포함: collision/retry 확인 |
| S4 | VLA 판단이 필요한 복잡 배치: 최종 통합 확인 |

### 산출물

- `docs/testbed_setup.md`
- scene별 사진과 치수표
- 안전 점검표
- `config/scene/`에 들어갈 환경 설정 초안

### 완료 기준

- S0~S3 환경을 다시 동일하게 구성할 수 있음
- tray marker가 camera에서 안정적으로 보임
- 로봇 저속 동작에서 물리 충돌 위험 구간을 파악함

---

## 3단계. 저장소/코드 구조와 Logging 기반 만들기

### 목적

미니프로젝트의 동작 코드를 옮기기 전에, 최종 프로젝트에서 기능과 실험
결과가 섞이지 않도록 뼈대를 만듭니다.

### 본인 할 일

- ROS package 구성 또는 workspace 배치 방식을 정합니다.
- 다음 책임을 가진 모듈 구조를 만듭니다.

```text
planning/       planner backend, collision world, trajectory 평가
task/           pick/place state machine, retry policy
tray/           tray pose, slot generation, occupancy
interfaces/     target proposal, motion result 형식
diagnostics/    log, rqt/TF/RViz 확인 도구
config/         scene, tray geometry, planner parameter
launch/         실험별 bringup
docs/           설계 결정, 실험 결과, 회고
```

- 실행마다 남길 log schema를 만듭니다.
  - `run_id`, scene, target, planner, tray pose source
  - planning/execution 시간
  - 결과 code와 실패 원인
  - 관련 영상/이미지/rosbag 참조 경로
- 실험 artifact와 calibration/weight가 public Git에 섞이지 않게 관리합니다.

### 산출물

- 초기 package skeleton
- `docs/log_schema.md`
- `config/` 및 `launch/` 기본 구조
- `rqt_graph`, TF tree, RViz 확인 절차

### 완료 기준

- 빈 pipeline 또는 mock target으로 노드 연결 구조를 확인할 수 있음
- 실험 1회를 실행하면 동일 형식의 결과 log가 남음
- 민감/대용량 artifact가 Git 추적 대상이 아님

---

## 4단계. 미니프로젝트 Motion Baseline 이식

### 목적

이미 동작했던 기능을 최종 프로젝트에서 다시 실행 가능한 최소 baseline으로
복원합니다. 이 단계에서는 기능 추가보다 동일 동작 재현이 우선입니다.

### 본인 할 일

- `strawberry_miniproject`에서 안정된 부분만 확인해 옮깁니다.
  - Doosan 실행 service 연결
  - gripper open/soft close/release sequence
  - current joint state 수신
  - cuRobo planning 호출
  - approach/grasp/retreat/place 기본 sequence
- fixed target과 fixed tray pose로 S0를 먼저 실행합니다.
- 이후 기존과 같은 taught slot 방식으로 S1을 확인합니다.
- 이식하면서 발견한 hard-coded parameter와 구조 문제를 목록화합니다.

### 하지 않을 일

- 이 단계에서 바로 VLA를 연결하지 않음
- planner를 새 알고리즘으로 갈아엎지 않음
- tray 자동 인식이 완성되지 않았는데 전체 환경을 복잡하게 만들지 않음

### 산출물

- 동작 가능한 motion baseline
- baseline 실행 launch/config
- `docs/baseline_migration.md`
- S0/S1 실험 결과

### 완료 기준

- fixed target으로 pick & place가 수행됨
- 딸기 3개 수준의 반복 실행에서 실패 위치와 원인이 기록됨
- 미니프로젝트 대비 동작이 사라지거나 악화된 부분을 설명할 수 있음

---

## 5단계. Tray Localization 및 자동 Place 구현

### 목적

현재 가장 분명한 약점인 고정 티칭 place를 없애고, 이동된 tray에 대응합니다.

### 본인 할 일

1. marker detector를 구성합니다.
   - 우선순위: AprilTag 또는 ArUco 기반 baseline
   - 입력: RGB/depth image, camera intrinsic
   - 출력: `camera_frame -> tray_frame`
2. hand-eye/TF를 이용해 `base_link -> tray_frame`을 계산합니다.
3. tray geometry config를 만듭니다.
   - 행/열 수
   - slot 간격
   - tray frame 기준 slot origin
   - `above` 높이와 `release` 높이
4. `slot_manager`가 tray pose에서 place pose를 자동 생성하도록 만듭니다.
5. tray collision object를 planning scene/curobo world에 반영합니다.
6. RGB-D 기반 slot occupancy 확인을 추가합니다.
   - 처음에는 단순 threshold 또는 영역 기반 판정도 허용
   - 이후 실제 딸기/깊이 noise에 맞춰 보정

### 실험

- tray 위치 A/B/C에서 동일한 slot place 수행
- marker가 순간적으로 보이지 않는 경우 처리 확인
- 점유된 slot을 건너뛰는 동작 확인

### 산출물

- `tray_detector` 또는 동등 기능 node/module
- `slot_manager`
- tray geometry config
- S2 실험 결과표 및 영상

### 완료 기준

- tray를 이동해도 `tray_frame` 기준 slot pose를 생성함
- A/B/C 위치에서 목표 slot place 결과를 측정함
- `TRAY_NOT_FOUND`, `TRAY_POSE_UNCERTAIN`, `SLOT_OCCUPIED`를 구분해 기록함

---

## 6단계. 수확 Motion Sequence 모듈화 및 안정화

### 목적

demo용 한 파일 중심 구조에서 벗어나, 실패를 다룰 수 있는 task-level motion
system으로 만듭니다.

### 본인 할 일

- sequence를 state machine으로 분리합니다.

```text
IDLE
  -> TARGET_VALIDATION
  -> APPROACH_PLAN
  -> APPROACH_EXECUTE
  -> GRASP_PLAN
  -> GRASP_EXECUTE
  -> GRIP
  -> RETREAT
  -> TRANSFER
  -> PLACE
  -> VERIFY
  -> SUCCESS / RECOVERY / FAIL
```

- 각 단계별 timeout, 실패 code, retry 여부를 정합니다.
- position/scene에 따라 approach orientation 또는 offset을 조절할 policy를
  분리합니다.
- gripper 성공 판정은 실제 가능한 feedback 종류를 확인한 뒤 정합니다.
- 로봇 실행 중 중단/복귀 동작과 home/safe pose를 관리합니다.

### 산출물

- `pick_place_state_machine`
- `retry_policy`
- `trajectory_executor`
- S1/S2 반복 실험 log

### 완료 기준

- 어느 단계에서 실패했는지를 log만 보고 알 수 있음
- 실패 후 무조건 전체 재시작이 아니라 정해진 recovery 동작을 수행함
- 최소 10회 이상 반복 실험을 동일 protocol로 기록함

---

## 7단계. Collision World와 장애물 환경 대응

### 목적

실제형 환경에서 딸기만 잡는 것이 아니라, tray, 벽, 잎/줄기 모형,
이미 수확한 과실과의 간섭을 줄입니다.

### 본인 할 일

- 실제 치수 기반 collision object를 정리합니다.
  - 작업대/재배판
  - tray
  - marker fixture
  - 장애물 또는 잎/줄기 근사 모델
  - placed strawberry
  - attached/held fruit
- RViz/MoveIt planning scene과 cuRobo collision world가 같은 장면을
  설명하는지 비교합니다.
- grasp 직후 held object를 붙이는 시점과 retreat 실패 여부를 검증합니다.
- S3에서 충돌 위험 경로와 안전 경로를 비교합니다.

### 산출물

- collision scene config
- scene visualization 캡처
- false collision/missed collision 사례 정리
- S3 결과

### 완료 기준

- 주요 물리 장애물이 scene model에 반영됨
- 계획 성공이 곧 실제 충돌 안전을 보장하지 않는 사례와 보완 방법을 설명할 수 있음
- 장애물 scene에서 정해진 안전 기준을 만족하는 실행 결과를 기록함

---

## 8단계. Planner 비교 분석 및 Motion 개선

### 목적

“어떤 planner가 좋다”를 말로 주장하지 않고 동일 조건에서 비교하고,
모션 담당자로서 task-specific 개선점을 도출합니다.

### 비교 대상 후보

- Doosan 기본 joint/cartesian motion
- MoveIt 기반 planner
- cuRobo 기반 planner
- 필요 시 task-specific waypoint/retry policy가 추가된 방식

### 본인 할 일

- 동일한 start joint, target pose, scene, orientation 조건을 저장합니다.
- planner별로 다음 항목을 기록합니다.

| 항목 | 내용 |
| --- | --- |
| planning 성공 여부 | 경로 생성 성공/실패 |
| planning time | 경로 생성 시간 |
| execution result | 실제 실행 성공/실패 |
| trajectory length | joint 또는 TCP 경로 길이 |
| clearance | 장애물과의 최소 거리 또는 근사 지표 |
| joint behavior | wrist flip, 큰 J1 회전, limit 근접 여부 |
| failure type | IK, collision, execution, grasp/place 실패 |

- 문제가 반복되는 target 유형을 구분합니다.
  - 좌우 workspace 끝
  - 깊은 grasp
  - 장애물 근처
  - tray까지 transfer가 긴 위치
- 알고리즘 자체를 처음부터 새로 쓰기보다 다음 개선을 실험합니다.
  - 위치별 approach waypoint/orientation
  - 안전 retreat/transfer pose
  - branch 또는 joint range 제한
  - 장애물/held object 반영
  - failure별 retry policy

### 산출물

- `docs/planner_benchmark.md`
- 동일 scene benchmark dataset/config
- motion 개선 전후 비교 결과

### 완료 기준

- 사용한 planner 선택 이유를 수치와 실패 사례로 설명할 수 있음
- 범용 planner 위에 추가한 task-specific policy의 효과를 제시할 수 있음
- 면접에서 “왜 MoveIt/cuRobo를 썼고 무엇을 직접 개선했나”에 답할 근거가 있음

---

## 9단계. VLA 팀원과의 통합

### 목적

팀원의 복잡 장면 판단을 안전한 motion execution으로 연결합니다.

### 팀원에게 받을 것

- 재현 가능한 입력 scene 또는 dataset
- target/action proposal message 형식
- confidence/risk가 의미하는 기준
- `PICK`, `SKIP`, `REOBSERVE` 판단 예시

### 본인이 구현할 것

- VLA proposal 수신 interface
- target pose가 없거나 위험한 경우 reject/재관찰 요청
- geometric validity 및 collision validation
- motion 실행 결과를 structured result로 반환
- VLA 판단 실패와 motion 실패가 섞이지 않도록 log 분리

### 통합 실험 순서

1. 팀원이 정한 target을 사람이 입력한 것처럼 mock message로 motion 실행
2. 실제 VLA output을 연결하되 로봇 실행 없이 validation/log만 수행
3. 저속 real robot 실행
4. S4 복잡 scene에서 end-to-end test

### 산출물

- versioned request/result interface
- integration launch
- 통합 실험 log와 시연 영상

### 완료 기준

- VLA가 잘못된 target을 제안해도 motion 시스템이 위험 실행을 차단함
- 실패가 VLA 판단 문제인지 motion planning/execution 문제인지 구분됨
- 복잡 scene에서 end-to-end demo를 재현할 수 있음

---

## 10단계. 최종 검증, 포트폴리오, 발표 준비

### 목적

동작 영상 한 개가 아니라, 무엇을 개선했고 어떤 조건에서 성능이 나왔는지
근거를 남깁니다.

### 반드시 확보할 결과

- S0: 단일 딸기 기본 수확 영상 및 결과
- S1: 다중 딸기 반복 수확 결과
- S2: 이동한 tray 자동 place 결과
- S3: 장애물 포함 motion 안정화 결과
- S4: VLA 연동 복잡 장면 수확 결과
- planner 비교표 및 선택 근거
- failure distribution과 개선 전후 비교

### 포트폴리오에서 강조할 내용

- 미니프로젝트에서 최종 프로젝트로 넘어오며 해결한 고정 티칭 문제
- 본인이 담당한 전체 수확 motion pipeline
- planner의 단순 적용이 아니라 collision/retry/waypoint/검증 계층을 직접 설계한 점
- VLA와 실제 로봇 execution 사이의 안전한 interface를 만든 점
- 성공뿐 아니라 실패 유형을 구조화하고 재현 가능하게 개선한 점

### 완료 기준

- 코드, 문서, 영상, 실험표가 서로 같은 version/scene을 가리킴
- 발표자가 architecture, 역할 분담, 실패 원인, 개선 선택을 설명할 수 있음
- 새로운 사람이 저장소 문서만 읽어도 demo를 재현할 방향을 파악할 수 있음

---

## 전체 순서 한눈에 보기

| 순서 | 단계 | 본인 핵심 산출물 | 팀원/VLA 연결 여부 |
| --- | --- | --- | --- |
| 1 | 목표/역할/interface 확정 | scope, interface, failure code | 협의 시작 |
| 2 | 실제형 테스트베드 구축 | scene, 치수, 안전 조건 | 불필요 |
| 3 | 코드/로그 기반 구성 | package skeleton, log schema | interface 자리만 확보 |
| 4 | Motion baseline 이식 | fixed scene pick & place | 불필요 |
| 5 | Tray 자동 place | tray localization, slot manager | 불필요 |
| 6 | Motion 안정화 | state machine, retry policy | 불필요 |
| 7 | Collision 환경 대응 | scene model, 장애물 실험 | 불필요 |
| 8 | Planner 비교/개선 | benchmark, 선택 근거 | 불필요 |
| 9 | VLA 통합 | validation interface, 통합 demo | 본격 연결 |
| 10 | 최종 검증/포트폴리오 | 영상, 결과표, 발표 자료 | 공동 결과 |

## 지금 당장 시작할 작업

현재 기준으로 바로 수행할 일은 다음 순서입니다.

1. 실제 딸기 모형과 이동형 tray의 형태, 크기, 배치를 정합니다.
2. tray에 붙일 AprilTag/ArUco 방식과 `tray_frame` 기준을 정합니다.
3. motion baseline을 최종 저장소에 어떻게 이식할지 package 구조를 확정합니다.
4. `interfaces.md`와 `experiment_protocol.md`를 먼저 작성합니다.
5. 그 다음 `tray localization -> slot 생성 -> place` 기능부터 구현합니다.

VLA 통합은 중요한 최종 방향이지만, 지금 당장 첫 코드 작업은 아닙니다.
본인의 담당 범위에서 먼저 보여줘야 하는 결과는 **tray가 움직여도 안정적으로
place할 수 있는 모션 시스템**입니다.
