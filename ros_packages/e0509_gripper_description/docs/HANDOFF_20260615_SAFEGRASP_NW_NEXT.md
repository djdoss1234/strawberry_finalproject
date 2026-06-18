# HANDOFF - SafeGrasp 통신 확인, NW 가림 셀 모션/KPI 검증 전

기준 시각: 2026-06-16
기준 커밋: `5f949aa docs: confirm original SafeGrasp TCP bridge`

> 2026-06-17 추가: NW 실기 중 J4 equivalent rewrite, gripper service 재시작,
> 저속화, joint jog 안전 차단, KPI 적용 상태가 추가로 정리됐다. 최신 인계는
> [HANDOFF_20260617_NW_MOTION_GRIPPER_STATUS.md](HANDOFF_20260617_NW_MOTION_GRIPPER_STATUS.md)를
> 먼저 읽고, 이 문서는 SafeGrasp 배경 자료로 참고한다.

## 0. 2026-06-15 후속 미커밋 변경

SafeGrasp 통합 코드는 작성 및 빌드됐지만 실기 검증 전이다.

- `bringup.launch.py`: real mode 그리퍼 노드를 `dsr_gripper_tcp`로 교체
- `curobo_planner_node.py`: `SetPosition`, `GetState`, `SafeGrasp` action 연동
- `package.xml`: `dsr_gripper_tcp` 실행 의존성 추가
- `bringup.launch.py`: `start_gripper_service`, `gripper_init_attempts`,
  `gripper_init_timeout_sec` 인수 추가
- `prime_gripper_serial_drl.py`: 실험용 serial primer 추가

2026-06-16 기준 startup blocker의 직접 원인은 고아 `gripper_service_node`
중복 실행이었다. 2026-06-15 17:24에 실행한 노드가 터미널 종료 후에도 남아
`/gripper_service/state`를 계속 publish했고, 새 노드와 중복되면서 cached
state를 정상으로 오판했다.

현재 해결 절차는 고아 노드 종료 후 `ros2 daemon` 재시작, `Unknown topic`
확인, 단일 노드 재실행이다. 그 후 `Gripper service node ready at 20.0 Hz`,
`status_text: ok`, SafeGrasp action 발견까지 확인됐다.

남은 주의점: DRL 내부 `flange_serial_open()` block 가능성은 이론상 남아
있지만, 이번 실기 문제의 직접 원인은 아니었다.

또한 `dsr_gripper_tcp`는 시작 시 `/dsr01/system/set_robot_mode`로 autonomous
전환을 이미 요청한다. 따라서 AUTO 여부는 확인 항목이지만 단독 근본 원인으로
확정하지 않는다.

## 1. 지금 상태

### 실기 검증 완료

- SW 단일 딸기에서 검출, 수평 접근, 줄기 파지, 아래 방향 분리, 후퇴 성공 사례 확보
- 실측 TCP 모델 적용: flange에서 실제 파지 중심까지 약 `260mm`
- 파지 모션 변경:
  - 그리퍼 position `600`으로 열린 상태 유지
  - 줄기 위쪽으로 수평 진입
  - 열린 상태로 BASE `-Z 30mm` 하강하여 KP1 부근에서 close
  - BASE `-Z 40mm` detach pull
  - TOOL `-Z` retreat
- 꺾인 줄기는 KP0/KP1의 국소 방향과 midpoint를 사용하도록 target 보정
- 긴 이동은 cuRobo 계획 + MoveSplineJoint, 접촉 구간은 Doosan MoveLine으로 분리
- operation speed `100%` 강제 및 후보/대기시간 축소 코드 반영
- 계란판 Slot0, Slot1, Slot3, Slot4 place 실기 성공
- Slot2는 30도 tilt 방식으로 도달 관찰했으나 약 3cm 오차가 남음
- Slot5 row2 하강은 수직선에서 `100.8mm` 이탈하여 release 전에 안전 차단
- NW 실험 context, runtime KPI 집계, 수동 라벨, PNG/JSON/Markdown KPI 보고서 도구 구현
- 원본 `Dakae/Doosan-E0509-ROBOTIS-RH-P12-RN-TCP-Bridge`의 SafeGrasp 실기 동작 확인

### SafeGrasp에서 확인한 사실

기존 `/dsr01/gripper/read_state` 직접 경로는 `-1/-1`이었지만, 원본
`dsr_gripper_tcp`의 DRL TCP bridge 경로는 정상 동작한다.

```text
Gripper service node ready at 20.0 Hz
state: ready=true, present_position=700, present_current=8
empty SafeGrasp result: target reached without grasp
```

빈 파지 실행 중 position/current feedback가 연속 기록됐으며, 정상 조건에서는
`grasp_detected=false`를 반환한다. 단 threshold가 낮으면 빈 파지도 오검출된다.

자동 로그:

```text
logs/gripper_calibration/2026-06-15/safe_grasp_trials.jsonl
logs/gripper_calibration/2026-06-16/safe_grasp_trials.jsonl
```

2026-06-16 반복 보정 결과:

| 조건 | threshold | 결과 | 결론 |
| --- | --- | --- | --- |
| empty | 120 | `delta=147`, `detected=True` | 빈 파지 오검출 |
| empty | 220 | 대부분 `delta=0~35`, `detected=False` | 빈 파지 억제 가능 |
| stem_moru | 220 | `peak delta=199`, `detected=False` | 줄기 미검출 |
| stem_moru | 180 | 5회 중 1회 검출 | 단독 판정 불안정 |
| stem_moru | 140 | 3회 중 1회 검출 | 줄기 감지 부족 |
| empty | 140 | 3회 중 2회 오검출 | 사용 불가 |

현재 판단:

- SafeGrasp는 자동 KPI에 `present_position`, `present_current`,
  `current_delta`, `detected`, `object_lost`를 남기는 접촉 후보 신호로 사용한다.
- `grasp_detected=true`는 줄기 파지 성공이 아니라 "무언가 닿음"이다.
- 최종 수확 성공은 실험 중 사람이 직접 보고 즉시 수기 라벨로 입력한다.

팀원에게 설명할 때는 다음처럼 말한다.

```text
그리퍼 양방향 통신 자체는 살아났고 position/current 값은 읽힌다.
다만 딸기 줄기가 너무 얇아서 전류 임계값 하나만으로
"줄기를 정확히 잡았다"를 자동 확정하기에는 아직 불안정하다.
그래서 지금은 SafeGrasp를 접촉 후보/빈 파지 후보 자동 로그로 쓰고,
실제 줄기 파지·분리·유지는 실험자가 바로 보고 수기 라벨로 남긴다.
NW 검증을 하면서 SafeGrasp 값과 실제 성공 라벨을 비교해 임계값을 다시 잡을 예정이다.
```

## 2. 아직 실행하지 않은 것

다음 항목은 아직 완료되지 않았다.

- 잎/비목표 접촉 SafeGrasp 보정 시험
- 얇은 전기테이프 줄기(`stem_tape`) SafeGrasp 보정 시험
- 조건별 임계값 확정. 현재는 단일 threshold로 최종 파지 성공 판정 불가
- 변경된 `curobo_planner_node.py` SafeGrasp 경로의 실기 검증
- SafeGrasp 통합 후 실제 pick close 단계 검증
- NW 잎/줄기 가림 셀 실제 Pick
- AnyGrasp/GraspGen 설치 또는 point-cloud offline 평가

### AnyGrasp 적용 계획

AnyGrasp는 지금 당장 실기 주 경로에 넣지 않는다. 현재 병목을 먼저 데이터로
확인한다.

적용 순서:

1. NW 가림 셀에서 현재 KP1/rule 기반 접근을 3~5회 이상 실행한다.
2. 실패 원인을 `perception miss`, `줄기 방향 오차`, `잎 접촉`, `grasp miss`,
   `planning reject`로 나눠 기록한다.
3. 줄기 꺾임/가림 때문에 고정 KP1 접근이 반복 실패하면, 저장된 RGB-D/point
   cloud를 입력으로 AnyGrasp를 offline 평가한다.
4. AnyGrasp가 줄기 또는 잡기 좋은 접근 후보를 더 안정적으로 제안하는지
   기존 rule 기반 target과 비교한다.
5. offline 결과가 의미 있을 때만 ROS runtime 후보 생성기로 연결한다.

즉 AnyGrasp는 "지금 코드 대체"가 아니라, NW에서 고정 방향 접근의 한계가
수치로 확인된 뒤 적용할 후보 grasp generator다. 라이선스/SDK/CUDA 의존성이
있으므로 SDK 확보가 막히면 GraspNet-baseline/GraspGen 계열을 fallback으로
검토한다.

## 3. 원본 SafeGrasp 실행 시 주의

동시에 두 그리퍼 제어 노드를 실행하면 안 된다.

### 처음 그리퍼 패키지 연동이 안 된 이유

팀원 또는 다음 작업자에게는 다음 원인으로 설명한다.

```text
처음에는 기존 /dsr01/gripper/read_state 경로가 position/current를 -1로 반환해서
전류 기반 파지 판정이 불가능했다.

dsr_gripper_tcp 패키지로 바꾼 뒤에도 바로 정상처럼 보이지 않았던 이유는,
예전에 실행한 gripper_service_node가 고아 프로세스로 남아
/gripper_service/state를 계속 publish했기 때문이다.

그 상태에서 새 gripper_service_node를 다시 띄우면서 publisher가 2개가 됐고,
한쪽은 cached ready 상태, 다른 한쪽은 timeout/error 상태를 내보내서
ready:true인데 status_text:timed out인 모순 상태가 생겼다.

즉 패키지 자체가 안 된 게 아니라,
기존 read_state 경로 실패 + 중복/고아 gripper_service_node + cached state 오판이
겹친 문제였다.
```

확인 및 복구 절차:

```bash
pgrep -af 'gripper_service_node|dsr_gripper_tcp'
kill <launch_pid> <node_pid>
ros2 daemon stop
ros2 daemon start
ros2 topic info /gripper_service/state -v
```

정상적으로 모두 꺼지면 다음처럼 떠야 한다.

```text
Unknown topic '/gripper_service/state'
```

그 후 그리퍼 노드를 하나만 다시 실행하고, Publisher count가 `1`인지 확인한다.

원본 패키지는 workspace의 별도 경로에 설치돼 있다.

```text
~/doosan_ws/src/dsr_gripper_tcp
~/doosan_ws/src/dsr_gripper_tcp_interfaces
```

원본 `dsr_gripper_tcp`는 검증 직후에는 ready였지만 인계 문서 작성 시점에는
`/gripper_service/state`가 더 이상 publish되지 않았다. Claude Code 시작 시
충돌 노드를 확인한 뒤 원본 패키지를 다시 실행해야 한다.

두 패키지는 workspace에 복사된 소스이며 현재 별도 `.git` 저장소는 아니다.
주 프로젝트 git에는 SafeGrasp 연동 스크립트와 검증 문서만 기록돼 있다.

유지:

```text
e0509_gripper_description bringup.launch.py
```

종료:

```text
/dsr01/gripper_service_node
safe_grasp_ros_adapter.py
```

원본 실행:

```bash
source ~/doosan_ws/install/setup.bash
ros2 launch dsr_gripper_tcp gripper_service_node.launch.py \
  controller_host:=110.120.1.66 \
  namespace:=dsr01 \
  stop_existing_drl:=true \
  initialize_on_start:=true \
  init_attempts:=10 \
  goal_current:=400
```

첫 `INITIALIZE status 3`만 보고 종료하지 않는다. TCP bridge 재연결 후
`Gripper service node ready`가 출력될 수 있으므로 ready 또는 전체 재시도
종료까지 기다린다.

상태 확인:

```bash
ros2 topic echo /gripper_service/state --once
ros2 action list -t | grep safe_grasp
```

## 4. Claude Code가 바로 할 일

### 우선 1 - 그리퍼 노드를 분리 실행하여 startup 진단

로봇 bringup 전체가 그리퍼 초기화에 묶이지 않도록 먼저 그리퍼 없이 실행한다.

```bash
source ~/doosan_ws/install/setup.bash
ros2 launch e0509_gripper_description bringup.launch.py \
  mode:=real host:=110.120.1.66 start_gripper_service:=false
```

그 후 별도 터미널에서 짧은 timeout으로 그리퍼 노드를 실행한다.

```bash
source ~/doosan_ws/install/setup.bash
ros2 launch dsr_gripper_tcp gripper_service_node.launch.py \
  controller_host:=110.120.1.66 namespace:=dsr01 \
  stop_existing_drl:=true init_attempts:=3 init_timeout_sec:=30.0
```

그리퍼 노드가 정상인지 먼저 확인한다.

```bash
ros2 topic info /gripper_service/state -v
ros2 topic echo /gripper_service/state --once
ros2 action list -t | grep safe_grasp
```

정상 기준은 Publisher count `1`, `ready: true`, `status_text: ok`,
`/gripper_service/safe_grasp` 존재다. `prime_gripper_serial_drl.py`는 검증된
해결책이 아니므로 진단 실험으로만 사용한다.

### 우선 2 - SafeGrasp 조건별 보정

빈 파지, 줄기 파지, 잎/비목표 접촉을 각각 최소 5회 수행한다. 처음에는
`max_current=400`, `current_delta_threshold=120`을 기준으로 분포를 확인한다.

```bash
python3 scripts/run_safe_grasp_trial.py \
  --condition stem \
  --target-position 700 \
  --max-current 400 \
  --current-delta-threshold 120 \
  --notes "manual stem fixture calibration" \
  --execute
```

잎 시험은 `--condition leaf_or_non_target`, 빈 파지는 `--condition empty`를 쓴다.

### 우선 3 - cuRobo SafeGrasp 통합 실기 검증

현재 미커밋 코드는 다음 구조로 변경됐다.

```text
/gripper_service/safe_grasp action
 -> feedback position/current/current_delta runtime JSONL 기록
 -> result grasp_detected/object_lost
 -> GRASP_CONTACT_DETECTED | GRASP_EMPTY | GRASP_UNVERIFIED
```

주의:

- `grasp_detected=true`는 무언가 잡힌 것이며, 줄기 파지 성공은 아니다.
- SafeGrasp 서버가 없으면 `SetPosition + GetState`로 fallback한다.
- 빌드와 Python compile은 통과했지만 실기 자동 반복은 아직 켜지 않는다.

### 우선 4 - NW 예비 실험과 수기 KPI 입력

SafeGrasp 통합 후 Place를 끄고 `root/nw`에서 5회 예비 실험한다.

```bash
python3 scripts/set_experiment_context.py \
  --cell root/nw \
  --scene-id nw_leaf_stem_occlusion_v1 \
  --occlusion leaf_and_stem \
  --stem-shape mixed
```

측정할 핵심:

- target 발견 여부 및 KP1 가시성
- 접근/계획 성공 여부
- SafeGrasp 접촉/빈 파지
- 실제 줄기 파지, 분리, 유지 여부
- 잎 또는 비목표 접촉
- 사람 개입 여부

현재 NW는 SW에서 검증한 수확 모션을 이식해 검증하는 단계다. 따라서 처음부터
성공률을 주장하지 않고, 각 시도 직후 `manual_labels_root_nw.csv`에 사람이
직접 보고 라벨을 입력한다.

## 5. KPI 자동/수동 구분

### 자동 기록

- plan success/fail/reject와 planning latency
- MoveSplineJoint/MoveLine 실행 결과
- pick sequence time과 hold/recovery 원인
- SafeGrasp position/current/current_delta
- 접촉 후보, 빈 파지, object-lost

### 사람이 현장에서 바로 입력해야 하는 항목

- 실제 줄기를 잡았는지
- 딸기가 줄기에서 분리됐는지
- retreat 후 유지됐는지
- 잎/다른 딸기/구조물에 접촉했는지
- 목표 slot에 정상 배치됐는지

사용자는 영상을 따로 찍어 후처리 라벨링하지 않고, 각 시도 직후 육안으로
확인한 값을 수동 라벨 CSV에 입력한다. `grasp_detected=true`만으로 최종 수확
성공을 선언하지 않는다.

## 6. Place 현재 결론

현재 place는 marker localization이 아니라 Slot0/1/3 티칭값에서 계산한 고정
격자 baseline이다.

```text
Slot0 Slot3 Slot6 Slot9 Slot12
Slot1 Slot4 Slot7 Slot10 Slot13
Slot2 Slot5 Slot8 Slot11 Slot14
```

- row0/1: BASE `-Z` 방식으로 Slot0/1/3/4 검증
- row2: J3 실측 한계와 수직 하강 문제가 있음
- Slot5: line deviation `100.8mm > 20mm`, 정상 안전 차단
- row2는 Cartesian constraint/waypoint IK 또는 collision geometry 보강 전까지 중단

## 7. 보존 및 금지

- `scripts/측정.py`는 사용자 원본이다. 수정, stage, commit 금지.
- 기존 SW 동작 baseline을 전면 재작성하지 않는다.
- SafeGrasp 통합 전후 결과를 별도 로그로 비교한다.
- AnyGrasp/GraspGen은 기존 KP1 rule을 즉시 대체하지 않고 offline baseline부터 평가한다.

## 8. Claude Code 인계 요약 - 바로 이어갈 것

### 현재 결론

- SW 단일 딸기 수확 모션은 baseline으로 확보했다.
- NW는 잎/줄기 가림 조건이라 SW 모션을 그대로 적용했을 때 어디서 깨지는지
  검증하는 단계다.
- SafeGrasp는 통신/feedback은 확인됐지만, 얇은 딸기 줄기에서는 최종 파지
  성공 판정기로 아직 검증되지 않았다.
- 현재 SafeGrasp는 `present_position`, `present_current`, `current_delta`,
  `grasp_detected`, `object_lost`를 자동 기록하는 접촉 후보 신호로 사용한다.
- 최종 성공 KPI는 사람이 시도 직후 직접 보고 `manual_labels_root_nw.csv`에
  입력한다.

### 다음 실험 순서

1. 그리퍼 노드가 단일 실행인지 확인한다.

```bash
pgrep -af 'gripper_service_node|dsr_gripper_tcp'
ros2 topic info /gripper_service/state -v
ros2 topic echo /gripper_service/state --once
ros2 action list -t | grep safe_grasp
```

정상 기준:

```text
Publisher count: 1
ready: true
status_text: ok
/gripper_service/safe_grasp 존재
```

2. 로봇 bringup은 그리퍼 자동 시작을 끄고 실행한다.

```bash
source ~/doosan_ws/install/setup.bash
ros2 launch e0509_gripper_description bringup.launch.py \
  mode:=real host:=110.120.1.66 start_gripper_service:=false
```

3. 별도 터미널에서 그리퍼 서비스를 실행한다.

```bash
source ~/doosan_ws/install/setup.bash
ros2 launch dsr_gripper_tcp gripper_service_node.launch.py \
  controller_host:=110.120.1.66 namespace:=dsr01 \
  stop_existing_drl:=true init_attempts:=3 init_timeout_sec:=30.0
```

4. NW 실험 context를 설정한다.

```bash
python3 scripts/set_experiment_context.py \
  --cell root/nw \
  --scene-id nw_leaf_stem_occlusion_v1 \
  --occlusion leaf_and_stem \
  --stem-shape mixed
```

5. NW에서 place는 처음부터 묶지 말고 pick/detach/retreat 중심으로 3~5회
   예비 검증한다.

6. 각 시도 직후 라벨 시트를 갱신하고 사람이 직접 입력한다.

```bash
python3 scripts/prepare_harvest_label_sheet.py --cell root/nw
```

입력 파일:

```text
reports/harvest_kpi/manual_labels_root_nw.csv
```

사람이 입력할 열:

```text
stem_grasp, detach, retention, non_target_contact,
human_intervention, place, notes
```

7. 자동 KPI를 확인한다.

```bash
python3 scripts/summarize_runtime_kpis.py --cell root/nw
python3 scripts/generate_harvest_kpi_report.py --cell root/nw
```

### SafeGrasp 운영값

현재 추천값:

```text
pre_open_position = 600
target_position = 700
max_current = 400
current_delta_threshold = 180
```

주의:

- `current_delta_threshold=140`은 empty 3회 중 2회 오검출로 사용 불가.
- `180`은 접촉 후보 로그용이다. 파지 성공 확정값이 아니다.
- `220`은 empty 억제는 좋지만 모루줄기 미검출 가능성이 크다.

### AnyGrasp

AnyGrasp는 지금 바로 실기 runtime에 넣지 않는다. NW 3~5회 검증 후
`줄기 방향 오차`, `잎 접촉`, `grasp miss`가 반복되면 저장된 RGB-D/point
cloud로 offline 평가한다. offline에서 rule 기반 KP1 target보다 좋은 후보를
안정적으로 만들 때만 runtime 후보 생성기로 연결한다.

## 9. 관련 파일

```text
scripts/curobo_planner_node.py
scripts/run_safe_grasp_trial.py
scripts/set_experiment_context.py
scripts/summarize_runtime_kpis.py
scripts/generate_harvest_kpi_report.py
docs/SAFE_GRASP_STANDALONE_TEST_20260615.md
docs/GRIPPER_BIDIRECTIONAL_DIAGNOSIS_20260615.md
docs/HANDOFF_20260614_PLACE_TRAY_GRID.md
docs/NW_OCCLUSION_KPI_AND_GRASP_DIRECTION_20260615.md
docs/HARVEST_EXPERIMENT_OPERATION_PLAN_20260615.md
```

현재 git에서 사용자 원본 `scripts/측정.py`만 untracked 상태로 남아 있어야 한다.
