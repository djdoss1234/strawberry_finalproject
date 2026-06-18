# HANDOFF - 2026-06-17 NW Motion, Gripper, KPI Status

기준 시각: 2026-06-17 저녁 실기 이후

이 문서는 SW 단일딸기 검증 이후 NW 잎/줄기 가림 셀로 넘어가며 발생한
모션, 그리퍼, place, KPI 관련 상태를 다음 작업자가 바로 이어받기 위한 기록이다.

## 1. 현재 전체 진행상황

### SW 단일딸기

- SW 단일딸기에서 비전 검출, 3D target 변환, pre-approach, 줄기 파지,
  BASE `-Z` 분리, retreat까지 성공 사례를 확보했다.
- 기준 runtime log:

```text
logs/runtime/2026-06-09/curobo_planner_node_20260609T160052-da5edd5a.jsonl
```

- 기준 Pick cycle:

```text
target 수신 -> scan pose 복귀: 약 36.4초
```

- 이후 후보 순서, IK seed, close wait, grasp check timeout을 줄여 병목을 줄였다.
  다만 동일 조건 반복 측정은 아직 완료하지 않았으므로 공식 평균 KPI는 미확정이다.

### NW 잎/줄기 가림 셀

- 현재는 `root/nw`를 대상으로, SW에서 검증한 rule 기반 수확 모션이 가림
  환경에서도 작동하는지 검증 중이다.
- workspace scan은 NW 영역을 세부 scan pose들로 나누어 관측한다.
- 목표는 다음을 확인하는 것이다.

```text
1. 4개 세부 scan pose에서 target이 안정적으로 잡히는가
2. target 좌표가 실제 줄기 파지점과 맞는가
3. cuRobo pre-approach가 위험 branch 없이 생성되는가
4. 열린 그리퍼 하강/close/detach/retreat가 잎을 밀지 않고 가능한가
5. KPI 수집이 자동/수동 라벨과 함께 남는가
```

현재 NW는 아직 안정화 전이다. 특히 2026-06-17 실기에서 J4 branch가 크게
튀는 문제와 gripper timeout이 반복됐다.

## 2. 현재 Pick 시퀀스

현재 수확 모션은 단순히 target에서 바로 닫는 방식이 아니다.

```text
비전 검출
 -> 3D target 변환
 -> cuRobo pre-approach 계획
 -> Doosan MoveSplineJoint로 pre-approach 접근
 -> MoveLine으로 줄기 방향 직선 진입
 -> gripper open position 600 유지
 -> KP1보다 위쪽으로 접근
 -> 열린 그리퍼 상태로 BASE -Z 약 30mm 하강
 -> KP1 근처 줄기 위치에서 close
 -> BASE -Z 약 40mm 아래로 당김 분리
 -> TOOL -Z 후퇴
 -> place 또는 scan pose 복귀
```

의도:

- 잎/과실 넓은 부분을 정면으로 밀지 않고 줄기 방향으로 진입한다.
- 줄기보다 약간 위에서 바로 닫지 않고, 열린 상태로 내려오며 KP1 부근의 가는
  줄기 위치에서 닫는다.
- 파지 후 BASE `-Z`로 당겨 줄기에서 과실을 분리한다.

## 3. 2026-06-17 주요 실패와 원인

### 3.1 NW J4 branch가 크게 튀며 벽/그리퍼 파손 위험 발생

관찰:

- 첫 scan을 건너뛰고 다음 scan pose에서 target을 찾은 뒤, pick 접근 중 J4가
  크게 돌아 위험한 움직임이 발생했다.
- 사용자는 실제로 그리퍼 파츠가 벽에 닿아 파손됐다고 보고했다.

대표 로그:

```text
logs/runtime/2026-06-17/curobo_planner_node_20260617T192623-c8ed4d45.jsonl
```

문제 패턴:

```text
Cartesian plan joint equivalent rewrite: J4 305.5~340.4 -> -54.5~-19.6
```

원인:

- NW scan/pick에서는 J4가 `305~340deg` 근처인 물리 branch가 실제로 가능하다.
- 기존 normalize 로직이 현재 joint와 가까운 equivalent를 고르면서 `305deg`를
  `-55deg` branch로 바꿨다.
- 이 rewrite는 수학적으로 같은 각도처럼 보이지만, 실제 로봇은 시작점으로 맞추는
  과정에서 J4를 크게 돌 수 있다.

조치:

- `OPERATIONAL_JOINT_LIMITS_DEG`의 J4 범위를 `±280deg -> ±360deg`로 넓혔다.
- cuRobo `plan()`과 `plan_js()`에서 `robot_start_joints_rad=self.current_joints`
  기준 normalize 호출을 제거했다.
- 의도는 cuRobo가 낸 실제 branch를 함부로 반대 부호 equivalent로 재작성하지
  않는 것이다.

검증 필요:

- 빌드 후 새 run에서 위 rewrite 로그가 사라지는지 확인한다.
- 특히 J4가 `305~340deg` branch를 유지하는지 확인한다.

### 3.2 속도 30% 적용이 안 된 것처럼 보임

관찰:

- 사용자가 `colcon build` 후에도 "아직 너무 빠르다"고 보고했다.
- `workspace_scan.launch.py` 실행 시 일부 파라미터 이름을 잘못 넣은 적이 있다.

주의:

```text
잘못된 이름:
scan_movej_vel
scan_movej_acc

실제 launch에서 써야 하는 이름:
scan_movej_vel_deg_s
scan_movej_acc_deg_s2
overview_return_vel_deg_s
overview_return_acc_deg_s2
```

조치:

- `curobo_planner_node.py` 내부 MoveLine/MoveSpline 속도를 약 30% 수준으로 낮췄다.
- `execute_spline()`의 `req.vel/acc`도 낮췄다.
- place 전 operation speed 강제값도 `100 -> 30`으로 낮췄다.

확인할 로그:

```text
FINAL_APPROACH_STRAIGHT ... vel=15.0mm/s
RETREAT ... vel=24.0mm/s
Spline ... velocity_deg_s=[36.0,...] 또는 req.vel=36
```

### 3.3 overview pose gate가 등가 관절 때문에 실패

증상:

```text
ros2 service call /strawberry/scan/start std_srvs/srv/Trigger "{}"
response: success=False,
message='current joints do not match verified overview pose within 1.0 deg'
```

원인:

- 로봇은 물리적으로 overview pose에 있었지만, joint state가 예를 들어
  `J1=-272deg`처럼 `87.98deg`의 `-360deg` equivalent로 들어왔다.
- 기존 gate는 단순 차이만 보므로 같은 자세를 다른 자세로 오판했다.

조치:

- `strawberry_motion`의 scan executor 쪽에 wrap-aware overview 판정을 적용했다.
- 이 파일은 이 저장소가 아니라 `/home/user/doosan_ws/src/strawberry_finalproject`
  쪽이다. 해당 패키지는 별도로 build/source해야 한다.

검증 필요:

```bash
cd ~/doosan_ws
colcon build --packages-select strawberry_motion
source install/setup.bash
```

### 3.4 joint_jog_control overview 이동도 위험할 수 있음

증상:

- cuRobo node가 다녀온 뒤 `joint_jog_control.py`로 overview를 보내면 관절 한도
  또는 큰 swing 때문에 로봇이 멈출 수 있었다.

조치:

- `joint_jog_control.py`에 J1/J4/J6 360도 equivalent 선택을 추가했다.
- 현재 joint에서 가장 가까운 equivalent를 고른 뒤, 과도한 joint swing이면
  MoveJoint를 차단한다.

확인 메시지:

```text
MoveJoint blocked: excessive joint swing ...
```

### 3.5 gripper service가 가끔 timeout/status 3로 실패

증상:

```text
INITIALIZE attempt ... failed (gripper): Controller returned error status 3
INITIALIZE transport failed (TCP connection closed while receiving data.)
GRIPPER: set_position(600) timed out
```

확인된 원인/단서:

- 예전에 실행한 `gripper_service_node`가 고아 프로세스로 남아
  `/gripper_service/state`를 계속 publish한 적이 있었다.
- 이 경우 `ready:true`처럼 보이지만 실제 새 노드와 cached state가 섞여
  이상 상태가 된다.
- `ros2 daemon stop/start`만으로는 살아있는 프로세스가 사라지지 않는다.
- 반대로 gripper service가 정상일 때 매번 죽이고 다시 띄우면 DRL/serial 초기화
  문제가 재발할 수 있다.

복구 스크립트:

```bash
cd ~/doosan_ws/src/e0509_gripper_description
./scripts/clean_robot_runtime.sh
```

주의:

- 이 스크립트는 gripper service도 죽인다.
- gripper가 이미 정상(`status_text: ok`)이면 불필요하게 실행하지 않는다.

정상 확인:

```bash
ros2 topic echo /gripper_service/state --once
ros2 action list -t | grep safe_grasp
```

정상 예:

```text
ready: true
present_position: 600 or 700
present_current: single digit~수십
status_text: ok
/gripper_service/safe_grasp [dsr_gripper_tcp_interfaces/action/SafeGrasp]
```

## 4. Gripper / SafeGrasp / 파지 성공 판정 상태

### 현재 사용하는 값

- 파지 전 open: `position=600`
- close target: `position=700`
- place release도 완전 개방이 아니라 `position=600`

현재 코드 기준:

```text
GRIPPER_APPROACH_POS = 600
GRIPPER_PLACE_RELEASE_POS = 600
GRASP_EMPTY_POSITION_THRESHOLD = 700
```

의미:

- `present_position >= 700`이면 목표까지 거의 완전히 닫힌 것이므로 빈 파지
  후보로 본다.
- `present_position < 700`이라고 바로 성공은 아니다. 줄기, 잎, 과실 표면, 파츠
  마찰 모두 가능하다.
- SafeGrasp의 `current_delta`는 접촉 후보 신호로 남긴다.

### 2026-06-16 SafeGrasp 보정 결론

확인:

- `dsr_gripper_tcp`의 `/gripper_service/state`와 `/gripper_service/safe_grasp`
  경로에서 position/current 읽기는 가능했다.
- 기존 `/dsr01/gripper/read_state` 직접 경로는 `-1/-1`로 실패했다.

문제:

- 모형 딸기 줄기가 얇고 유연해서 전류 변화가 일정하지 않다.
- `current_delta_threshold`를 낮추면 empty도 오검출되고, 높이면 줄기를 놓친다.
- moru stem 조건에서도 5회 중 1회만 안정적으로 검출되는 등 전류 단독 판정은
  아직 불안정하다.

현재 운영 판단:

```text
SafeGrasp 자동값 = 접촉/빈 파지 후보 로그
최종 수확 성공 = 사람이 즉시 확인해 수기 라벨 입력
```

다음 NW 실험에서는 SafeGrasp 값을 runtime JSONL과 KPI 보고서에 남기고,
실험자가 직접 줄기 파지/분리/유지/place 성공 여부를 수기로 적는다.

## 5. Place 검증 상태

계란판은 현재 marker localization 완전 자동화가 아니라, Slot0/1/3 티칭 좌표와
격자 오프셋을 이용한 고정 tray grid 방식이다.

검증 상태:

| Slot | 상태 | 비고 |
| --- | --- | --- |
| Slot0 | 성공 | 티칭 좌표 기반 |
| Slot1 | 성공 | 티칭 좌표 기반 |
| Slot3 | 성공 | 티칭 좌표 기반 |
| Slot4 | 성공 | Slot0/1/3 격자 기반 생성 |
| Slot2 | 보정 후 도달 관찰 | row2 pitch tilt + correction 적용. 반복 검증 필요 |
| Slot5 | 실패/안전 차단 | row2 descent line deviation `100.8mm > 20mm` |

Slot5 대표 로그:

```text
ROW2_DESCENT_LINE_CHECK max_deviation=100.8mm limit=20.0mm
TAUGHT_TRAY_SLOT5_PLACE_BLOCKED
```

결론:

- row0/row1은 기존 BASE `-Z` 방식으로 나머지 slot 검증을 계속할 수 있다.
- row2는 cuRobo가 관절공간으로 꺾인 하강 경로를 만들기 때문에, Cartesian
  constraint/waypoint IK 또는 collision geometry 보강 전까지 자동 release를
  막는 것이 맞다.

## 6. KPI 상태

자동으로 남는 것:

- runtime JSONL path
- target 수신/계획/실행/result event
- cuRobo plan OK/FAIL/reject
- plan latency
- MoveSpline/MoveLine 실행 결과
- SafeGrasp feedback가 성공적으로 들어오면 `present_position`,
  `present_current`, `current_delta`, `detected`
- cycle time 후보

사람이 직접 적어야 하는 것:

- 실제 줄기 파지 여부
- 잎/과실/파츠 접촉 여부
- 딸기 분리 성공 여부
- retreat 후 유지 여부
- place 성공 여부
- 사람 개입 여부와 이유

수기 입력 파일:

```text
reports/harvest_kpi/manual_labels_root_nw.csv
```

도구:

```bash
python3 scripts/prepare_harvest_label_sheet.py --cell root/nw
python3 scripts/check_harvest_logging.py --cell root/nw
python3 scripts/summarize_runtime_kpis.py --cell root/nw
python3 scripts/generate_harvest_kpi_report.py --cell root/nw
```

현재 KPI는 아직 공식 수치가 아니다. NW 모션이 실패/불안정하므로,
다음 안정화 run부터 자동 로그와 수기 라벨을 함께 모아 반복 측정한다.

## 7. 현재 미커밋 코드 변경 요약

### `scripts/curobo_planner_node.py`

- 실기 안전 검증을 위해 MoveLine/MoveSpline 속도 약 30% 수준으로 낮춤.
- J4 operational limit을 `±360deg`로 확대.
- cuRobo trajectory equivalent normalize에서 현재 joint 기준 강제 rewrite 제거.
- place 전 operation speed 강제값을 `100 -> 30`으로 낮춤.

### `scripts/joint_jog_control.py`

- named pose 이동 전 현재 joint 기준 J1/J4/J6 equivalent 선택.
- 과도한 joint swing이면 MoveJoint 실행 차단.
- gripper service는 `/gripper_service/set_position` 경로 사용.

### `scripts/clean_robot_runtime.sh`

- 남아 있는 bringup/gripper/curobo/workspace_scan/fusion 관련 프로세스 정리.
- ROS daemon 재시작.
- 중복 gripper_service_node, stale ROS graph 정리용.

### `scripts/prime_gripper_serial_drl.py`

- gripper serial/DRL 초기화 block 문제 진단용 실험 스크립트.
- 검증된 해결책은 아니며, 필요할 때만 사용한다.

### 절대 건드리지 말 것

```text
scripts/측정.py
```

현재 git에서 untracked로 보일 수 있지만, 사용자가 만든 측정용 파일이므로 add,
수정, 삭제하지 않는다.

## 8. 다음 작업 순서

1. 코드 build/source

```bash
cd ~/doosan_ws
colcon build --packages-select e0509_gripper_description strawberry_motion
source install/setup.bash
```

2. gripper service가 살아 있으면 건드리지 말고 상태만 확인

```bash
ros2 topic echo /gripper_service/state --once
ros2 action list -t | grep safe_grasp
```

3. 필요 시 bringup은 gripper service 제외로 실행

```bash
ros2 launch e0509_gripper_description bringup.launch.py \
  mode:=real host:=110.120.1.66 start_gripper_service:=false
```

4. cuRobo planner 실행

```bash
ros2 run e0509_gripper_description curobo_planner_node.py --ros-args \
  -p measured_tcp_plan_only:=false
```

5. NW workspace scan은 저속 파라미터 이름을 정확히 사용

```bash
ros2 launch strawberry_motion workspace_scan.launch.py \
  enable_robot_execution:=true \
  target_cell:=root/nw \
  enable_fusion_detection:=true \
  max_total_picks:=1 \
  scan_movej_vel_deg_s:=5.0 \
  scan_movej_acc_deg_s2:=10.0 \
  overview_return_vel_deg_s:=5.0 \
  overview_return_acc_deg_s2:=10.0
```

6. Trigger

```bash
ros2 service call /strawberry/scan/start std_srvs/srv/Trigger "{}"
```

7. 첫 테스트에서 반드시 확인할 로그

```text
J4 305~340 -> -55 rewrite가 더 이상 없어야 함
FINAL_APPROACH_STRAIGHT vel=15.0mm/s
OPEN_STEM_DESCENT vel=9.0mm/s
RETREAT vel=24.0mm/s
gripper set_position(600) timeout이 없어야 함
```

## 9. 발표/노션에 넣을 수 있는 현재 결론

- SW 단일딸기에서는 수확 모션 성공 사례를 확보했다.
- 현재는 NW 잎/줄기 가림 셀에서 4개 세부 scan pose 기반 검증을 진행 중이다.
- 파지 모션은 `바로 close`가 아니라 `open 상태 하강 -> KP1 근처 close ->
  BASE -Z detach` 구조로 개선했다.
- 계란판 place는 Slot0/1/3/4 성공, Slot2는 보정 후 도달 관찰, Slot5 row2는
  직선 하강 이탈로 안전 차단했다.
- 전류 기반 SafeGrasp는 통신/로그는 가능하지만 얇은 줄기에서는 단독 성공
  판정기로 아직 불안정하다.
- KPI는 자동 로그와 수기 라벨 구조를 만들었고, NW 모션 안정화 후 반복 측정을
  시작한다.
- 2026-06-17에는 NW J4 branch rewrite와 gripper service 불안정이 핵심
  blocker였고, J4 equivalent rewrite 방지 및 저속화 코드를 반영했다.
