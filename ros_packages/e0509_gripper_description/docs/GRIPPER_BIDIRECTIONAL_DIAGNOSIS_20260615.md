# RH-P12-RN-A 양방향 통신 진단 - 2026-06-15

## 결론

기존 `/dsr01/gripper/read_state` 직접 ROS 경로에서는 FC03 읽기가 실패했지만,
원본 `dsr_gripper_tcp` 패키지의 DRL TCP bridge 경로에서는 RH-P12-RN-A의
position/current 양방향 판독과 SafeGrasp 자동 판정이 정상 동작한다.

따라서 수확 파지 자동 판정의 기준 경로는
`/gripper_service/safe_grasp`와 `/gripper_service/state`로 정한다.

## 확인한 항목

- 통신 설정: port 1, slave ID 1, 57600 baud, 8-N-1
- 기존 ROS flange-serial 방식의 position/torque 쓰기 동작
- `dsr_gripper_tcp` DRL TCP 서버 연결 성공
- DRL 내부 첫 INITIALIZE: `status 3` 실패 후 bridge 재연결
- 재연결 후 `Gripper service node ready at 20.0 Hz`
- ROS `/dsr01/gripper/read_state`: `position=-1`, `current_raw=-1`
- 요청 직후 0.1초 단위 반복 raw read: 항상 0바이트
- FC03 읽기 주소:
  - Torque Enable 256
  - Goal Current 275
  - Moving Status 285
  - Present Current/Position 287~291

직접 ROS flange-serial 주소 읽기는 실패했지만, 이는 원본 TCP bridge 경로의
동작 가능 여부를 뜻하지 않는다.

## 의미

| 기능 | 현재 판단 |
| --- | --- |
| position/torque 쓰기 | 가능 |
| Goal Current 쓰기 | 프로토콜상 가능, 실기 검증 필요 |
| Present Current/Position 읽기 | 원본 TCP bridge에서 가능 |
| 전류 기반 실시간 접촉 판정 | SafeGrasp에서 가능, 임계값 보정 필요 |
| 전류 기반 object-lost 판정 | SafeGrasp에서 가능, 실기 검증 필요 |
| 사람/영상 기반 수확 성공 판정 | 계속 사용 가능 |

2026-06-15 원본 패키지 빈 파지 시험에서는 `present_position=700`,
`present_current=8`, `grasp_detected=false`, `target reached without grasp`가
기록되어 빈 파지 자동 판정을 확인했다.

## SafeGrasp 전류제어 원리와 판정 기준

그리퍼 전류는 모터가 목표 위치까지 닫히는 동안 느끼는 부하를 간접적으로
나타낸다.

```text
아무것도 없음
 -> 목표 위치까지 거의 닫힘
 -> 저항이 작음
 -> 전류 증가가 작음

물체가 끼어 있음
 -> 목표 위치까지 완전히 닫히지 못함
 -> 모터 부하 증가
 -> 전류 증가
```

따라서 SafeGrasp는 "딸기 줄기를 잡았다"를 직접 판정하는 것이 아니라,
**그리퍼가 닫히는 중 예상보다 큰 저항을 만났는지**를 판정한다.

### 주요 값 의미

| 값 | 의미 | 해석 |
| --- | --- | --- |
| `present_position` | 현재 그리퍼 위치 pulse | 숫자가 클수록 더 닫힌 상태. `600`은 파지 전 열린 상태, `700`은 닫기 목표 근처 |
| `present_current` | 현재 모터 전류 raw 값 | 모터 부하/힘의 간접값 |
| `current_delta` | 시작 전류 대비 증가량 | 닫는 중 저항이 얼마나 증가했는지 |
| `max_current` | 최대 허용 전류 | 너무 높으면 손상 위험, 너무 낮으면 줄기 파지 실패 가능 |
| `current_delta_threshold` | 잡힘 감지 전류 증가 기준 | 낮으면 빈 파지 오탐, 높으면 실제 줄기 파지 미검출 |

추가 상태값:

| 값 | 의미 | 실험에서 보는 법 |
| --- | --- | --- |
| `goal_position` | 명령한 목표 위치 | 현재 close 목표는 보통 `700` |
| `current_limit` | 현재 적용된 전류 제한 | SafeGrasp goal의 `max_current`와 대응 |
| `grasp_detected` | SafeGrasp 접촉 감지 | 접촉 후보. 줄기 파지 성공 확정값 아님 |
| `object_lost` | 잡은 물체 이탈 감지 | 실기 반복 검증 필요 |
| `moving` | 그리퍼 이동 중 여부 | SafeGrasp feedback 해석 보조 |
| `in_position` | 목표 위치 도달 여부 | 빈 파지는 보통 목표 근처까지 닫힘 |
| `present_velocity` | 현재 속도 | 닫힘 중 상태 확인용 |
| `present_temperature` | 그리퍼 온도 | 과열 감시 |
| `status_text` | 상태 문자열 | 실험 전 `ok` 확인. `timed out`이면 신뢰하지 않음 |

예:

```text
start current = 8
final current = 155
current_delta = 147
threshold = 120
=> 147 > 120 이므로 grasp_detected=true
```

### 조정해야 하는 명령값과 방향

| 값 | 현재 기준 | 역할 | 조정 방향 |
| --- | --- | --- | --- |
| `pre_open_position` | `600` | 파지 전 줄기가 들어갈 공간 확보 | 너무 닫혀 있으면 줄기가 못 들어가고, 너무 열리면 닫힘 거리/시간과 마찰 오검출이 늘 수 있음 |
| `target_position` | `700` | close 목표 위치 | 너무 작으면 줄기를 못 잡고, 너무 크면 구조물/과실을 더 누를 수 있음 |
| `max_current` | `400` | 허용 최대 힘/전류 | 높이면 강하게 잡지만 손상/충돌 위험 증가, 낮추면 파지 실패 가능 |
| `current_delta_threshold` | 후보값 `180` | 접촉 감지 민감도 | 낮추면 줄기 감지는 쉬워지지만 빈 파지 오탐 증가, 높이면 오탐은 줄지만 얇은 줄기 미검출 |

2026-06-16 현재 추천 운영값:

```text
pre_open_position = 600
target_position = 700
max_current = 400
current_delta_threshold = 180  # 접촉 후보 로그용, 최종 성공 판정 아님
```

중요:

```text
SafeGrasp 자동값 = 접촉 후보/빈 파지 후보를 기록하는 센서값
수확 성공 KPI = 사람이 즉시 확인한 stem_grasp/detach/retention/place 라벨
```

향후 자동 판정을 개선한다면 전류 하나만 쓰지 않고 다음처럼 위치 조건을 같이
본다.

```text
접촉 후보:
  current_delta >= threshold
  AND present_position <= position_limit
```

다만 2026-06-16 empty threshold 140 시험에서 `position=690`, `delta=230`
오검출이 확인됐으므로, 현재 데이터만으로는 위치+전류 조합도 최종 판정기로
확정하지 않는다.

### 연장 파츠 조건에서의 주의점

15cm급 연장 파츠를 장착해도 전류제어 자체는 동작한다. 모터가 끝단 접촉으로
인한 저항을 느끼면 전류가 증가하기 때문이다. 다만 얇은 딸기 줄기에서는 다음
이유로 자동 판정이 어려워진다.

- 줄기가 얇고 말랑해서 모터 부하 변화가 작을 수 있음
- 연장 파츠 끝단에서 줄기가 미끄러지거나 파츠가 미세하게 휠 수 있음
- 잎이나 과실 표면에 스쳐도 줄기보다 큰 전류 변화가 발생할 수 있음
- 따라서 `grasp_detected=true`는 "무언가 닿았다"는 뜻이지,
  "정확히 줄기를 잡았다"는 뜻이 아님

과자봉지나 캔처럼 면적이 넓고 단단한 물체는 전류 차이가 크게 나므로 쉬운
조건이다. 딸기 줄기는 훨씬 작은 접촉 대상이라 별도 보정이 필요하다.

### 현재 empty run 해석

2026-06-16 빈 파지 시험:

```text
start: position=600 current=8 delta=0 detected=False
final: position=697 current=155 delta=147 detected=True
```

해석:

- `position=697`은 목표 `700`에 거의 도달한 것이므로 빈 파지 자체는 정상
- 그러나 `current_delta=147`이 기존 threshold `120`보다 커서
  SafeGrasp가 빈 파지를 잡힘으로 오판
- 따라서 `current_delta_threshold=120`은 현재 장비/연장 파츠 조건에서 너무 낮다

### 보정 방식

조건별로 최소 5회씩 수집한다.

```text
1. empty: 아무것도 없는 빈 파지
2. stem: 줄기만 정확히 파지
3. leaf_or_non_target: 잎 또는 과실 표면 접촉
```

각 run에서 기록할 값:

```text
final_position
final_current
current_delta
grasp_detected
사람 라벨: empty / stem / leaf_or_non_target
```

권장 초기 threshold:

```text
current_delta_threshold = empty 평균 + 2 * empty 표준편차
```

예:

```text
empty delta = 130, 145, 147, 150, 155
mean = 145
std = 10
threshold = 145 + 20 = 165
```

그 다음 stem delta가 threshold보다 충분히 큰지 확인한다. empty와 stem 분포가
겹치면 전류 단독 판정은 불안정하므로 영상/사람 라벨과 함께 사용한다.

최종 자동 판정은 전류 하나만 보지 않고 위치까지 함께 본다.

```text
접촉 후보:
  current_delta > threshold
  AND final_position < empty_position_mean - margin

빈 파지 후보:
  final_position ~= empty_position_mean
  AND current_delta <= threshold
```

단, 줄기가 매우 얇으면 position 차이가 작을 수 있으므로 실제 임계값은
empty/stem/leaf 실측 분포를 보고 결정한다.

### 2026-06-16 SafeGrasp 보정 결과

`dsr_gripper_tcp`의 `/gripper_service/safe_grasp`와
`scripts/run_safe_grasp_trial.py`로 빈 파지와 모루줄기 파지를 반복 측정했다.
모든 시험은 `pre_open_position=600`, `target_position=700`,
`max_current=400` 조건이다.

| 조건 | threshold | 결과 요약 | 판단 |
| --- | --- | --- | --- |
| empty | 120 | `position=697`, `delta=147`, `detected=True` | 빈 파지 오검출. 너무 낮음 |
| empty | 220 | 대부분 `position=700`, `delta=0~35`, `detected=False` | 빈 파지 억제는 안정 |
| stem_moru | 220 | `peak delta=199`, 최종 `position=699`, `detected=False` | 모루줄기 미검출 |
| stem_moru | 180 | 5회 중 1회만 검출. 성공 시 `position=671`, `delta=218` | 단독 기준으로 불안정 |
| stem_moru | 140 | 3회 중 1회 검출. 실패는 `position=697~699` | 줄기 감지 부족 |
| empty | 140 | 3회 중 2회 오검출. `position=697/delta=140`, `position=690/delta=230` | 사용 불가 |

결론:

- `current_delta_threshold=140`은 빈 파지 오검출이 커서 사용할 수 없다.
- `180`은 빈 파지에는 상대적으로 안전하지만 모루줄기 검출률이 낮다.
- `220`은 빈 파지 억제에는 좋지만 실제 줄기를 놓친다.
- 따라서 현재 장비/연장 파츠/줄기 조건에서는 SafeGrasp 단독으로
  "줄기 파지 성공"을 확정하지 않는다.

운영 기준:

```text
SafeGrasp detected=True
  => 접촉 후보 자동 기록
  => 실제 줄기 파지 성공은 사람이 현장에서 바로 육안 확인하여 라벨 입력

SafeGrasp detected=False
  => 빈 파지 후보 또는 미약 접촉
  => 사람이 실제 파지 여부를 함께 확인
```

현재 수확 KPI에서는 SafeGrasp를 "접촉/빈 파지 자동 보조 신호"로 사용하고,
최종 수확 성공률은 `stem_grasp`, `detach`, `retention`, `place` 수기 라벨과
함께 계산한다.

## 재기동 후 startup blocker

컨트롤러 완전 재기동 후 `dsr_gripper_tcp` DRL 서버의 TCP 연결까지는
성공하지만, 첫 `INITIALIZE`에서 DRL 내부 `flange_serial_open()`이 반환하지
않는 사례가 확인됐다.

- Python socket timeout 증가는 대기 시간만 늘리며 근본 해결이 아니다.
- DRL `try/except`는 함수가 예외를 던질 때만 유효하고 block에는 효과가 없다.
- `dsr_gripper_tcp`는 시작 시 autonomous 전환 서비스를 이미 호출하므로,
  AUTO 모드는 확인해야 하지만 단독 원인으로 확정할 수 없다.
- 이전 DRL에서 새 DRL로 전환할 때 성공했던 패턴은 관찰됐지만, fresh boot
  재현 절차는 아직 확정되지 않았다.

### 2026-06-16 재현 및 해결

다음 문제가 실제 원인으로 확인됐다.

1. 2026-06-15 17:24에 실행한 `dsr_gripper_tcp gripper_service_node`가
   터미널 종료 후에도 고아 프로세스로 남아 있었다.
2. 이 고아 노드가 `/gripper_service/state`를 계속 publish했다.
3. 새 그리퍼 노드를 추가 실행하면서 `/gripper_service/state` publisher가
   2개가 됐다.
4. 고아 노드는 과거 정상 상태를 캐시한 채 `ready: true`를 publish했고,
   동시에 polling 실패로 `status_text: timed out`을 publish했다.
5. 이 때문에 `ready: true`를 보고 정상으로 오해할 수 있었다.

고아 프로세스:

```text
764910 ros2 launch dsr_gripper_tcp gripper_service_node.launch.py ...
764923 .../install/dsr_gripper_tcp/lib/dsr_gripper_tcp/gripper_service_node
```

해결 절차:

```bash
pgrep -af 'gripper_service_node|dsr_gripper_tcp'
kill <launch_pid> <node_pid>
ros2 daemon stop
ros2 daemon start
ros2 topic info /gripper_service/state -v
```

정상적으로 모두 꺼지면 `/gripper_service/state`는 `Unknown topic`이 된다.
그 후 bringup은 그리퍼 없이 띄우고, 그리퍼 노드는 하나만 별도 실행한다.

```bash
ros2 launch e0509_gripper_description bringup.launch.py \
  mode:=real host:=110.120.1.66 start_gripper_service:=false

ros2 launch dsr_gripper_tcp gripper_service_node.launch.py \
  controller_host:=110.120.1.66 namespace:=dsr01 \
  stop_existing_drl:=true init_attempts:=3 init_timeout_sec:=30.0
```

2026-06-16 재실행 결과:

```text
Connected to gripper TCP bridge at 110.120.1.66:20002
Gripper service node ready at 20.0 Hz
```

상태 확인 3회 모두 정상:

```text
ready: true
status_text: ok
present_position: 600
present_current: 8
```

SafeGrasp action도 정상 발견됐다.

```text
/gripper_service/safe_grasp [dsr_gripper_tcp_interfaces/action/SafeGrasp]
```

따라서 이번 startup 문제의 직접 원인은 하드웨어 고장이 아니라
**중복/고아 gripper_service_node와 cached state 오판**이었다.
다만 polling 중 일회성 `READ_STATE status 3` 경고가 나타날 수 있으므로,
실험 전에는 반드시 `status_text: ok`를 여러 번 확인한다.

진단 중에는 로봇 bringup과 그리퍼 startup을 분리한다.

```bash
ros2 launch e0509_gripper_description bringup.launch.py \
  mode:=real host:=110.120.1.66 start_gripper_service:=false

ros2 launch dsr_gripper_tcp gripper_service_node.launch.py \
  controller_host:=110.120.1.66 namespace:=dsr01 \
  stop_existing_drl:=true init_attempts:=3 init_timeout_sec:=30.0
```

### 중복 노드 주의

`ros2 topic info /gripper_service/state -v`에서 Publisher count가 `2`이면
동일 이름의 `gripper_service`가 두 개 실행 중이다. 이 경우 한 노드는 초기화
중이고 다른 노드는 과거 성공 상태를 캐시하여 다음처럼 모순된 값을 발행할 수
있다.

```text
ready: true
status_text: timed out
```

이는 정상 준비 상태가 아니다. 모든 `gripper_service_node` 프로세스를 종료한
뒤 하나만 다시 실행하고, Publisher count가 `1`인지 확인한다.

실험 전 정상 기준:

```text
ros2 topic info /gripper_service/state -v
  Publisher count: 1

ros2 topic echo /gripper_service/state --once
  ready: true
  status_text: ok

ros2 action list -t | grep safe_grasp
  /gripper_service/safe_grasp [dsr_gripper_tcp_interfaces/action/SafeGrasp]
```

## 다음 선택지

### A. 공식 ROBOTIS USB/DYNAMIXEL 통신 사용

그리퍼를 USB DYNAMIXEL 인터페이스에 연결하고 공식
`RH-P12-RN-A`/`dynamixel_hardware_interface`의 state interface에서
Present Current/Position을 읽는다. 양방향 전류 제어가 가장 명확한 경로지만,
현재 툴 플랜지 배선 구성을 변경해야 한다.

### B. 제공된 Doosan RH-P12-RN-DR Skill 검증

제조사 Skill의 `RH_GET_STATUS`, `RH_GET_CONFIG` 반환값을 DRL/ROS로 전달할 수
있는지 확인한다. 공식 매뉴얼상 상태 반환은 Hardware Error, Moving Status,
Moving이며 Present Current/Position 반환은 명시돼 있지 않다.

### C. 기존 수확 실험 계속 진행

기존 쓰기 기반 그리퍼 동작을 유지하고, 파지/분리/유지는 영상 및 사람 라벨로
판정한다. 자동 KPI는 planning, execution, cycle time, result code를 계속
수집한다.

## 보존 도구

```bash
python3 scripts/diagnose_gripper_read.py \
  --execute-read --start-register 287 --count 5
```

이 도구는 그리퍼를 움직이지 않고 FC03 raw read 응답만 확인한다.
