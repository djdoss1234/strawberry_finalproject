# RUN-20260527-013: 단일 Cell Scan 중단 및 Recovery 절차

## 상태

```text
DOCUMENTED_PENDING_PHYSICAL_VERIFICATION
```

이 문서는 첫 단일 cell scan(`root/ne` 또는 `root/nw`) 실기 실행 전에 운용자가
확인해야 하는 abort/recovery 절차를 기록한다.
물리 확인 (게이트 5) 이후 `use_for_automated_motion: true`로 전환한다.

---

## 1. 실행 전 체크리스트

| 항목 | 확인 기준 |
| --- | --- |
| 로봇 전원 및 DART 연결 | DART 화면에 정상 상태 표시 |
| E-stop 위치 확인 | 운용자 손 닿는 곳에 E-stop 준비 |
| Clear space | 화이트보드 앞 80 cm 반경 내 장애물 없음 |
| overview pose 확인 | `joint_deg=[97.84, -94.40, 65.95, -10.93, 95.49, -94.79]` 수동 이동 완료 |
| DART 수동 모드 대기 | 자동 실행 중 언제든 수동 전환 가능 |
| 담당자 2인 | 운용자 1인 (DART/E-stop), 보조 1인 (terminal 모니터링) |

---

## 2. 정상 실행 순서

```bash
# 터미널 1: ROS2 launch (visualization + executor, execute_motion:=false 기본)
ros2 launch strawberry_motion workspace_scan.launch.py

# 터미널 2: executor opt-in (단일 셀, root/ne 먼저)
ros2 launch strawberry_motion workspace_scan.launch.py \
  execute_motion:=true target_cell:=root/ne

# 터미널 3: 상태 모니터링
ros2 topic echo /strawberry/scan/status

# 터미널 4: scan 시작 명령 (운용자가 직접)
ros2 service call /strawberry/scan/start std_srvs/srv/Trigger {}
```

executor 거부 조건 (자동):
- overview 관절이 `±1.0 deg` 기준에서 벗어난 경우
- `execute_motion:=false` 또는 YAML 플래그 미설정
- `target_cell`이 `root/nw` 또는 `root/ne`가 아닌 경우

---

## 3. 비상 중단 절차

### 3-A. Software abort (우선)

```bash
# 별도 터미널에서 즉시 실행
ros2 service call /dsr01/motion/stop dsr_msgs2/srv/Stop "{stop_mode: 1}"
```

`stop_mode: 1` = 즉시 정지 (브레이크 포함).

### 3-B. E-stop (hardware)

Software abort가 응답하지 않으면 즉시 E-stop 버튼 압박.
E-stop 이후:
1. DART에서 alarm 확인 및 reset
2. joint 상태 확인 후 수동으로 safe 위치로 이동
3. incident 상세를 `docs/issues/` 에 기록

### 3-C. DART 수동 전환

언제든지 DART 조이스틱의 모드 전환 버튼으로 수동 모드로 전환 가능.
수동 전환 후 자동 motion 명령은 무시됨.

---

## 4. Executor 내부 abort 조건 (자동)

`scan_executor_node`는 다음 경우 즉시 sequence를 중단하고 overview로 복귀:

| 상황 | 조치 |
| --- | --- |
| cuRobo plan fail (IK_FAIL / JOINT_LIMIT_REJECT) | 시퀀스 중단, `MoveJoint` overview 복귀 |
| spline 실행 서비스 timeout (60s) | 시퀀스 중단, overview 복귀 시도 |
| overview 복귀 후 joint 미확인 (10s timeout) | `ABORT` 로그 후 node 대기 |

overview 복귀 속도: `vel=20.0, acc=20.0 deg/s` (scan 이동보다 느리게).

---

## 5. Recovery 절차 (executor abort 이후)

1. `/strawberry/scan/status` 로그에서 abort 원인 확인
2. 로봇이 overview 근방에 있으면 DART로 수동 미세 조정
3. 원인 분석 후 `docs/issues/`에 기록
4. 재시도 전 cuRobo dry-run으로 target pose 재검증
5. 운용자 2인 다시 배치 후 재시도

---

## 6. 첫 실행 완료 기준

| 항목 | 기준 |
| --- | --- |
| overview → scan pose → overview 1회 완주 | log에 `SCAN_COMPLETE` 확인 |
| joint limit 위반 없음 | executor JOINT_LIMIT_REJECT 없음 |
| 물리적 충돌 없음 | 로봇 arm, tool, 보드 간 접촉 없음 |
| 담당자 안전 이상 없음 | — |

첫 실행이 위 기준을 통과하면 `root/nw` 단일 셀 반복 실행 및
전체 4셀 순회 검토를 진행한다.
