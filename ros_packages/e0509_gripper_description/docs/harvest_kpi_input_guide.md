# 수확 실험 KPI 입력 가이드

## 언제 입력하는가

개발 중 모든 수확 시도를 입력할 필요는 없다. 초기 자동 파지 판정 보정용 표본,
실패 시도, 무작위 표본에 아래 명령을 실행한다. 최종 성공률을 보고하는 정식
반복 실험에서는 모든 시도를 입력한다.

여러 시도를 한 번에 기록할 때는 CSV 라벨 시트를 사용한다. 현재 운영 방식은
영상을 따로 찍어 후처리하지 않고, 각 시도 직후 사람이 직접 보고 즉시 라벨을
입력하는 방식이다.

```bash
python3 scripts/prepare_harvest_label_sheet.py --cell root/nw
```

생성 파일:

```text
reports/harvest_kpi/manual_labels_root_nw.csv
```

자동 열은 이미 채워진다. 사람이 채울 열은 다음뿐이다.

```text
stem_grasp, detach, retention, non_target_contact,
human_intervention, place, notes
```

누락 확인:

```bash
python3 scripts/check_harvest_logging.py --cell root/nw
```

```bash
cd ~/doosan_ws/src/e0509_gripper_description
python3 scripts/label_harvest_attempt.py
```

도구는 가장 최근 `curobo_planner_node` runtime JSONL의 마지막 수확 시도를
자동으로 선택한다. 다른 run을 판정할 때만 `--runtime <파일>`을 지정한다.
CSV를 쓰는 경우에는 `prepare_harvest_label_sheet.py`로 최신 run 목록을 만든
뒤, 방금 끝난 시도 행에 직접 값을 입력한다.

## 사람이 확인하여 입력할 항목

| 입력 시점 | 사람이 확인할 내용 | 입력 항목 |
| --- | --- | --- |
| 그리퍼 close 직후 | 실제 목표 딸기의 **줄기**를 잡았는가 | 실제 줄기 파지 |
| detach pull 직후 | 딸기가 줄기/고정부에서 분리됐는가 | 분리 성공 |
| retreat 완료 직후 | 딸기를 놓치지 않고 유지했는가 | 후퇴 유지 |
| 진입 및 후퇴 전체 | 잎, 다른 딸기, 구조물에 닿았는가 | 비목표 접촉 |
| 시도 전체 | 정지, 복구, 위치 조정 등 사람이 개입했는가 | 사람 개입 |
| Place 수행 후 | 목표 slot에 정상 배치됐는가 | Place 결과 |
| 전체 자동화 검증 시 | scan 시작부터 다음 작업 준비까지 걸린 시간 | 전체 작업 시간(초) |

전체 작업 시간은 스톱워치로 측정한 경우에만 입력하고, 모르면 Enter로 넘긴다.
자동 Pick 시퀀스 시간과 motion/planning 결과는 runtime JSONL에서 가져온다.

라벨은 실행 로그를 수정하지 않고 다음 경로에 별도로 누적된다.

```text
logs/human_labels/YYYY-MM-DD/harvest_attempt_labels.jsonl
```

## KPI 확인

```bash
python3 scripts/summarize_harvest_kpis.py
```

runtime JSONL만으로 자동 계산 가능한 계획/시간/접촉 후보 KPI:

```bash
python3 scripts/summarize_runtime_kpis.py --cell root/nw
```

핵심 KPI는 다음 6개다.

1. 실제 줄기 파지 성공률
2. 최종 Pick 성공률: 줄기 파지, 분리, 후퇴 유지가 모두 성공
3. 평균 Pick 시퀀스 시간
4. Place 성공률
5. 전체 작업 시간
6. 사람 개입률

## SafeGrasp 자동 판정 범위

`dsr_gripper_tcp`의 `/gripper_service/safe_grasp` 액션은 다음 값을 자동으로
기록할 수 있다.

- `present_position`, `present_current`, `current_delta`
- `grasp_detected`: 그리퍼에 접촉 또는 부하가 감지됐는지
- `object_lost`: 파지 후 물체 이탈이 감지됐는지
- action 성공 여부와 종료 상태

단, `grasp_detected=true`는 **목표 딸기의 줄기를 정확히 잡았다는 뜻이 아니다.**
잎이나 다른 구조물을 잡아도 접촉으로 판정될 수 있다. 따라서 SafeGrasp 연동
후에도 정식 성능 평가에서는 `stem_grasp`, `detach`, `non_target_contact`,
`place`를 사람이 시도 직후 육안으로 확인해 입력한다.

2026-06-16 보정 결과 현재 결론:

| 조건 | threshold | 결과 | 판단 |
| --- | --- | --- | --- |
| empty | 120 | 빈 파지 오검출 | 너무 낮음 |
| empty | 220 | 빈 파지 억제 | 줄기 검출에는 높을 수 있음 |
| stem_moru | 180 | 5회 중 1회 검출 | 단독 판정 불안정 |
| stem_moru | 140 | 3회 중 1회 검출 | 줄기 검출 부족 |
| empty | 140 | 3회 중 2회 오검출 | 사용 불가 |

따라서 현 단계 KPI에서 SafeGrasp는 다음처럼 해석한다.

```text
grasp_detected=true
  -> 접촉 후보 자동 기록
  -> 줄기 파지 성공 여부는 사람이 직접 입력

grasp_detected=false
  -> 빈 파지 후보 또는 약한 접촉
  -> 실제 결과는 사람이 직접 입력
```

SafeGrasp 실기 연동 순서:

1. 기존 그리퍼 실행 노드와 동시에 실행하지 않는다.
2. 단독 저속 시험으로 `max_current`와 `current_delta_threshold`를 보정한다.
3. cuRobo pick 시퀀스의 close/read-state 구간을 SafeGrasp action으로 교체한다.
4. action result와 feedback을 runtime JSONL에 저장한다.
5. 자동 판정과 사람 라벨을 비교하여 임계값의 precision/recall을 검증한다.

## Place 안전 게이트

기본값에서는 `GRASP_CONTACT_DETECTED`일 때만 Place를 허용한다.
`GRASP_UNVERIFIED` 상태에서 Place를 시험해야 한다면 사람이 실제 파지를 확인한
단일 저속 테스트에서만 다음 파라미터를 명시한다.

```text
-p allow_unverified_grasp_place:=true
```

이 옵션은 자동 파지 검증을 대신하지 않는다. 사용한 모든 시도에 사람 판정
라벨을 반드시 남긴다.

## AnyGrasp/GraspGen 적용 시점

NW 가림 셀의 첫 목표는 현재 SW 기반 rule motion이 어디서 깨지는지 KPI와
수기 라벨로 확인하는 것이다. AnyGrasp는 바로 runtime에 넣지 않고, 다음 조건이
반복 확인될 때 적용한다.

- KP1/rule 기반 접근이 줄기 꺾임 또는 잎 가림 때문에 반복 실패
- SafeGrasp는 접촉을 감지하지만 사람이 볼 때 줄기 파지가 아닌 경우가 반복
- 같은 target에서 접근 방향 후보를 더 다양하게 생성할 필요가 생김

그때 저장된 RGB-D/point cloud와 실패 라벨을 이용해 AnyGrasp를 offline으로
먼저 평가한다. 기존 rule target보다 좋은 6-DOF grasp 후보를 안정적으로
만들 때만 ROS runtime 후보 생성기로 연결한다.
