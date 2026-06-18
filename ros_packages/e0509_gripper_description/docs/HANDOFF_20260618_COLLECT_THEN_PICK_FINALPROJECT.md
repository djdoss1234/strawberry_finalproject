# 2026-06-18 Codex Handoff — NW collect-then-pick and finalproject repo

## 절대 전제

- 현재 공식 작업/커밋 대상은 `strawberry_finalproject`다.
- `/home/user/doosan_ws/src/e0509_gripper_description`는 finalproject 내부 패키지로
  연결된 작업 경로다.
- legacy miniproject 패키지는 아래에 보존되어 있고 `COLCON_IGNORE` 처리되어 있다.

```text
/home/user/doosan_ws/src/e0509_gripper_description_legacy_miniproject_20260618
```

- `scripts/측정.py`는 수정 금지.

## git/repo 복구 이력

문제:

- 5월 말 이후 일부 실전 프로젝트 작업이 `strawberry_miniproject` 쪽에 잘못
  커밋되어 finalproject GitHub 이력이 6월 8일 이후 비어 보였다.

복구:

- `strawberry_miniproject`의 `afef927` 이후 핵심 ROS 패키지/문서/워크로그를
  `strawberry_finalproject`로 이관했다.
- 추적 파일 기준 누락 0개로 확인했다.
- local-only/ignored 자산은 finalproject 작업트리에 복구했지만 Git에는 올리지 않는다.

관련 커밋:

```text
0181c98 chore: migrate harvest package from miniproject
b2c380a docs: backfill June harvest worklogs
73e7565 docs: record finalproject package path switch
3c55644 docs: explain miniproject recovery reference
602ddfb docs: record local asset recovery audit
```

관련 문서:

```text
/home/user/doosan_ws/src/strawberry_finalproject/docs/MINIPROJECT_MIGRATION_20260618.md
```

앞으로는 반드시:

```bash
git -C /home/user/doosan_ws/src/strawberry_finalproject status
git -C /home/user/doosan_ws/src/strawberry_finalproject add ...
git -C /home/user/doosan_ws/src/strawberry_finalproject commit ...
git -C /home/user/doosan_ws/src/strawberry_finalproject push
```

## 오늘 해결하려던 문제

NW 잎/줄기 가림 셀에서 detection은 잡히지만 수확 접근이 계속 실패했다.

원인 분리:

```text
기존 scan_executor 동작:
각 sub-scan pose에서 target 발견
 -> 즉시 /dsr01/curobo/pick_pose publish
 -> 그 sub-scan pose의 현재 joint branch에서 바로 pick 시작
```

문제:

- 세부 scan pose는 카메라 관측에는 유리하지만 pick 시작 branch로는 불리할 수 있다.
- `max_total_picks:=1`이면 첫 번째로 보인 후보 하나에 바로 수확 시도를 소비한다.
- 최근 로그에서는 target은 정상 stem-level로 보였지만, pre-approach 이후 final
  approach가 150/130/110/90mm에서 막히고 MoveLine fallback도 no-motion으로 실패했다.

## 오늘 구현한 것

### 1. collect-then-pick 모드

파일:

```text
/home/user/doosan_ws/src/strawberry_finalproject/src/strawberry_motion/execution/scan_executor_node.py
/home/user/doosan_ws/src/strawberry_finalproject/launch/workspace_scan.launch.py
```

구현 흐름:

```text
root/nw/nw scan
 -> root/nw/ne scan
 -> root/nw/se scan
 -> root/nw/sw scan
 -> 후보 PoseStamped 전체 수집/중복 제거
 -> root/nw 중앙 pick-ready pose로 이동
 -> best target 1개를 /dsr01/curobo/pick_pose로 publish
```

새 파라미터:

```text
collect_then_pick:=true             # workspace_scan.launch.py 기본값 true
collect_pick_ready_cell:=root/nw    # 생략 시 target_cell 사용
max_total_picks:=1                  # 한 번에 하나만 따고 재스캔 권장
```

사용되는 NW 중앙 pick-ready pose:

```text
source: config/scan_pose_candidates_refit_candidate.yaml / root/nw
TCP BASE [mm,deg] = [-225.46, 338.93, 902.31, 88.42, 87.31, -89.88]
joints_deg = [144.09, 22.90, -1.00, -238.52, -75.31, 108.68]
```

실행 중 기대 로그:

```text
COLLECT_THEN_PICK_ENABLED scan_cells=[...] pick_ready_cell=root/nw
COLLECT_TARGETS root/nw/nw kept=... total_buffer=...
COLLECT_TARGETS root/nw/ne kept=... total_buffer=...
COLLECT_TARGETS root/nw/se kept=... total_buffer=...
COLLECT_TARGETS root/nw/sw kept=... total_buffer=...
COLLECT_THEN_PICK_READY_MOVE root/nw candidates=... best=(x,y,z)mm
PICK_TRIGGER root/nw/best 1/...
```

### 2. scan 순서 변경

NW 세부 scan pose 순서를 reverse-ㄷ 형태로 맞췄다.

```text
root/nw/nw -> root/nw/ne -> root/nw/se -> root/nw/sw
```

### 3. fusion target gate

파일:

```text
ros_packages/e0509_gripper_description/scripts/strawberry_fusion_node.py
```

추가 파라미터:

```text
pick_target_max_z_m:=0.88
prefer_lower_z_target:=true
```

의도:

- NW에서 leaf/top 후보처럼 너무 높은 target을 planner에 넘기지 않는다.
- stable 후보가 여러 개면 화면 중심보다 낮은 stem-level 후보를 우선한다.

## 검증 완료

```bash
python3 -m py_compile /home/user/doosan_ws/src/strawberry_finalproject/src/strawberry_motion/execution/scan_executor_node.py
python3 -m py_compile /home/user/doosan_ws/src/strawberry_finalproject/launch/workspace_scan.launch.py
python3 -m py_compile scripts/strawberry_fusion_node.py
git -C /home/user/doosan_ws/src/strawberry_finalproject diff --check
colcon build --packages-select e0509_gripper_description strawberry_motion --allow-overriding e0509_gripper_description
```

결과:

- py_compile 통과
- diff check 통과
- colcon build 통과

## 다음 실행 커맨드 (2026-06-18 갱신, 커밋 `aae1832` 기준)

> 이 섹션은 한 번 갱신되었음. tie-break 버그 수정(`69037f0`)으로 NW가 `GRASP_POSE_REACHED`까지는
> 2회 연속 도달했으나, `measured_tcp_max_approach_m:=0.150`이 옛 0.180mm 하드 천장에 막혀 있던
> 시절의 값이라 깊이가 부족해 빈손 파지(GRASP_EMPTY)가 났다. 천장을 0.220m로 올렸으므로
> 아래는 0.200으로 더 깊게 시도하는 값으로 갱신함. 자세한 원인/근거는
> [[project_scan_pose_current]] 참고.

Planner:

```bash
source ~/doosan_ws/install/setup.bash

ros2 run e0509_gripper_description curobo_planner_node.py --ros-args \
  -p measured_tcp_plan_only:=false \
  -p direct_curobo_final_approach_for_measured_tcp:=true \
  -p measured_tcp_max_approach_m:=0.200 \
  -p measured_tcp_tool_line_after_curobo_fallback:=true \
  -p debug_dump_plan_calls:=true
```

Scan:

```bash
source ~/doosan_ws/install/setup.bash

ros2 launch strawberry_motion workspace_scan.launch.py \
  enable_robot_execution:=true \
  target_cell:=root/nw \
  enable_fusion_detection:=true \
  enable_pick_integration:=true \
  collect_then_pick:=true \
  collect_pick_ready_cell:=root/nw/pick_ready \
  max_total_picks:=1 \
  scan_movej_vel_deg_s:=20.0 \
  scan_movej_acc_deg_s2:=30.0 \
  overview_return_vel_deg_s:=20.0 \
  overview_return_acc_deg_s2:=30.0
```

`collect_pick_ready_cell`을 `root/nw/pick_ready`로 바꾼 이유: 옛 `root/nw` 중심 포즈는 J3가
특이점 근처(약 -1°)라 건강한 IK 분기가 그 위치엔 존재하지 않음(128-seed 스윕으로 확인). 4개
서브셀의 TCP 중심으로 새로 계산한 `root/nw/pick_ready`(J3=62.73°)로 교체함.

Trigger:

```bash
ros2 service call /strawberry/scan/start std_srvs/srv/Trigger "{}"
```

## 아직 못 해결한 것

- 위 깊이(0.200m)/속도(+30%) 조합은 아직 실기 미검증.
- SafeGrasp/gripper service는 간헐적으로 timeout/초기화 실패가 있었다. 현재 NW
  모션 안정화 우선이라 place/전류 기반 자동 판정은 보류.
- SW regression: `FINAL_APPROACH_VEL_MM_S/ACC`, `RETREAT_VEL_MM_S/ACC`,
  `PRE_APPROACH_SETTLE_SEC`, `STRAIGHT_RETREAT_SETTLE_SEC`는 SW와 공유되는데
  +30% 속도/단축 settle 적용 후 SW 재실행 검증을 아직 안 함.

## 다음 의사결정

1. 위 커맨드로 실제로 줄기를 잡는지(GRASP_EMPTY가 아니라 진짜 저항이 걸리는지) 확인한다.
2. 여전히 짧으면 `measured_tcp_max_approach_m`을 0.210까지만 더 올려본다(ceiling은 0.220).
3. 잡히면 SW 셀로 회귀 테스트 1회 돌려서 속도 변경이 SW를 깨지 않았는지 확인한다.
4. 한 개라도 따면 바로 재스캔한다. NW 안정화 전에는 여러 개를 한 번에 연속 pick하지 않는다.
