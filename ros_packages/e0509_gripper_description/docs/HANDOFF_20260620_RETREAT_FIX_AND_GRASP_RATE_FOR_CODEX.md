# 2026-06-20 (세션2) Handoff — retreat 2단계 분리 수정 + 파지 성공률/인식률 문제

## 절대 전제

- 공식 작업/커밋 대상: `strawberry_finalproject`. `e0509_gripper_description`은 그 안의
  symlink 패키지(`ros_packages/e0509_gripper_description`).
- **`scripts/측정.py` 절대 수정/삭제/커밋 금지.**
- 코드 수정 후 항상:
  ```bash
  python3 -m py_compile ros_packages/e0509_gripper_description/scripts/curobo_planner_node.py
  git -C /home/user/doosan_ws/src/strawberry_finalproject diff --check
  cd /home/user/doosan_ws && colcon build --packages-select e0509_gripper_description strawberry_motion \
    --allow-overriding e0509_gripper_description
  ```
- `config/scan_pose_candidates_depth2.yaml`, `ros_packages/e0509_gripper_description/config/camera_calibration_eye_in_hand.yaml`는
  의도적으로 untracked 상태 — 건드리지 말 것(커밋 금지).
- SW(다른 셀) 회귀 위험 없이 NW만 고치는 게 원칙. 모든 수정은 "틸트가 0이면 SW는 동작
  무변화"를 수학적으로 보장하는 방식으로 했음(아래 각 커밋 설명 참고).

## 브랜치 상태

- 작업 브랜치: `debug/nw-return-to-depth-good` (현재 HEAD `37056bc`)
- 베이스: `f2ec778` (Codex가 NW high 쪽에 회귀를 일으키기 전 last-known-good)
- `main`은 `8b37f34`에서 안 건드림 (Codex가 작성한 회귀 인계문서
  `HANDOFF_20260620_NW_REGRESSION_FOR_CLAUDE.md`는 main에만 있고 이 브랜치엔 없음 —
  필요하면 `git show main:ros_packages/e0509_gripper_description/docs/HANDOFF_20260620_NW_REGRESSION_FOR_CLAUDE.md`로 조회)
- **디버그 브랜치를 main에 머지할지는 아직 미결정** — 아래 미해결 항목(특히 J2 한도) 때문에
  보류 중.

## 이 세션에서 한 일 (커밋 7개, `f2ec778..HEAD`)

1. `730d4b0` — `nw_high_target_crane_z_offset_m`를 SW 공유 상수에서 분리
2. `8b59772` — open descent를 고정 보정값 대신 "실제 도달 Z - KP1 Z" 동적 계산으로 변경
3. `de2e995` — FINAL_APPROACH_TOOL_FINISH의 마지막 직선을 틸트 방향(TOOL+Z) 대신
   horiz_dir(BASE-frame 수평)로 실행 (처음엔 `is_nw_high_target` 게이트)
4. `bed6c2f` — 죽은 코드(`NW_HIGH_TARGET_CLOSE_EXTRA_DOWN_M`) 제거, 기본값 확정
   (`crane_z_offset_m=5mm`, `final_extra_m=15mm`)
5. `66f71de` — 3번 수정을 높이(`is_nw_high_target`, z≥750mm) 대신 실제 틸트
   (`abs(used_approach_dir[2])>1e-3`)로 재게이팅. z<750mm인데 같은 +15deg variant를 고른
   타겟에서도 증상이 재현된 게 근거.
6. `ff5c083` — **RETREAT_BASE/CLOSE_FAIL_RETREAT_BASE를 2단계로 분리**. 배경: 3번 수정
   이후 전진 경로가 [틸트 curobo 다리] + [수평 tool_finish 다리]로 꺾이는데, retreat은 전체
   거리를 단일 틸트 방향으로만 되돌리고 있어서 실기에서 **J2가 -97.55°/-97.7°로 한도(±95°)를
   초과**, `PICK_SEQUENCE_HOLD_LATCHED` 발생(사용자가 수동 복구). `tool_finish_executed_m`/
   `tool_finish_executed_dir`로 실제 실행된 수평 다리를 기록해 retreat을
   [수평 다리 되돌리기]→[남은 틸트 다리 되돌리기]로 분리. 틸트 0이면 no-op.
7. `37056bc` — `nw_high_target_final_extra_m`(깊이 +15mm 보정)도 같은 이유로 높이 대신
   틸트 기반으로 재게이팅. 배경: z=696mm(<750mm) 타겟이 measured_tcp_max_approach_m을
   200mm로 올려도 실제로는 180mm에서 그대로였음 — `wall_y_clamped`(아래 참고)가 뜨면
   `adaptive_dist` 계산이 항상 정확히 `baseline_approach`(180mm)로 floor-lock되기 때문에
   파라미터 자체가 무력했고, 유일한 탈출구인 `final_extra`도 높이 게이트 때문에 안 붙었음.

코드 외 진단(수정 아님): `strawberry_fusion_node.py`의 `pick_target_max_z_m`
(launch 기본값 0.88m, `workspace_scan.launch.py`의 `fusion_pick_target_max_z_m` 인자)이
실제 z=0.92~0.93m짜리 진짜 딸기를 "너무 높은 leaf/top 후보"로 오인해 매번 걸러내고 있었음
(로그: `strawberry_fusion_node_20260620T213844-1856b6ba.jsonl`, `pick_target_z_out_of_range`
63회). `fusion_pick_target_max_z_m:=0.95`로 올려서 같은 회차에 그 타겟이 통과/픽되는 것까지
확인함(`curobo_planner_node_20260620T214515-eaba59c4.jsonl`, raw z=925mm, retreat 정상 완료).

## 검증된 것 (실기)

- `ff5c083` 직후 같은 종류 타겟(z=717-719mm, +15deg 틸트)으로 재현 → `RETREAT_TOOL_FINISH_UNDO`
  → `RETREAT_BASE` 순서로 정상 완료, J2 재발 없음, `PICK COMPLETE`까지 정상 종료.
- `37056bc` 직후 z=696mm 타겟에서 `NW_HIGH_TARGET_FINAL_EXTRA: 180mm -> 195mm` 로그가
  의도대로 찍힘(틸트 게이트가 작동함).
- 우측 z=925mm 타겟(`fusion_pick_target_max_z_m` 완화 후): final_extra(15mm)+y_plane_relax
  (10mm, NW-high 전용)가 같이 붙어 총 195mm 깊이, retreat도 90mm 틸트 다리 포함 정상 완료.
- 마지막 배치(3연속 pick, `curobo_planner_node_20260620T215659-32e08412.jsonl`): 3건 모두
  retreat 정상 완료(J2 문제 없음) — 단 3건 모두 `GRASP_EMPTY`(x2)/`GRASP_UNVERIFIED`(x1)로
  실제 파지는 확인 안 됨(아래 미해결 참고).

## 미해결 — 우선순위 순

### 1. J2 한도 초과가 "특정 타겟"에서는 여전히 재발함 (안전 이슈, 최우선)

`ff5c083`(retreat 2단계 분리) 적용 후에도, x=-318mm/z=696mm 타겟에서 final_extra 적용 후
(180→195mm) 다시 J2 -97.50°로 한도 초과 (`curobo_planner_node_20260620T212751-eded81e2.jsonl`,
`pick_sequence_hold_latched` joints_deg=`[-33.13, -97.50, 95.96, -57.23, -74.16, 92.03]`).
**retreat 2단계 분리 자체는 정상 작동**(`RETREAT_TOOL_FINISH_UNDO` 125mm 성공, 마지막 70mm
틸트 `RETREAT_BASE`에서 실패) — 즉 "잘못된 방향으로 되돌리는" 원래 버그는 고쳤지만, 이
특정 타겟(좌측, J1이 -33°대로 끝남)은 OPEN_STEM_DESCENT+DETACH_PULL_DOWN까지 내려간 뒤
J2 여유가 2~3°밖에 안 남는 것으로 보임. 같은 깊이(195mm)였던 우측 타겟들(J1이 -76~-85°대로
끝남, 완전히 다른 elbow 형상)은 전부 무사했음 — 깊이 자체가 원인이 아니라 이 특정 좌측
타겟의 IK 분기가 원래 마진이 거의 없는 것으로 추정됨. **이 가설은 검증 안 됨** —
같은 타겟을 `nw_high_target_final_extra_m:=0.000`(180mm로 되돌림)로 다시 시도해서 retreat이
안전해지는지 A/B 비교가 필요함. 안전해지면 final_extra을 x-구간별로 게이팅하거나, 안 되면
retreat 전에 목표 joint state를 미리 IK/FK로 검사해서 J2 마진이 부족하면 사전에 막는
구조적 가드가 필요할 수 있음(더 큰 변경, 신중히 설계할 것).

### 2. 파지 성공률이 낮음 — 같은 +15deg 틸트 variant만 매번 선택됨

마지막 3연속 배치 전부 `GRASP_EMPTY`/`GRASP_UNVERIFIED`(확정 성공 0건). 사용자 관찰:
"접근을 죄다 옆으로 함" — `MEASURED_TCP_GRASP_QUAT_RETRY_VARIANTS`/
`NW_HIGH_TARGET_GRASP_QUAT_RETRY_VARIANTS` 둘 다 `+15deg`를 1순위로 두고 있고, depth-probe
동률 시 J3 건강도 tie-break + `MEASURED_TCP_J3_GOOD_ENOUGH_DEG`(40°) early-exit 때문에
+15deg가 거의 항상 그 자리에서 바로 채택되어 나머지(flatter) variant는 한 번도 시도되지
않음. 이게 파지 실패의 실제 원인인지는 **검증 안 됨** — 가능한 다음 실험: variant 순서를
바꾸거나 early-exit 임계값을 낮춰서 flatter variant를 강제로 한 번 시도해보고 파지 확인율을
비교. 추측성 수정 금지 원칙([[feedback_debug_before_fix]]) 적용 — 데이터 없이 순서/임계값
바꾸지 말 것.

### 3. 인식률 낮음 — 다수 타겟이 detection 단계에서 탈락

사용자: "5개중에 3개는 아예 인식을 못함". 마지막 배치와 동시에 돈
`strawberry_fusion_node_20260620T215703-bce59102.jsonl` 확인 결과 `pick_target_rejected`
사유: `stem_keypoint_depth_invalid` 219회, `stem_geometry_implausible` 22회,
`stem_keypoint_low_confidence` 3회 (`stable_pick_target_published` 180회는 있었지만 실제
distinct 타겟 3개만 픽으로 이어짐 — 같은 타겟이 반복 publish되는 구조). 높이 필터
(`pick_target_z_out_of_range`)는 이번엔 거의 없었음(`fusion_pick_target_max_z_m:=0.95`로
완화한 효과). 즉 이번엔 키포인트 깊이 무효(아마 잎/타 줄기에 가려 stereo depth가 안 잡힘)가
주된 탈락 원인 — `strawberry_fusion_node.py`의 detection/keypoint depth 로직 쪽 문제이고
`curobo_planner_node.py`와는 무관. 이 세션에서 손 안 댐.

### 4. wall_y_clamped(FK calibration drift)가 거의 매 pick마다 발생

Detection Y가 등록된 벽(672mm)보다 90~260mm 더 깊게(뒤쪽으로) 잡히는 현상이 이 세션
내내 거의 모든 pick에서 발생(`Detection Y=...mm > wall surface 672mm` 경고). 현재는
`adaptive_dist` floor-lock + NW-high `y_plane_relax`/`final_extra`로 "우회"하고 있을 뿐
근본 원인(카메라 calibration 또는 wall pose 등록 자체의 드리프트)은 안 고침. 사용자가
이전에 X축 좌우 드리프트는 "캘리브 문제 아님"이라고 결론 내린 바 있으나(그건 그대로
존중), **Y축 드리프트는 방향이 항상 일정(항상 더 깊게 나옴, 한 번도 반대로 나온 적 없음)**
이라 X와는 성격이 다른 진짜 calibration/wall-registration 이슈일 가능성이 있음. 다음에
시간이 나면 `calibration_eye_in_hand_1.npz`나 `panel_registration.yaml`(SVD wall pose)
재검증을 고려할 것 — 단, 사용자 명시적 동의 없이 재캘리브레이션 절차에 들어가지 말 것
(범위가 큼).

### 5. 그리퍼 serial 에러 산발적 재발

이번 세션 중 한 번 `current_raw=-3`(invalid state/virtual or serial error) 발생
(`curobo_planner_node_20260620T214515-eaba59c4.jsonl`). 기존에 알려진 패턴대로
`gripper_service_node` 재시작(DRL 재업로드 포함)이 필요. 세션 종료 시점까지 재시작
안 했을 가능성 있음 — 다음 세션 시작 전 확인할 것.

## 다음 실행 커맨드 (이 브랜치 기준, 검증된 파라미터)

```bash
# planner
ros2 run e0509_gripper_description curobo_planner_node.py --ros-args \
  -p measured_tcp_plan_only:=false \
  -p direct_curobo_final_approach_for_measured_tcp:=true \
  -p measured_tcp_max_approach_m:=0.200 \
  -p measured_tcp_tool_line_after_curobo_fallback:=true \
  -p debug_dump_plan_calls:=true

# scan — fusion 높이필터 완화 + collect-then-pick 무제한(max_total_picks 생략 = 0 = 무제한)
ros2 launch strawberry_motion workspace_scan.launch.py \
  enable_robot_execution:=true target_cell:=root/nw \
  enable_fusion_detection:=true enable_pick_integration:=true \
  collect_then_pick:=true collect_pick_ready_cell:=root/nw/pick_ready \
  fusion_pick_target_max_z_m:=0.95 \
  scan_movej_vel_deg_s:=20.0 scan_movej_acc_deg_s2:=30.0 \
  overview_return_vel_deg_s:=20.0 overview_return_acc_deg_s2:=30.0

# trigger
ros2 service call /strawberry/scan/start std_srvs/srv/Trigger "{}"
```

안전 주의: 좌측(x≈-300~-320mm 부근) 타겟이 나오면 retreat 중 J2 한도초과로
`PICK_SEQUENCE_HOLD_LATCHED`가 또 발생할 수 있음(미해결 1번) — 무인 연속 실행하지 말고
지켜보다가 멈추면 수동 개입할 것.

## 참고용 로그 파일 (이 인계문서가 인용한 증거)

- `logs/runtime/2026-06-20/curobo_planner_node_20260620T212751-eded81e2.jsonl` — J2 -97.5
  재발 사례 (x=-318mm, z=696mm)
- `logs/runtime/2026-06-20/curobo_planner_node_20260620T214515-eaba59c4.jsonl` — 우측 높은
  타겟(z=925mm) 성공적 retreat + fusion 높이필터 완화 효과 확인
- `logs/runtime/2026-06-20/curobo_planner_node_20260620T215659-32e08412.jsonl` — 마지막
  3연속 배치(전부 GRASP_EMPTY/UNVERIFIED, retreat은 전부 정상)
- `logs/runtime/2026-06-20/strawberry_fusion_node_20260620T213844-1856b6ba.jsonl` — 높이필터
  오탈락 증거(`pick_target_z_out_of_range` 63회, target z≈0.92-0.93m)
- `logs/runtime/2026-06-20/strawberry_fusion_node_20260620T215703-bce59102.jsonl` — 인식률
  저하 증거(`stem_keypoint_depth_invalid` 219회 등)
