# 2026-06-18 NW 셀 수확 모션 — Codex 인계 (세션 종료)

이 문서는 오늘(2026-06-18) NW 셀 수확 모션 안정화 작업 전체를 정리한 최종 인계 문서다.
같은 날짜의 `HANDOFF_20260618_NW_MOTION_DEBUG.md`, `HANDOFF_20260618_COLLECT_THEN_PICK_FINALPROJECT.md`
는 이 문서가 작성된 시점 기준으로 **일부 내용이 stale**하다(예: pick-ready pose가 이후 교체됨).
이 문서를 우선 참고할 것.

## 0. 작업 범위 (변하지 않음)

NW 셀 수확 모션 안정화만. place, AnyGrasp, KPI 작업, 큰 리팩터링 금지.
**`scripts/측정.py` 절대 수정/삭제/커밋 금지.** 커밋은 항상
`~/doosan_ws/src/strawberry_finalproject`에서만 한다 (`e0509_gripper_description`은
`ros_packages/e0509_gripper_description`의 symlink — 같은 파일임).

## 1. 공식 repo / 최신 커밋

- 공식 repo: `~/doosan_ws/src/strawberry_finalproject`
- 최신 커밋은 `git log --oneline -5`로 확인할 것. 이 문서 원본 작성 직후 기준은
  `27bb50e`였고, 이후 Codex가 collect target ranking 수정을 추가했다.
- 이번 세션에서 push된 커밋 순서: `1d9bb59` → `7004a3a` → `69037f0` → `aae1832`

### 1.1 Codex 추가 수정 (27bb50e 이후)

`27bb50e` 인계 이후 Codex가 로그를 다시 확인했을 때, collect-then-pick 자체는
정상 동작했지만 후보 선택 기준이 문제였다.

확인된 로그:

```text
COLLECT_TARGETS root/nw/se kept=1 total_buffer=1
COLLECT_TARGETS root/nw/sw kept=1 total_buffer=2
COLLECT_THEN_PICK_READY_MOVE root/nw/pick_ready candidates=2 best=(-373,771,765)mm
```

즉, "다른 딸기"를 노린 이유는 collect 후 best target이 이전 성공 타겟
`x≈-255mm`가 아니라 더 왼쪽 `x≈-373mm`로 선택됐기 때문이다.

수정:

- collect-then-pick mode에서는 첫 detection이 들어와도 dwell을 바로 끊지 않고
  scan dwell 전체 동안 후보를 더 모은다.
- collect 후 best target은 기존 lower-left-first 정렬이 아니라
  `collect_pick_ready_cell`의 TCP 중심에 X/Z가 가까운 후보를 우선한다.
- 로그에 후보 ranking을 남긴다.

기대 로그:

```text
COLLECT_PICK_READY_RANK center=(-292,670,854)mm (...)
COLLECT_THEN_PICK_READY_MOVE root/nw/pick_ready candidates=N best=(...)mm
```

### 1.2 추가 버그 발견/수정: ranking 후 publish 직전 재정렬

이후 로그를 다시 확인했을 때, ranking 자체는 정상인데 실제 publish 직전에 순서가
다시 바뀌는 버그가 있었다.

확인된 문제 로그:

```text
COLLECT_PICK_READY_RANK center=(-292,670,854)mm (-252,795,827)mm score=49 ...
COLLECT_THEN_PICK_READY_MOVE root/nw/pick_ready candidates=4 best=(-252,795,827)mm
PICK_TRIGGER root/nw/best 1/4 pos=(-374,771,765)mm
```

원인:

- collect mode에서 `unique_all`을 `_rank_poses_for_pick_ready()`로 올바르게 정렬했다.
- 하지만 `_trigger_picks_for_cell()` 내부가 항상 `_deduplicate_poses()`를 다시 호출했다.
- `_deduplicate_poses()`는 lower-left/stem-level 우선 정렬을 포함하므로, 이미 정한
  pick-ready 기준 ranking을 다시 깨뜨렸다.

수정:

- `_trigger_picks_for_cell(..., poses_are_ranked=True)` 옵션 추가.
- collect-then-pick의 best target publish에서는 이미 정렬된 리스트를 그대로 사용한다.

수정 후 기대 로그:

```text
COLLECT_PICK_READY_RANK ... (-252,795,827)mm score=...
COLLECT_THEN_PICK_READY_MOVE ... best=(-252,795,827)mm
PICK_SEQUENCE_USING_RANKED_ORDER root/nw/best first=(-252,795,827)mm
PICK_TRIGGER root/nw/best 1/4 pos=(-252,795,827)mm
```

의도:

- NW 중앙 pick-ready branch에서 물리적으로 더 가까운 후보를 먼저 시도한다.
- `x=-373mm`처럼 왼쪽 끝 후보가 먼저 선택되어 final approach가 막히는 일을 줄인다.
- 이 변경은 `strawberry_motion`의 scan target forwarding 정책만 바꾸며,
  `curobo_planner_node.py`의 pick motion 자체는 건드리지 않는다.

## 2. 오늘 한 일 (시간순)

### 2.1 root/nw pick-ready pose 교체 (`1d9bb59`)

- 기존 `root/nw` 중심 포즈는 J3(팔꿈치) ≈ -1°로 특이점 근접 — 128-seed IK 스윕으로
  **그 위치엔 건강한 분기 자체가 없음**을 확인.
  도구: `scripts/compute_nw_pick_ready_pose.py`
- 해결: NW 4개 서브셀(`root/nw/nw,ne,sw,se`) TCP 위치의 중심점에서 새로 IK를 풀어
  `root/nw/pick_ready` 항목 추가 (J3=62.73°, `config/scan_pose_candidates_refit_candidate.yaml`).
  4개 서브셀 전부에서 ≤36° joint delta로 도달 가능 확인됨.
- **collect-then-pick의 `collect_pick_ready_cell`은 이제 `root/nw/pick_ready`를 써야 한다.**
  (기존 두 stale 문서는 여전히 `root/nw`로 되어 있음 — 틀린 값.)

### 2.2 디버그 덤프 + 리플레이 도구 (`7004a3a`)

- `debug_dump_plan_calls:=true` 파라미터: `plan()` 호출 직전 입력(start_joints, target,
  world cuboids/spheres 등)을 JSONL에 그대로 기록.
- `scripts/replay_plan_call_dump.py`: 그 JSONL을 읽어 동일 입력으로 오프라인 재생,
  실기 결과와 비교.
- **왜 필요했나**: 손으로 로그값을 베껴서 만든 재현 테스트는 소수점 근사 때문에 실기와
  다른 결과를 냈다. 정확한 입력을 그대로 재생해야 진짜 원인을 찾을 수 있었다.
  ([[feedback_debug_before_fix]] 참고 — 증거 없는 수정 금지 원칙)

### 2.3 진짜 원인 발견 + tie-break 수정 (`69037f0`)

- NW measured-TCP pick은 6개 회전 variant 중 최대 4개가 **정확히 같은 깊이**에서 성공하는데,
  팔꿈치(J3) 건강도가 variant마다 전혀 다름(-0.0°/28.9°/42.8°/52.5° 사례 확인).
- 기존 선택 로직이 `depth_m > best`(엄격한 초과)만 봐서 동률이면 **항상 가장 먼저 시도한
  (최악의) variant**를 유지 — 이게 Doosan MoveLine "success인데 실제로는 안 움직이는" 버그의
  실제 원인이었음(J3≈0° 특이점 근접).
- 수정: 동률일 때 더 건강한 J3로 교체하는 tie-break 추가.
- **결과**: 실기에서 NW가 처음으로 `GRASP_POSE_REACHED`에 2회 연속 도달.
  단, 그리퍼는 빈손(GRASP_EMPTY, pos=700까지 완전히 닫힘 — 그리퍼는 정상 작동, 위치가
  짧았던 것).

### 2.4 180mm 하드 천장 발견 + 속도 개선 (`aae1832`)

- 원인: 코드에 `min(0.180, measured_tcp_max_approach_m)`라는 하드코딩된 천장이 있어서,
  파라미터로 0.200을 줘도 실제로는 0.180에서 잘렸다. 그래서 줄기보다 20~30mm 못 들어가
  GRASP_EMPTY가 났던 것.
- 수정: `MEASURED_TCP_MAX_APPROACH_CEILING_M = 0.220`으로 천장을 별도 상수로 분리해 상향.
- 추가: `MEASURED_TCP_J3_GOOD_ENOUGH_DEG = 45.0` — 깊이 탐색 중 J3가 이미 45° 이상이면
  나머지 회전 variant 탐색을 건너뜀 (불필요한 IK 계산 15~20초 절약, NW-only).
- 속도 +30% (사용자 요청): `FINAL_APPROACH_VEL/ACC`, `RETREAT_VEL/ACC`
  (**SW와 공유**), `CRANE_DESCENT/ASCENT_VEL` (NW-only). settle 시간 0.5→0.3s 단축.
- **이 시점까지는 전부 실기 미검증 상태로 다음 테스트를 기다리고 있었음.**

### 2.5 사실 정정: SW도 measured_tcp_260mm를 쓴다

- 처음에는 "NW는 measured_tcp_260mm, SW는 legacy_160mm라서 깊이를 직접 비교 못 한다"고
  설명했는데 **틀렸다.**
- 확인 결과: `tool_model_profile` 파라미터 기본값은 2026-06-11 도입 이후 줄곧
  `measured_tcp_260mm`였고, SW 성공 런 커맨드(2026-06-11 PLACE_CUROBO, 2026-06-14
  PLACE_TRAY_GRID)도 이 파라미터를 따로 넘기지 않아 **기본값(260mm 모델)을 그대로 썼다.**
  `legacy_160mm`는 코드에 "문제 생기면 되돌리는 롤백 옵션"으로만 존재했지 실제 SW 성공
  경로가 아니었다.
- 결론: SW/NW는 같은 모델·같은 깊이 기준을 쓴다. 180→220mm 천장 수정 방향은 그대로
  맞지만, "모델이 달라서"라는 설명은 삭제. 실제 이유는 "NW 특정 타겟이 SW보다 더 깊은
  접근이 필요한데 천장이 막고 있었다"였다.
- 메모리 `project_scan_pose_current.md`에도 같은 오류가 있어 정정해뒀음.

## 3. 가장 최근 실기 로그 (0.200m 테스트, 새로 발견된 문제)

`measured_tcp_max_approach_m:=0.200`, `debug_dump_plan_calls:=true`로 실행한 결과:

```text
WARN: Detection Y=771mm > wall surface 672mm (FK calibration drift) — clamped to 672mm
INFO: PICK 딸기 raw=(-373,672,765)mm grasp=(-373,672,765)mm
... depth probe: 200/150/130/110/90mm 전부 IK_FAIL, 70mm만 성공(두 variant 모두) ...
WARN: FINAL_APPROACH_STRAIGHT_BASE skipped (measured TCP는 항상 cuRobo 경로 사용)
WARN: FINAL_APPROACH_TOOL_FINISH: cuRobo reached 70mm only; executing remaining 110mm with TOOL +Z MoveLine
WARN: MoveLine returned early (0.15s < expected 2.54s)
ERROR: MoveLine reported success but joints barely moved (max_delta=0.01deg); treating as failed
ERROR: ABORT: 직선 진입 실패
```

### 3.1 이게 무슨 뜻인가 (사용자 질문 답변)

**"이번엔 다른 딸기 노린 거 같은데" — 맞다.** 좌표 `x=-373mm`는 이전 두 번의
`GRASP_POSE_REACHED` 성공 타겟(`x≈-255mm`, 메모리/`root/nw/pick_ready` note 참고)과
다르다. NW 서브셀 중 더 왼쪽/아래(`root/nw/sw`, `root/nw/se` 근처, x=-464~-141mm 범위)
쪽 딸기를 본 것으로 보인다.

**"더 깊이 들어가야 된다는 건 확인했음?" — 아니, 이번 로그로는 확인도 반박도 안 됐다.**
이번 타겟은 천장(0.220m)에 막힌 게 아니다. cuRobo가 200/150/130/110/90mm **전부 IK_FAIL**
판정했다 — 즉 천장을 올려도 의미가 없을 정도로, 이 특정 타겟에선 70mm보다 깊은 곳이
물리적으로(혹은 충돌 제약상) 도달 불가능했다. 천장 수정이 효과 있는지 확인하려면
**이전에 성공했던 것과 같은 타겟(x≈-255mm 근처)으로 다시 테스트해야 한다.** 이번 로그는
새로운 다른 문제를 보여준 것.

**"얘는 아예 직선진입을 못하네" — 맞다, ABORT까지 갔다.** 원인 분석:

1. perception이 이 딸기의 Y를 771mm(벽보다 99mm 뒤쪽, 즉 벽을 뚫고 들어간 위치)로
   감지했다 — **이 자체가 calibration drift 의심 신호**다. 벽 뒤에 줄기가 있을 수 없으므로
   이 좌표(특히 X, Z도 같은 detection에서 나왔으므로 같이 의심)는 신뢰도가 낮다.
2. 코드가 Y를 672mm(벽면)로 clamp했지만, X/Z는 그대로 썼다 — 그 결과 만들어진 3D 목표가
   실제 줄기 위치가 아닐 가능성이 있다.
3. 그 (의심스러운) 목표에 대해 cuRobo는 70mm까지만 plan 가능했고, 남은 110mm를
   Doosan TOOL +Z MoveLine으로 마무리하려다 또 "success인데 무동작" 버그가 재발했다.
   **이번엔 J3=-34.2°로 특이점 근처가 아니었다** — 즉 tie-break 수정이 막아주는
   "팔꿈치 특이점" 케이스가 아니라, **cuRobo도 못 찾는 진짜 도달 불가 영역**(충돌 또는
   관절 한계)을 Doosan MoveLine이 "성공"으로 속여서 보고하는, **더 일반적인 패턴**일
   가능성이 높다. 즉 "MoveLine 무동작 버그 = J3 특이점 때문"이라는 기존 가설은
   **일부만 맞았다** — cuRobo 자체가 IK_FAIL인 영역에서도 같은 증상이 재현된다.

### 3.2 다음 사람이 검증해야 할 가설 (코드 수정 전에 증거부터 — [[feedback_debug_before_fix]])

- 가설 A: "MoveLine 무동작"은 요청한 직선 끝점이 cuRobo IK로도 안 풀리는 영역일 때 항상
  재현된다(특이점 여부와 무관). → `debug_dump_plan_calls` 로그 + 이번 JSONL 파일로
  `final_approach_tool_finish_requested`/`final_approach_fallback_requested` 이벤트의
  타겟 좌표를 cuRobo로 직접 plan 시도해서 IK_FAIL 나는지 확인.
- 가설 B: 이번 타겟 자체가 calibration drift로 잘못된 3D 좌표였다 — 같은 서브셀
  (`root/nw/sw` 또는 `root/nw/se`)에서 재스캔해서 같은 딸기가 다시 비슷하게 잘못된 Y로
  잡히는지 재현성 확인 필요. 재현되면 perception/extrinsic 쪽 문제, 1회성이면 무시 가능.
- 이 JSONL 로그 파일 경로: `/home/user/doosan_ws/src/e0509_gripper_description/logs/runtime/2026-06-18/curobo_planner_node_20260618T184001-1adc603f.jsonl`
  (`replay_plan_call_dump.py`로 재생 가능)

## 4. 해결된 것 (정리)

1. NW pick-ready pose 특이점 문제 — 해결 (`root/nw/pick_ready`)
2. depth-probe tie-break 버그 (항상 최악의 variant 유지) — 해결, 실기 확인됨
   (`GRASP_POSE_REACHED` 2회 연속)
3. 180mm 하드 천장이 파라미터를 무시하고 깊이를 잘랐던 문제 — 해결 (0.220m로 상향)
4. 불필요한 IK variant 탐색 — J3 good-enough early-exit로 개선
5. 속도 +30%, settle 시간 단축 — 코드 반영 완료
6. "SW는 legacy_160mm" 라는 잘못된 전제 — 정정 완료 (둘 다 measured_tcp_260mm)
7. collect-then-pick ranking 후 publish 직전 재정렬 문제 — 해결
   (`PICK_SEQUENCE_USING_RANKED_ORDER` 로그로 확인 가능)
8. NW measured-TCP 후보 탐색 낭비 — 개선
   - `+15deg` grasp orientation을 먼저 시도한다. 직전 실기 로그에서 같은 90mm depth에서
     J3=53.6deg로 가장 건강한 branch였기 때문이다.
   - 첫 성공 depth가 나온 뒤에는 다음 orientation에서 그보다 깊은 IK_FAIL 후보를 반복
     검사하지 않는다.
   - probing 때 이미 계산한 final cuRobo plan을 실행 시 재사용한다. 기존에는 같은 90mm
     final plan을 버리고 실행 단계에서 다시 fallback search를 수행했다.
9. 파지점 미세 보정용 ROS 파라미터 추가
   - `pick_target_x_bias_m` (default 0.0)
   - `pick_target_z_bias_m` (default `GRASP_Z_BIAS`, 현재 0.0)
   - 로그의 `PICK 딸기 raw=... grasp=... x_bias=... z_bias=...`로 실제 보정량 확인 가능.
10. NW high target 접근 높이/깊이 보정 추가
   - 2026-06-18 19:07 run에서 target Z≈825mm는 접근 자체는 성공했지만,
     육안상 파지점보다 약 30mm 높고 얕게 들어가 `GRASP_EMPTY`가 발생했다.
   - 19:18 run에서 final approach를 180→210mm로 늘리자 TOOL finish가 90→120mm가 되었고,
     Doosan MoveLine이 "success but no motion"으로 실패했다.
   - 19:22 run에서는 높이는 괜찮아졌고 깊이만 10~20mm 부족한 것으로 관찰됐다.
   - 하지만 15mm final_extra를 넣자 +15deg 접근 방향을 따라 들어가며 옆으로 빗겨가는
     부작용이 확인됐다. 따라서 기본값은 다시 0mm로 둔다.
   - 기본값:
     - `nw_high_target_z_threshold_m:=0.750`
     - `nw_high_target_final_extra_m:=0.000`
     - `nw_high_target_close_extra_down_m:=0.000`
   - 현재 기대 로그:
     - `NW_HIGH_TARGET_KP1_OFFSET: pre-approach and open descent 30mm -> 15mm`
     - `NW_HIGH_TARGET_KP1_OFFSET_DESCENT: 30mm -> 15mm`
   - 이것은 장기적인 perception/KP target correction이 아니라, 현재 NW high-cell 실기
     편차를 흡수하는 제한적 보정이다. 실제 근본 해결은 KP1/KP2 stem target 정확도와
     eye-in-hand/depth drift를 별도로 줄여야 한다.
11. NW high target은 건강한 J3만 보지 말고 수평 branch 우선
   - +15deg는 J3가 건강하고 계산은 잘 되지만 접근선이 위/옆으로 빗겨가는 경향이 있다.
   - 깊이 extra를 거리로 더 밀면 같은 방향으로 더 빗겨가므로 기본값에서 끈다.
   - NW high target 전용 variant order:
     `0deg -> +5deg -> -5deg -> +10deg -> -10deg -> +15deg`
   - 같은 final depth에서 여러 branch가 성공하면, J3가 최소 안전선(20deg)을 넘는 후보 중
     더 수평에 가까운 branch를 우선한다. 이전처럼 무조건 J3가 더 큰 branch를 고르면
     접근선이 위/옆으로 빗겨갈 수 있기 때문이다.
12. 19:36 실기 결과: +10deg branch로 바뀌었지만 깊이는 크게 개선되지 않음
   - 로그: `curobo_planner_node_20260618T193657-7d510ad0.jsonl`
   - target: raw=(-248,672,819)mm
   - 선택 branch: `variant=('base', [1, 0, 0], 10.0)`, J3=47.0deg
   - final path: cuRobo 90mm + TOOL finish 90mm = total 180mm
   - open descent: 60mm
   - 결과: `GRASP_EMPTY` (present_pos=700)
   - 결론: orientation을 +15deg에서 +10deg로 낮춰도 cuRobo가 허용한 final depth는 여전히
     90mm이고 총 접근 깊이도 180mm라서, 깊이 부족 자체는 해결되지 않았다.
   - 즉 현재 병목은 "branch 각도"보다 **target depth / TCP frame / final straight segment**
     쪽에 더 가깝다.
13. -130mm TCP final standoff 실험은 실패 → -120mm로 복귀
   - 19:36 run 이후 관찰은 "높이는 대체로 맞고 깊이만 10~20mm 부족"이었다.
   - `nw_high_target_final_extra_m`처럼 approach distance를 늘리면 TOOL 방향의 기울기 때문에
     옆으로 빗겨가거나 MoveLine 무동작이 생겼다.
   - 37fefe8에서 measured-TCP final standoff를 -120mm에서 -130mm로 바꿔 총 진입을
     180mm→190mm로 늘렸지만, cuRobo는 여전히 90mm까지만 도달했고 남은 TOOL finish가
     90mm→100mm로 증가했다.
   - 2026-06-20 15:40 run 결과, 100mm TOOL finish에서 Doosan MoveLine이 다시
     "success but joints barely moved"로 실패했다.
   - 결론: 깊이를 TCP standoff로 10mm 늘리는 방식도 현재 직선진입 신뢰 한계를 넘긴다.
     따라서 -120mm로 복귀하고, 깊이 문제는 target definition/perception/TCP 실측 검증으로
     분리한다.
   - 기본값:
     - `MEASURED_TCP_FINAL_STANDOFF_M = -0.120`
     - `nw_high_target_base_y_nudge_m:=0.000`
   - 기대 로그:
     - `NW_HIGH_TARGET_VARIANT_ORDER: +0deg, +5deg, -5deg, +10deg, -10deg, +15deg`
     - `GRASP_POSE_REACHED ... pre=6cm+180mm+0mm ...`
   - 실행 커맨드에서 `-p nw_high_target_base_y_nudge_m:=0.010`를 넘기면 다시 후처리
     10mm nudge가 켜지므로, 다음 테스트에서는 반드시 빼야 한다.
14. 2026-06-20 15:45 run: 수평 branch pruning 버그 수정
   - `0deg` branch는 side-drift가 적지만 해당 run에서는 cuRobo final depth가 70mm까지만
     성공했다.
   - 기존 pruning은 "best depth가 하나라도 있으면 다음 variant는 그 depth 이하만 검사"라서,
     70mm가 best가 된 순간 +5/+10deg 후보의 90mm endpoint를 아예 검사하지 않았다.
   - 그 결과 남은 TOOL finish가 110mm가 되어 Doosan MoveLine success/no-motion이 재발했다.
   - 수정: `MEASURED_TCP_MIN_PRUNE_DEPTH_M = 0.090` 추가. 최소 90mm cuRobo depth를 찾기
     전에는 variant pruning을 하지 않는다.
   - 기대 로그:
     - `MEASURED_TCP_PROBE_NOT_PRUNED: existing best depth=70mm < 90mm minimum`
     - 이후 +5/+10 branch에서 `MEASURED_TCP_FINAL_PROBE_BEST depth=90mm ...`
   - 목표: 예전에 직선진입이 됐던 `90mm cuRobo + 90mm TOOL finish = 180mm` 구조를 복구.
15. 2026-06-20 15:49 run: 직선진입 복구, 단 +15deg tilted descent로 밀림 의심
   - 로그: `curobo_planner_node_20260620T154912-04e3a3c3.jsonl`
   - pruning 수정 후 직선진입은 다시 성공했다.
   - 최종 선택은 `variant=('base', [1,0,0], +15.0)`, cuRobo 90mm + TOOL finish 90mm.
   - 하지만 열린 그리퍼 상태에서 `BASE -Z 60mm` 하강했고, 사용자 관찰상 수평이 아닌
     그리퍼가 딸기를 같이 밀어 움직였을 가능성이 있다.
   - 시간 문제도 확인됨: NW high에서 200/150/130/110mm는 반복적으로 IK_FAIL인데 매 run마다
     모두 probe하여 실패 로그와 시간이 과도하게 발생했다.
   - 수정:
     - `NW_HIGH_TARGET_PROBE_DEPTHS_M = [0.090, 0.070, 0.060]`
       - NW high에서는 검증된 90mm부터 probe하여 불가능한 깊은 후보를 건너뛴다.
     - `NW_HIGH_TARGET_EXTRA_DOWN_MAX_TILT_DEG = 10.0`
       - +15deg처럼 기울어진 branch가 선택되면 `close_extra_down` 30mm를 생략하고
         기본 open descent 30mm만 수행한다.
   - 기대 로그:
     - 깊은 IK_FAIL 후보가 줄어든다.
     - +15deg 선택 시 `NW_HIGH_TARGET_CLOSE_EXTRA_DOWN_SKIPPED`가 뜬다.
   - 목적: 직선진입은 유지하되, tilted open descent가 딸기를 밀어 파지 실패로 이어지는
     문제를 줄인다.
16. 2026-06-20 16:04 run: 여전히 얕음, 실패 로그 과다
   - 로그: `curobo_planner_node_20260620T160401-7a81cb4e.jsonl`
   - 직선진입은 성공했고, `close_extra_down`은 의도대로 skip되어 open descent가 30mm로 줄었다.
   - 하지만 결과는 여전히 `GRASP_UNVERIFIED`, 사용자는 "너무 얕다"고 판단.
   - 실패 로그가 많은 이유:
     - NW high에서 +15deg가 사실상 90mm depth를 만드는 유일한 branch인데, 기존 순서가
       `0,+5,-5,+10,-10,+15`라서 실패 branch를 다 지나간 뒤에야 성공 branch를 찾았다.
   - 얕음의 원인 가설:
     - `Detection Y≈800mm`를 `wall_y=672mm`로 clamp하면서 target depth가 지나치게 앞쪽으로
       고정된다.
     - final move 끝에서 더 밀면 MoveLine 한계 또는 side-drift가 생기므로, motion 끝단 보정이
       아니라 target plane 정의를 제한적으로 보정한다.
   - 수정:
     - NW high variant order를 `+15deg -> 0deg -> +5deg -> -5deg -> +10deg -> -10deg`로 변경.
       +15deg를 먼저 시도해 반복 IK_FAIL 시간을 줄인다.
     - `NW_HIGH_TARGET_Y_PLANE_RELAX_M = 0.010` 추가.
       wall clamp된 NW high target의 Y를 계획 전에 10mm만 안쪽으로 이동한다.
   - 기대 로그:
     - `NW_HIGH_TARGET_Y_PLANE_RELAX: clamped target Y 672mm -> 682mm`
     - `NW_HIGH_TARGET_VARIANT_ORDER: +15deg, +0deg, +5deg, -5deg, +10deg, -10deg`
   - 주의: 이것도 장기적으로는 calibration/perception target 정의로 해결해야 한다. 다만
     최종 MoveLine에 후처리 오프셋을 붙이는 것보다 원인에 가까운 target-plane 보정이다.
17. 2026-06-20 16:09 run: 깊이는 개선, 접근점이 아직 높아 주변 줄기와 겹침
   - 로그: `curobo_planner_node_20260620T160902-deba2eb6.jsonl`
   - `NW_HIGH_TARGET_Y_PLANE_RELAX` 이후 사용자가 "더 깊이 들어갔다, 깊이는 괜찮다"고 판단.
   - 남은 문제는 파지점보다 접근 종료점이 아직 높아, 열린 그리퍼로 내려오는 동안
     주변 줄기가 같이 겹치며 그리퍼가 헐렁하게 닫히는 느낌이 있다는 점.
   - SW에서 좋았던 느낌은 접근 위치가 파지점보다 과하게 높지 않았기 때문일 가능성이 있다.
   - 수정:
     - `NW_HIGH_TARGET_CRANE_Z_OFFSET_M = 0.015`
     - `NW_HIGH_TARGET_CLOSE_EXTRA_DOWN_M = 0.000`
     - NW high에서만 접근 종료점을 기본 30mm 위에서 15mm 위로 낮춘다.
     - 열린 하강 거리도 같은 15mm로 맞춰, 계획 지점과 실제 close 지점이 함께 이동한다.
     - 전역 `CRANE_Z_OFFSET_M`은 SW/다른 셀 회귀를 피하려고 건드리지 않는다.
   - 기대 로그:
     - `NW_HIGH_TARGET_KP1_OFFSET: pre-approach and open descent 30mm -> 15mm`
     - `NW_HIGH_TARGET_KP1_OFFSET_DESCENT: 30mm -> 15mm`
     - `OPEN_STEM_DESCENT BASE REL xyz=[0.0, 0.0, -15.0]mm`
   - 목적: 깊이는 유지하되, 접근점과 close 지점을 함께 KP1에 가깝게 내려 주변 줄기
     동시 파지와 fruit push를 줄인다. 단순 후처리 오프셋 떡칠이 아니라 접근 기준점을
     바꾸는 수정이다.

## 5. 아직 안 된 것 / 새로 발견된 것

1. NW high target은 접근과 하강은 되지만 여전히 `GRASP_EMPTY`다.
   현재 성공한 실제 접근 깊이는 cuRobo 90mm + TOOL finish 90mm = total 180mm이고,
   이보다 깊게 하려는 단순 final_extra는 빗김 또는 MoveLine 무동작을 만들었다.
2. 깊이 부족의 근본 후보:
   - perception target이 실제 KP1보다 벽 바깥/안쪽으로 어긋남
   - measured TCP grasp center가 실제 줄기 파지점보다 뒤에 있음
   - final TOOL +Z segment가 90mm 이상에서 해당 branch/pose와 잘 맞지 않음
   - collision/world 또는 joint limit 때문에 cuRobo deep endpoint가 90mm까지만 허용됨
3. Detection Y drift가 계속 큼: raw Y≈799~805mm가 wall 672mm로 clamp됨.
   Y는 clamp되지만 X/Z target도 같은 detection에서 나온 값이라 같이 의심해야 한다.
4. SW regression 테스트 — `FINAL_APPROACH_VEL/ACC`, `RETREAT_VEL/ACC`,
   `PRE_APPROACH_SETTLE_SEC`, `STRAIGHT_RETREAT_SETTLE_SEC` 변경이 SW와 공유되는데
   아직 SW로 재실행 안 함.
5. 미커밋 상태로 남아있는 파일: `config/scan_pose_candidates_depth2.yaml`
   (untracked, `use_for_automated_motion: false`, 이번 세션에서 만든 게 아니라 이전부터
   있던 파일로 보임 — 내용 검토 후 필요하면 별도로 커밋할 것. 이번 세션에서는 건드리지
   않았음).

## 6. 다음 할 일 (우선순위)

1. **target/TCP 기준 재확인**: 카메라 검출점이 실제 잡아야 하는 줄기 위치보다 얼마나
   얕은지 실측/영상 기준으로 표시한다. 단순 approach distance 튜닝만으로는 옆빗김이 생김.
2. **파지점 자체 보정 실험**: `pick_target_x_bias_m`, `pick_target_z_bias_m` 또는
   perception의 KP target 정의를 조정한다. depth 방향을 approach distance로 더 밀지 말고,
   목표점 자체를 실제 줄기 파지점으로 옮기는 쪽을 우선 검토한다.
3. **TCP/URDF 확인**: measured TCP가 실제 "줄기를 잡는 중심"인지, 아니면 파츠 끝/집게 중심과
   10~20mm 차이가 남아 있는지 재확인한다. 지금 증상은 TCP 기준이 뒤쪽이면 그대로 재현된다.
4. **cuRobo deep endpoint 한계 분석**: debug dump/replay로 왜 110/130/150/200mm endpoint가
   IK_FAIL인지 확인한다. collision sphere, joint limit, orientation 중 무엇이 한계인지 분리.
5. SW 회귀 테스트 1회 실행.
6. `config/scan_pose_candidates_depth2.yaml` untracked 파일 검토.

## 7. 다음 실행 커맨드

Planner:
```bash
ros2 run e0509_gripper_description curobo_planner_node.py --ros-args \
  -p measured_tcp_plan_only:=false \
  -p direct_curobo_final_approach_for_measured_tcp:=true \
  -p measured_tcp_max_approach_m:=0.200 \
  -p measured_tcp_tool_line_after_curobo_fallback:=true \
  -p debug_dump_plan_calls:=true
```

Scan:
```bash
ros2 launch strawberry_motion workspace_scan.launch.py \
  enable_robot_execution:=true target_cell:=root/nw \
  enable_fusion_detection:=true enable_pick_integration:=true \
  collect_then_pick:=true collect_pick_ready_cell:=root/nw/pick_ready \
  max_total_picks:=1 \
  scan_movej_vel_deg_s:=20.0 scan_movej_acc_deg_s2:=30.0 \
  overview_return_vel_deg_s:=20.0 overview_return_acc_deg_s2:=30.0
```

Trigger:
```bash
ros2 service call /strawberry/scan/start std_srvs/srv/Trigger "{}"
```

## 8. 검증 방법 (수정할 때마다)

```bash
python3 -m py_compile <변경한 파일>
git -C /home/user/doosan_ws/src/strawberry_finalproject diff --check
cd /home/user/doosan_ws && colcon build --packages-select e0509_gripper_description strawberry_motion --allow-overriding e0509_gripper_description
```
