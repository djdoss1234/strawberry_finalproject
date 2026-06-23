# NW(딸기 벽면 수확) 트러블슈팅 케이스 로그

작성 기준일: 2026-06-21. 발표/포트폴리오용으로 "어떤 상황에서 무엇을 시도했고 됐는지/안됐는지"를
시간순으로 정리한다. 코드/파라미터 상세는 각 항목의 커밋 해시로 추적 가능. SW(계란판) 작업은
`sw_single_strawberry_harvest_*.md`, `project_retrospective_portfolio_roadmap.md` 참고 — 이
문서는 NW(벽면, measured-TCP) 경로만 다룬다.

## 요약 표

| # | 상황 | 시도 | 결과 |
|---|---|---|---|
| 1 | grasp_quat 후보 중 깊이는 같은데 팔꿈치(J3) 자세가 나쁜 걸 그대로 채택 | tie-break 로직 추가 | ✅ 해결·검증 |
| 2 | `measured_tcp_max_approach_m`을 올려도 180mm 천장에서 안 늘어남 | 하드코딩 천장 0.180→0.220 상향 | ✅ 해결 |
| 3 | pick-ready pose가 TCP 자체가 특이점(J3≈0°)이라 어떤 IK branch도 못 씀 | TCP 위치를 centroid로 재계산 | ✅ 해결·검증 |
| 4 | Codex 수정 누적으로 NW high 회귀 발생 | 디버그 브랜치로 known-good 커밋 복구 | ✅ 해결(검증된 기준점 확보) |
| 5 | crane_z_offset 고정값(25mm)이라 틸트 변형에 따라 닫는 위치가 KP1보다 56mm 위 | 고정값 → "실제도달Z−목표Z" 동적 보정 | ✅ 해결·검증 |
| 6 | 동적 보정 + 수동 여유(+20mm)로 빈 그리퍼가 하강하며 딸기를 쳐서 떨어뜨림 | 수동 여유 제거(0mm)로 재시도 예정 | ⚠️ 진행중(다음 실기) |
| 7 | `wall_y_clamped`로 깊이가 항상 180mm floor-lock, 깊은 타겟에서 얕게 잡힘 | 높이 기반 게이트를 틸트 기반으로 교체 | ✅ 해결·검증 |
| 8 | RETREAT 중 J2 한도(±95°) 초과 1차 사고 — 두 다리(틸트+수평)를 하나로 합쳐 후진 | 다리를 분리해 각각 되돌리기(2단계) | ✅ 해결·검증 |
| 9 | 같은 J2 한도 초과가 **2단계 분리 이후에도** 재발(다른 타겟) | 후진 leg를 cuRobo plan으로 변경 | ✅ J2 위험 해소 / ⚠️ 새 차단사유(벽 콜리전) 발견, 보류 |
| 10 | grasp 방향이 실제 줄기 방향(`kp0_to_kp1`)을 무시하고 항상 고정 라이브러리에서만 선택됨 | (발견만) | ❌ 미수정 — blast radius 커서 보류 |
| 11 | `gripper_client.py` import 오타로 `ros2 run` 자체가 기동 실패 | import명 수정 | ✅ 해결·검증 |
| 12 | `curobo_planner_node.py`가 비대해져 구조 파악·확장이 어려움 | 정책/실행 모듈로 분리(리팩토링) | ✅ 완료(기능 무변경 확인됨) |
| 13 | 항목12 이후에도 전체 코드베이스에 394줄까지의 큰 단일 메서드 다수 잔존 | 전수 AST 실측 후 순위대로 추가 분리 | ✅ 완료(최대 248줄까지 축소, 기능 무변경 확인됨), 실기 미검증 |
| 14 | `flat_grasp_only` 동률 깊이 tie-break에 틸트 패널티가 없어 +15°까지 불필요하게 올라감(딸기 2건 빈손) | tie-break에 NW-high용 flat-safe 규칙(20° 안전선) 적용 확장 | ✅ 해결(실제 로그 재시뮬레이션으로 효과 확인), 실기 재검증 필요 |
| 15 | 같은 좌표인데 직선(MoveLine) 진입이 1차는 성공, 재시도는 즉시 거부 — 시작 관절각 차이로 추정 | TOOL_FINISH raw MoveLine 실패 시 cuRobo plan으로 같은 목표점 재시도 추가 | ✅ 구현·실기 발동 확인(`final_approach_tool_finish_curobo_retry` 로그). 단 목표가 진짜 IK_FAIL인 경우(항목14 종합분석 #2/#3과 동일 자리)는 재시도도 실패 — 이건 정상 동작(구제 불가능한 케이스를 정직하게 실패 처리) |
| 16 | `scan_dwell_sec=4s`가 일부 코너엔 부족해 인식되고도 시간초과로 빈 셀 처리 | (발견만) | ❌ 미수정 — dwell 상향 또는 셀별 오버라이드 제안 |
| 17 | 항목14 수정(모든 variant 시도) 후 0°가 이미 이겼는데도 나머지 5개 variant를 끝까지 다 탐색 — 픽 1건당 +27초 지연 | (발견만) | ❌ 미수정 — 트레이드오프(반응성 vs 더 깊이 도달 가능성), 조기 종료 조건 추가 검토 필요 |

## 상세

### 1. depth-probe tie-break 버그 (`69037f0`, 2026-06-18)
**상황**: NW measured-TCP pick이 6개 그리퍼 회전(quat) variant를 순서대로 시도하며 깊이를
probing하는데, 4개 variant가 전부 같은 90mm 깊이에서 성공했지만 팔꿈치(J3) 건강도가
variant 1→4 순서로 0°→52.5°로 전혀 달랐음. 선택 로직이 "더 깊을 때만" 교체해서 동률이면
항상 가장 먼저 시도한(J3≈0°, 특이점 근접) variant를 그대로 썼고, Doosan MoveLine이
"success인데 무동작"이던 현상의 원인이었음.
**시도**: 동률일 때 J3가 0°에서 더 먼(건강한) variant로 교체하는 tie-break 추가.
**결과**: ✅ 실패했던 실제 로그로 재시뮬레이션해 효과 확인, 이후 실기에서 `GRASP_POSE_REACHED`
2회 연속 도달(이전엔 0회).

### 2. measured_tcp_max_approach_m 하드 천장 (`aae1832`, 2026-06-18)
**상황**: 파라미터로 0.200을 줘도 코드 내부 `min(0.180, ...)`에 막혀 조용히 180mm로 깎임.
**시도**: NW-only 천장 상수를 0.220으로 상향, J3 "good enough"(45°) early-exit 추가로
불필요한 variant 탐색도 줄임.
**결과**: ✅ 해결.

### 3. pick-ready pose 자체가 특이점인 문제 (`1d9bb59`, 2026-06-18)
**상황**: `root/nw`(옛 중앙 pose)의 TCP 위치는 128-seed IK 전수조사 결과 **어떤 IK branch로도
J3가 거의 0°(완전신전)인 해밖에 없었음** — "다른 branch를 찾자"는 시도 자체가 불가능한 케이스.
**시도**: 같은 TCP가 아니라 NW 서브셀 4개의 centroid에서 새로 오프라인 IK를 계산해 TCP 위치
자체를 바꿈.
**결과**: ✅ J3=62.7° 건강한 해 발견, 서브셀 전체에서 도달 가능 검증.

### 4. NW high 회귀 (`f2ec778` 복구, 2026-06-20)
**상황**: Codex가 NW high 쪽 수정을 계속 쌓다가 "MoveLine success인데 무동작" 회귀가 재발.
**시도**: 사용자가 "디버그 브랜치 분리"를 선택, `f2ec778`(회귀 전 마지막 정상 커밋)에서
`debug/nw-return-to-depth-good` 브랜치 생성, main은 보존.
**결과**: ✅ 이 브랜치에서 실기 재현 → `GRASP_POSE_REACHED` 성공, 사용자가 "지금 딱 좋다"로
확인한 known-good 기준점 확보. (단 그리퍼는 `pos=699`로 `GRASP_UNVERIFIED` — 항목 6/9로 이어짐)

### 5. crane_z_offset 고정값 → 닫는 위치가 KP1보다 56mm 위 (`8b59772`, 2026-06-20)
**상황**: +15° 틸트로 180mm 전체 접근하면 Z가 `sin(15°)*180mm≈47mm` 같이 올라가는데, 보정은
고정 25mm뿐이었고 보조 보정(+30mm)은 tilt>10°면 스킵되는 로직이라 +15°에서 전혀 작동 안 함.
**시도**: 고정 mm 보정을 버리고 "실제 도달 Z − 목표 KP1 Z" 차이를 그대로 open descent 거리로
사용 (variant/tilt/depth 무관하게 항상 KP1 도달).
**결과**: ✅ 동적 보정 거리가 의도대로 늘어남을 로그로 확인(25→76mm).

### 6. 동적 보정 + 수동 여유 과다 → 딸기 떨어짐 (2026-06-20)
**상황**: 항목 5의 동적 보정(56mm)에 사용자 요청 여유(+20mm)를 더했더니 빈 그리퍼가 하강하며
딸기를 건드려 떨어뜨림. 사용자 육안 확인: "방향은 맞는데 길이가 과함."
**시도**: 다음 실기에서 여유분을 0으로 되돌려(`nw_high_target_descent_extra_below_kp1_m:=0.000`)
순수 동적 보정만으로 재시도 예정.
**결과**: ⚠️ 다음 실기 결과 대기 중(이 케이스 로그 작성 시점까지 미확인).

### 7. wall_y_clamped로 깊이 floor-lock (`37056bc`, 2026-06-20)
**상황**: 카메라가 측정한 Y가 거의 매 pick마다 등록된 벽(672mm)보다 안쪽으로 잡혀 클램프되고,
그 결과 `adaptive_dist`가 정확히 baseline(180mm)으로 수렴 — `measured_tcp_max_approach_m`을
올려도 적용이 안 됨. 유일한 탈출구(`nw_high_target_final_extra_m`)는 "높이≥750mm" 게이트라서
z=717mm처럼 낮은 NW high 타겟엔 적용 안 됨(이미 두 번 고친 "키 대신 틸트로 게이팅" 패턴과
같은 버그가 여기에도 있었음).
**시도**: 게이트를 높이 기반 → 틸트 기반(`abs(used_approach_dir[2]) > 1e-3`)으로 교체. 틸트
0인 SW는 동작 무변화.
**결과**: ✅ 같은 타겟에서 `NW_HIGH_TARGET_FINAL_EXTRA: 180mm -> 195mm` 정상 적용 확인.

### 8. RETREAT 중 J2 한도 초과 — 1차 (다리 합산 버그) (`ff5c083`, 2026-06-20)
**상황**: 틸트 진입(+15°) 후 수평으로 꺾여 마무리하는 경로에서, 후진(retreat)이 이 두 다리를
구분 못 하고 전체 거리를 틸트 방향 하나로만 되돌림 — 실제로 안 내려간 수평 구간까지 틸트의
Z 성분만큼 더 내려가 버려 팔이 과신전, J2가 -97.55°(한도 ±95° 초과)에서 멈춤.
**시도**: 전진 시 실행한 수평 다리의 거리/방향을 따로 기록해두고, 후진을 [수평 다리 되돌리기]
→ [남은 틸트 다리 되돌리기] 2단계로 분리.
**결과**: ✅ 같은 타겟 재현 → `RETREAT_TOOL_FINISH_UNDO`+`RETREAT_BASE` 정상 완료, J2 재발 없음,
`PICK COMPLETE`까지 정상 종료.

### 9. 같은 J2 초과가 2단계 분리 후에도 재발 — 진짜 원인은 달랐음 (2026-06-21)
**상황**: 항목 8 수정을 검증하던 중, **다른 타겟**(더 깊은 195mm, ceiling 근접)에서 J2 -97.55°가
또 발생. 처음엔 "2단계 분리 로직 자체의 결함"으로 의심했으나, 디버그 로그(`retreat_step_complete`,
매 retreat 단계마다 joint 기록)를 추가해 재현한 결과 **1단계(순수 수평 -125mm) 단독으로 이미
한도를 넘김** — 2단계는 무관했음. 더 파보니 똑같은 125mm를 전진할 땐 J2가 4°만 움직였는데
후진할 땐 15°나 움직이는 비대칭이 있었고, 원인은 전진/후진 사이에 `OPEN_STEM_DESCENT`+
`DETACH_PULL_DOWN`(합 72.6mm 하강)이 끼어있어 후진이 전진보다 72.6mm 더 낮은, 기구학적으로
훨씬 민감한(특이점 인접) 자세에서 시작한다는 것.
**시도**: 이 수평 후진 leg를 Doosan raw 상대직선(`MoveLine`, IK 브랜치를 컨트롤러가 임의로 선택)
대신 cuRobo plan(조인트 한도를 사전에 체크하고, 못 가면 깨끗하게 실패)으로 실행하도록 변경
(`d7a8cc6`).
**결과**: ✅/⚠️ 절반 성공. 같은 타겟 재현 실기에서 **J2는 더 이상 위험하지 않음**
(`RETREAT_TOOL_FINISH_UNDO` 시도 시점 J2=-92.11°, ±95° 안쪽) — "한도 넘은 채로 멈춤"이라는
위험한 실패가 "plan이 거절되어 안전하게 멈춤"으로 바뀐 건 확인됨. 단 **새로운 차단 사유 발견**:
`INVALID_START_STATE_WORLD_COLLISION` — 전진 시 raw 125mm leg가 등록된 벽(`whiteboard_wall`)
면보다 더 깊이 들어가버려서, 그 자리에서 cuRobo로 재계획하면 "현재 위치가 이미 벽과 충돌 중"이라고
판단해 시작점부터 거절함(후진 방향인데도). 항목 7의 `wall_y_clamped` 캘리브레이션 드리프트와
같은 뿌리. **사용자 결정: 이 케이스는 일단 보류**, 후보로 (a) retreat plan만 wall cuboid 잠깐
제외하고 재계획, (b) 이 사유일 때만 raw MoveLine으로 fallback(예전 J2 위험 재도입 가능성),
(c) wall cuboid 등록 위치 자체 보정(더 큰 범위) — 미착수.

### 10. grasp 방향이 실제 줄기 방향을 무시함 (발견, 2026-06-20)
**상황**: 사용자 관찰("꺾인 딸기들은 다 옆으로 접근함")을 코드로 확인한 결과, `strawberry_fusion_node.py`가
`stem_grasp_direction_mode: kp0_to_kp1`로 계산해 보내는 **실제 줄기 방향**(orientation)을
`curobo_planner_node.py`의 `_pick()`이 로그에만 쓰고 실제 그리퍼 방향 결정에는 한 번도 안 씀 —
방향은 항상 고정 wall-relative 틸트 라이브러리(주로 +15°)에서만 고름.
**시도**: (수정 안 함) — SW를 포함한 모든 measured-TCP pick 경로에 영향을 미치는 범위라 blast
radius가 크고, 신중한 설계가 필요하다고 판단해 보류.
**결과**: ❌ 미해결. 빈손/측면접근 다발의 가장 유력한 후보 원인으로 남아있음.

### 11. gripper_client.py import 오타 (`f44572f`, 2026-06-21)
**상황**: 이전 분리 작업(Codex)에서 `gripper_client.py`가 존재하지 않는 `GRASP_CLOSE_SETTLE_SEC`를
import(정답은 `GRIPPER_CLOSE_SETTLE_SEC`) — `py_compile`은 import를 실제로 실행하지 않아서
빌드 통과 후에도 한 번도 못 잡혔고, `ros2 run` 시점에 `ImportError`로 즉시 죽음.
**시도**: import명 수정 + 전체 모듈을 실제로 import해서 체이닝 검증(이후 검증 절차에 추가).
**결과**: ✅ 해결·검증.

### 12. 구조 리팩토링 (2026-06-21)
**상황**: `curobo_planner_node.py`가 2,800줄 넘게 비대해져 신규 기능 추가/디버깅이 점점
어려워짐.
**시도**: 순수 계산/판단 로직은 `*_policy.py`로, 로봇/ROS 의존 실행 로직은 `*_client.py`/
`*_executor.py`로 분리(`grasp_search_executor.py` 등). 동작/파라미터/로그 이벤트명은 전부
무변경 원칙으로 진행.
**결과**: ✅ 51개 이상 커밋으로 분리 완료, py_compile/실제 import/colcon build로 매 단계
검증. 항목 9의 cuRobo retreat 수정이 가능했던 것도 이 정리 덕분(retreat 실행 지점이 한
메서드로 모여있어 빠르게 찾고 고칠 수 있었음). 마지막으로 남아있던 큰 단일 블록인 place
실행 로직(408줄, row2 분기 포함)도 `tray_place_executor.py`로 분리(`edb67de`) —
`curobo_planner_node.py`가 2,461→2,069 lines. row2 known 이슈는 고치지 않고 그대로 이동만.
사용자가 "그래도 `__init__`이 왜 아직 큰지" 재확인 요청 → 다시 까보니 `__init__` 안에
실제로 분리 가능한 두 블록이 있었음(앞선 "이득 없음" 판단이 부정확했음, 인정): cuRobo
MotionGen 부트스트랩(~50줄, config 로딩+warmup)과 ~40개 파라미터 declare/read 블록(~136줄).
둘 다 `planner_bootstrap.py`로 분리(`1836c92`) — 이동 전후 텍스트를 `self.`→`node.` 치환 후
diff로 0줄 차이 확인(완전 동치 검증). `curobo_planner_node.py` 2,069→**1,898 lines**.
남은 `__init__`(~280줄)은 ROS subscription/client/executor 생성 wiring이라 `self` 바인딩이
꼭 필요해서 더 분리 안 함.

**fusion/scanner 노드 전체 점검**: `strawberry_fusion_node.py`(1,050줄, `_loop()` 372줄)와
`strawberry_yolo_node.py`(769줄, `loop()` 325줄)는 이번 리팩토링 작업 전까지 분리된 모듈이
하나도 없었음(완전 monolithic) — 단, `strawberry_yolo_node.py`는 실제로는 **죽은 코드**임이
확인됨(현재 launch 경로 `workspace_scan.launch.py`는 `strawberry_fusion_node.py`만 띄움,
yolo_node는 미니프로젝트 시절 Grounding DINO 데모 launch에만 남아있음) — 우선순위 낮음.
`scan_executor_node.py`(strawberry_motion 패키지, 1,225줄)는 실제 활성 경로 확인됨 →
`_scan_sequence()`(245줄, 스캔 전체 상태머신)를 `_compute_scan_order`/`_scan_one_cell`/
`_finish_collect_then_pick`/`_finish_scan_sequence` 4개로 내부 메서드 분리(`b686cbd`,
파일 신규 생성 없이 같은 파일 안에서 분리 — 이 패키지엔 아직 policy/executor 모듈 컨벤션이
없어서). 가장 큰 메서드가 245→127줄로 줄어듦. py_compile/colcon build/install space 확인
통과, **실기 미검증**. `strawberry_fusion_node.py`는 grasp orientation 무시 버그(항목 10)와
`stem_keypoint_depth_invalid` 인식률 이슈가 현재 활성 상태로 들어있는 파일이지만, 사용자가
명시적으로 진행 승인("fusion node도 진행해 최적화 하고 쪼개") → `_loop()`(372줄, 매 프레임
호출되는 검출 파이프라인 전체)을 프레임 캡처/가드, YOLO 추론 캐싱, seg overlay 그리기,
scene position publish, per-pose 융합/필터링/grasp 계산/트래킹의 5단계로 분리
(`_capture_frame_and_guards`/`_run_or_reuse_inference`/`_draw_seg_overlays`/
`_publish_scene_positions`/`_process_pose_detection`, `c65f29a`). 가장 큰 블록
(`_process_pose_detection`, 241줄)을 포함해 5개 블록 전부 원본↔변환본 역치환 diff로 0줄
차이 확인 후 적용. `continue`/bare `return`은 새 메서드의 `return None`으로, 루프 안의
`if stable: ripe_candidates.append({...})`는 `candidate = None; if stable: candidate = {...}`
+ `return candidate`로, 호출부는 `if candidate is not None: ripe_candidates.append(candidate)`로
변환(동작 동일 — append가 일어나는 조건이 정확히 일치). seg overlay에서 쓰는 `cls_color` 딕셔너리는
per-pose 필터에서도 재사용되므로 파라미터로 전달. py_compile/실제 import/git diff --check/
colcon build/install space 확인 전부 통과. **버그(항목 10, `stem_keypoint_depth_invalid`)는
그대로 옮기기만 했고 고치지 않음, 실기 미검증.**

### 13. 전체 코드베이스 "남은 큰 메서드" 전수 분리 (2026-06-21)

**상황**: 사용자가 "전부 다 진행해. 문제 있으면 보고하고 없으면 진행해 전부" — 항목12 이후에도
남아있던 큰 단일 메서드 전체를 AST로 실측해서 순위대로 처리.

**시도/결과**:
- `_pick`(394줄, `curobo_planner_node.py`) — 이번 세션 최대 단일 메서드. `_prepare_pick_target_or_abort`
  (target 준비+x/z 가드, 실패시 None) / `_search_grasp`(quat variant 탐색 루프, 순수 이동)로 분리.
  394→**248줄** (`d4f1dce`).
- `tray_place_executor.py`의 두 메서드 — `execute_marker_place_after_retreat`(183줄)에서
  clearance/orientation 후보 탐색 루프를 `_search_marker_place_above`로 분리(183→**130줄**).
  `execute_taught_slot0_place_reference_after_retreat`(225줄)에서 슬롯 검증+위치 계산을
  `_compute_taught_slot_above_target`으로 분리(225→**194줄**) (`17cb269`).
- `_scan_one_cell`(126줄, `scan_executor_node.py`) — 이동+도착대기(`_move_to_scan_cell_and_wait`)와
  검출처리(`_process_cell_detections`)로 분리. 126→**42줄** (`8970a55`).
- `__init__` 3종 추가 분리 — `strawberry_fusion_node.py`(189줄)는 `planner_bootstrap.py`와 동일
  패턴으로 `fusion_bootstrap.py` 신규 생성, 파라미터 declare/load 80줄 이동(189→**109줄**,
  `10f0852`, `_as_bool`도 단일 사용처라 같이 이동). `scan_executor_node.py`(165줄)는 이 패키지에
  아직 모듈 분리 컨벤션이 없어 in-class 메서드(`_declare_and_load_params`)로 60줄 이동
  (165→**105줄**, `56ab209`). `curobo_planner_node.py`(278줄)는 객체 생성/wiring 부분(180줄,
  `self` 의존 심해 분리 보류 재확인)은 그대로 두고, 순수 로깅 100줄만 `_log_startup_banner`로
  분리(278→**180줄**, `9ed5554`).

**최종 상태(AST 실측)**: 전체 코드베이스에서 가장 큰 단일 메서드가 248줄(`_pick`)까지 줄어듦.
남은 100줄 이상 메서드(`_process_pose_detection` 246줄, `execute_taught_slot0_place_reference_after_retreat`
194줄, curobo `__init__` 180줄, `declare_and_load_params` 140줄, `execute_marker_place_after_retreat`
130줄)는 공통적으로 "독립적인 실패 조건이 없는 단일 개념의 순차 파이프라인이거나 순수
선언/로딩 나열"이라 더 쪼개면 변수만 여러 메서드에 분산되고 가독성은 오히려 떨어진다고 판단,
의도적으로 보류. 모든 분리는 원본↔변환본 역치환 diff로 0줄 차이 확인 + py_compile/실제
import/git diff --check/colcon build/install space 확인 전부 통과. **실기 미검증.**

### 14. `flat_grasp_only` tie-break 틸트 패널티 부재 (`<uncommitted>`, 2026-06-21)
**상황**: 항목 7 수정(wall_y_clamp 캡 스킵) 직후 첫 실기 5회차 중, 동일 위치를 두 번 시도한
케이스(아래 #2/#3)에서 그리퍼가 매번 +15°까지 기울어져 KP1보다 32.6mm 위를 잡으려다 빈손.
원인: 동률 깊이(depth tie)에서 "더 건강한 J3만 보면 무조건 교체"하는 규칙에 틸트 크기 패널티가
없어서, 0°(J3=18°, 불안전)→5°(J3=37.1°, 안전)로 교체된 뒤 그 자리서 멈춰야 하는데
15°(J3=57.3°, 동률인데도 더 건강)까지 계속 올라갔음.
**시도**: 항목 1(NW-high 전용)에 있던 "동률이면 J3≥20°로 이미 안전한 후보가 있으면 틸트가 더
작은 쪽을 유지" 규칙을 `flat_grasp_only`에도 적용(`grasp_candidate_policy.py`의
`should_replace_measured_best`에 `flat_grasp_only` 파라미터 추가, `is_nw_high_target` 조건에
`or` 결합). NW-high/SW 경로는 조건이 그대로라 무변화.
**결과**: ✅ 실제 실패 로그의 기록값(0°=18°, 5°=37.1°, 15°=57.3°, 전부 70mm 동률)을 수정된
함수에 그대로 재실행해 5°가 유지되는 것 확인(이전엔 15°로 교체됨). **실기 재검증 필요** —
아래 6회차 종합 분석의 #6에서 "0°가 정상 선택됨"까지는 확인했으나, 이 케이스(#2/#3) 자체를
재현해 실제로 5°가 나오는지는 다음 실기에서 확인해야 함.

### NW flat 모드 첫 실기 6회 시도 종합 분석 (2026-06-21)

항목 7(wall_y_clamp 캡 스킵, `3fb345f`)과 항목 14 수정을 실기에 처음 투입한 세션. 같은
`root/nw_flat` 셀에서 딸기 5개(그중 1개는 재시도) 총 6회 pick을 시도, 왼쪽부터 순서:
역Y자딸기①(#2) · 역Y자딸기②(#3) · 장줄기딸기(#4) · 처음성공딸기(#1, 재시도 #6) · 우측맨위딸기(#5).

| # | 로그(curobo_planner_node) | 대상 | 좌표 raw(x,y,z mm) | 선택 variant(틸트) | 도달 깊이 | KP1 위 overshoot | 결과 | 특이사항 |
|---|---|---|---|---|---|---|---|---|
| 1 | `210326-057eddb0` | 처음성공딸기(우측위) | (-91,796,829) | +5° | 90mm(직선 1회 179mm 직접 성공) | 32.6mm(동적보정으로 닫기 전 상쇄) | GRASP_UNVERIFIED(pos699) — 사용자 육안 확인 성공 | retreat `RETREAT_BASE` 충돌 거부→hold latched (항목9와 동일 원인, 사용자: "문제없었음") |
| 2 | `210700-b50d4bfa` | 역Y자딸기① | (-329,801,785) | +15°(항목14 버그, 수정 전) | 70mm | 32.6mm(상쇄됨) | GRASP_EMPTY(pos700) | "대각선 위로 가서 한참 위 파지 시도" — 동률인데 불필요하게 끝까지 기울어짐 |
| 3 | `211129-0aa6c077` | 역Y자딸기② | (-337,806,789) | +15°(#2와 동일 버그) | 70mm | 32.6mm(상쇄됨) | GRASP_UNVERIFIED(pos699) | #2와 좌표·variant·결과 패턴 거의 동일(같은 버그 재현) |
| 4 | `211600-4fff0c04` | 장줄기딸기 | (-247,785,781) | +15°(이번엔 진짜 70→90mm 추가 도달, tie 아님) | 90mm | 37.8mm(상쇄됨) | GRASP_EMPTY(log) / 사용자 관찰상 "끝에서 매가리없이" 약하게 파지 | 줄기가 길어 오차를 흡수한 것으로 추정 — 로그(EMPTY)와 실제 관찰이 불일치하는 약한 파지 |
| 5 | `212112-9b60de12` | 우측맨위딸기 | (+401,755,762 추정, fusion 로그 기준) | — (접근 자체 안 함) | — | — | 접근 실패(pick_sequence_start 없음) | `scan_dwell_sec=4.0s` 만료 후 "SCANNED_EMPTY"; 같은 윈도우 fusion 로그엔 5.8초 늦게 고신뢰(kp_conf 0.92) 안정 타겟이 찍혀 있음 — dwell이 짧아서 놓침 |
| 6 | `212302-087530ff` | 처음성공딸기 재시도 | (-80,793,829) | 0°(정상 — 6개 variant 전부 90mm 동률, 가장 작은 틸트 유지) | cuRobo로 90mm, 남은 90mm는 raw MoveLine 시도 | — | ABORT(그리퍼 시도 자체 없음) | `FINAL_APPROACH_STRAIGHT_BASE`(180mm 직선)와 `FINAL_APPROACH_TOOL_FINISH`(남은 90mm)가 둘 다 0.15초 만에 즉시 거부(success:false) — #1에서는 거의 동일 좌표로 같은 180mm 직선이 7.9초 걸려 그대로 성공했었음 |

**원인 분석**:
- **항목14 버그(#2, #3)는 틸트 자체보다 "불필요한 틸트"가 문제였다.** `OPEN_STEM_DESCENT`가
  도달 Z와 KP1 Z 차이를 그대로 보정해 닫기 직전엔 항상 KP1 높이로 내려가므로(표의 overshoot
  값 = executed_descent 값과 일치), 순수 "높이를 못 맞췄다"는 설명은 부정확함. 실제로는 틸트가
  클수록 진입 경로가 비스듬해져 **집게가 줄기와 정렬되지 않은 채로 스쳐 지나가는 효과**가
  커지는 것으로 보임 — #2/#3은 깊이가 0°와 동일(70mm)했는데도 불필요하게 15°까지 기울어졌다가
  빈손이었고, #1/#4는 그 틸트가 실제로 더 깊이(90mm) 들어가기 위해 필요했던 경우라 결과가
  나았음(완전성공 또는 약한 파지).
- **진짜 공통 변수는 "도달 깊이"로 보인다.** 70mm 도달 케이스(#2,#3)는 둘 다 실패/모호,
  90mm 도달 케이스(#1,#4,#6)는 성공/약한파지/(실행실패로 미시도). `wall_y_clamped`로 인해
  목표는 항상 180mm(uncapped)인데 이 구간 전체에서 어떤 orientation도 70~110mm 이상은
  cuRobo IK_FAIL — 즉 "실제 필요한 접근 거리(180mm)"와 "이 구간에서 IK로 실제 도달 가능한
  거리(~70~110mm)" 사이에 70mm 이상의 구조적 간극이 있고, 이 간극을 메우는 마지막 수단이
  raw `MoveLine`(TOOL_FINISH) 하나뿐인데 그게 신뢰할 수 없음(#6).
- **#6의 raw MoveLine 실패는 같은 좌표에서도 재현성이 없다.** #1은 거의 동일 좌표에서 180mm
  직선이 7.9초 걸려 정상 성공했는데, #6은 0.15초 만에 즉시 거부됨. 둘의 차이는 그 순간의
  시작 관절각(직전 동작에서 남은 팔꿈치/손목 자세)으로 추정 — 직선 경로가 특이점을 지나는지
  여부가 시작 자세에 따라 달라지는 것으로 보이나 확정은 아님.
- **#5는 perception 타이밍 문제로, 항목14/오늘 수정과 무관.**

**해결책(제안, 우선순위순)**:
1. **(완료, 실기 재검증 필요)** 항목14 tie-break 수정 — #2/#3류 "불필요한 틸트로 빈손"은 이미
   고쳤음. 다음 실기에서 이 정확한 좌표(역Y자 셀)로 재현해 0°/5° 근처에서 멈추는지 확인.
2. **(구현 완료, 실기 미검증)** `FINAL_APPROACH_TOOL_FINISH`의 raw MoveLine이 실패하면 그대로
   포기하는 대신, cuRobo로 같은 최종 목표점까지 joint-space plan을 한 번 더 시도(항목15).
   `final_approach_executor.py`의 `execute_tool_finish` 한 곳만 변경 — raw MoveLine이 성공하는
   기존 모든 경로(SW 포함)는 새 코드를 거치지 않아 무변화. #6류("직선진입 실패로 그리퍼
   시도조차 못함") 방지가 목표.
3. **(제안, 미구현, 다른 패키지)** `scan_dwell_sec`(현재 4.0s, `scan_executor_node.py`)을
   늘리거나 `root/nw_flat` 전용으로 늘려서 #5류(인식은 됐지만 늦어서 놓침) 방지. 전체 셀
   사이클이 느려지는 트레이드오프 있음.
4. **(이미 알려진 미해결, 항목9)** `RETREAT_BASE`가 매 pick마다(6회 중 적어도 4회) 벽
   콜리전으로 거부되어 hold latched — 사용자가 "문제없다"고 했지만 100% 재현되는 패턴이라
   계속 추적 필요.
5. **(장기, 더 근본적)** 이 구간(NW flat 셀 전체) 자체가 wall_y_clamp 탓에 "요구 깊이
   180mm vs 실제 IK 가능 깊이 ~70~110mm" 간극을 안고 있음 — `wall_y_clamped`의 진짜 원인
   (카메라가 등록된 벽보다 안쪽으로 보는 현상)을 찾아 캡 자체를 없애는 게 가장 근본적이지만,
   항목7 작성 시점에 이미 한 차례 조사하고 보류한 사안.

### 7번째 시도 — 처음성공 자리(x≈-60mm) 재현, 완전 실패 + 새 지연 문제 발견 (2026-06-22, `curobo_planner_node_20260622T103746`)

같은 세션에서 마지막으로 시도한 7번째 pick. `=== PICK ===`(x=-60, raw_y=798→672mm clamp,
z=828mm)부터 그리퍼 reset까지 stdout 원본으로 분석.

**결과**: 0° variant가 정상 선택(J3=36°, align=0° — 항목14대로 동작)됐지만 최대 도달 깊이가
90mm뿐이었고, `FINAL_APPROACH_STRAIGHT_BASE`(180mm 직선)·`FINAL_APPROACH_TOOL_FINISH`(남은
90mm)·항목15의 cuRobo 재시도까지 전부 IK_FAIL/MoveLine 거부로 실패 — **그리퍼를 한 번도 못
닫아보고 ABORT**. 이 좌표는 첫 시도(x≈-91mm, 거의 같은 자리)에서는 성공(GRASP_UNVERIFIED)했던
곳인데, 이번엔 완전히 막힘 — 항목③(`wall_y_clamped`로 인한 180mm 요구 vs ~90mm 실제 한계)가
"성공했던 자리조차" 재현성이 없을 만큼 경계선에 있다는 걸 보여줌.

**새로 발견된 문제(항목17)**: PICK 시작(`282.7s`)부터 grasp search 완료(`316.5s`)까지
**33.8초** 걸렸는데, 그중 0° variant 자체의 depth probe는 약 7초(`289.7s`까지)면 끝났고
**나머지 26.8초는 이미 이긴 0°를 두고도 5/-5/10/-10/15° variant를 전부 끝까지 탐색하는 데
소모**됨(`MEASURED_TCP_PROBE_PRUNED` 반복, 중간에 `J1 out of range`/`J4 swing > 90deg`
reject도 섞여 있음). 항목14가 "0-only 강제 필터"를 없애고 모든 variant를 매번 시도하게
바꾼 부작용 — 더 깊이 도달할 가능성을 안 버리려면 다른 variant도 찔러봐야 하는데, 그 비용이
픽 1건당 +27초 수준으로 큼. 아직 안 고침(트레이드오프라 우선순위 판단 필요).

**오늘(2026-06-21~22) NW flat 세션 종합 판정**: 실패. 7회 시도 중 확정 성공 0건
(GRASP_UNVERIFIED 2건은 사용자 육안으로만 일부 확인, 로그만으론 빈손과 구분 불가),
GRASP_EMPTY 2건, 완전 ABORT(그리퍼 시도조차 못함) 2건, 접근 자체 실패 1건. 코드 수정
2건(항목14·15)은 의도대로 정확히 동작하는 것까지 확인됐으나, 근본 원인(항목5,
`wall_y_clamped`로 인한 깊이 간극)이 안 풀려서 체감 성공률 개선은 없음. 다음 작업은
이 근본 원인을 파는 것이 우선순위가 가장 높음.

## 다음에 확인할 것

- 항목 14 — ✅ 확인됨. 역Y자 왼쪽 셀(x≈-329mm) 재시도(`curobo_planner_node_20260621T221349`)에서
  0°가 정상 선택됨(불필요한 +15° 없음).
- 항목 15 — ✅ 실기 첫 발동 확인(같은 로그, `final_approach_tool_finish_curobo_retry`). 단 이번
  케이스는 재시도도 IK_FAIL로 실패 — TOOL_FINISH 110mm 자체가 cuRobo로도 못 풀리는, 진짜 도달
  불가 영역이었음(=항목5 "180mm 요구 vs ~70mm 실제 한계" 간극의 또 다른 사례). 안전망은
  의도대로 동작(직선 경로 문제 한정 구제, IK_FAIL은 구제 불가가 정상).
- 항목 16(`scan_dwell_sec` 4초 부족) — 상향 여부/범위(전체 vs `root/nw_flat` 전용) 결정.
- 항목 17(variant 전체 탐색 +27초 지연) — 조기 종료 조건 추가 여부/방식 결정.
- **최우선**: 항목5(`wall_y_clamped` 근본 원인) — 7회 시도 전부가 결국 이 간극 때문에
  막혔거나 간신히 통과함. 다음 세션은 여기부터 시작.
- 항목 9(벽 콜리전으로 retreat plan 거절) — wall cuboid 제외 재계획 / fallback / 벽 등록위치
  보정 중 방향 결정 후 적용.
- 항목 6(open descent 여유 0mm) 실기 결과.
- `scan_executor_node.py` 분리(`b686cbd`) 실기 검증 — 다음 스캔 실행에서 전과 동일하게
  동작하는지 확인.
- `strawberry_fusion_node.py` `_loop()` 분리(`c65f29a`) 실기 검증 — 다음 스캔/검출 실행에서
  전과 동일하게 동작하는지(특히 인식률·HUD 표시) 확인.
- 항목 10(grasp 방향 무시) — blast radius 검토 후 수정 여부 결정.
