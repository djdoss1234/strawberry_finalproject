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

## 다음에 확인할 것

- 항목 9(벽 콜리전으로 retreat plan 거절) — wall cuboid 제외 재계획 / fallback / 벽 등록위치
  보정 중 방향 결정 후 적용.
- 항목 6(open descent 여유 0mm) 실기 결과.
- `scan_executor_node.py` 분리(`b686cbd`) 실기 검증 — 다음 스캔 실행에서 전과 동일하게
  동작하는지 확인.
- `strawberry_fusion_node.py` `_loop()` 분리(`c65f29a`) 실기 검증 — 다음 스캔/검출 실행에서
  전과 동일하게 동작하는지(특히 인식률·HUD 표시) 확인.
- 항목 10(grasp 방향 무시) — blast radius 검토 후 수정 여부 결정.
