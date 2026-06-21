# Codex 인계서 — 리팩토링 완료 현황 + 다음 작업 (2026-06-21)

이 문서는 `HANDOFF_20260621_REFACTOR_FOR_CLAUDE_CODE.md`(Codex가 작성, 같은 날 오전)에서
시작된 Claude Code 세션의 결과를 정리한 것이다. 그 문서의 5.1~5.4 전부, 그리고 거기 없던
추가 작업까지 완료했다. 이 문서를 먼저 읽고 이어갈 것.

## 0. 반드시 먼저 확인할 것

공식 repo:

```bash
cd /home/user/doosan_ws/src/strawberry_finalproject
git status --short --branch
```

현재 브랜치: `debug/nw-return-to-depth-good` (main은 안 건드림, 머지 여부 미결정 상태 유지)

절대 건드리지 말 것:

```text
scripts/측정.py
```

현재 의도적으로 남아있는 untracked 파일 (커밋/삭제/restore 금지):

```text
config/scan_pose_candidates_depth2.yaml
ros_packages/e0509_gripper_description/config/camera_calibration_eye_in_hand.yaml
```

작업 패키지 경로 (symlink 주의 — `~/doosan_ws/src/e0509_gripper_description`은 아래 경로의 symlink):

```text
/home/user/doosan_ws/src/strawberry_finalproject/ros_packages/e0509_gripper_description
/home/user/doosan_ws/src/strawberry_finalproject/src/strawberry_motion  (scan_executor_node 등)
```

커밋은 항상 `/home/user/doosan_ws/src/strawberry_finalproject`에서만.

## 1. 이번 세션 한 줄 요약

> 원래 핸드오프의 5.1~5.4(final approach fallback, final approach 전체 helper화, grasp
> search executor, place executor)를 전부 완료. 이어서 사용자가 "왜 아직도 2000줄이냐"고
> 재차 확인 요청 → 코드베이스 전체를 AST로 실측해서 남은 큰 메서드들(`_pick` 394줄 등)을
> 추가로 분리. 동시에 J2 retreat 한도초과 사고를 재현·수정(부분 해결, 새 차단사유 발견).
> 그 사이 발견한 실제 import 버그(`gripper_client.py`)도 수정. 마지막으로 모듈 인터페이스
> 문서 갱신 + 시뮬팀 인계용 신규 문서 작성. **이번 세션에서 코드를 만진 것은 전부
> "순수 코드 이동, 로직/파라미터/로그 이벤트명 무변경"이고, 단 하나(J2 retreat 수정)만
> 실제 동작이 바뀐 수정이며 이것만 부분적으로 실기 검증됨. 나머지 전부 실기 미검증.**

## 2. 완료된 작업 전체 (커밋 순서대로)

### 2-A. 원래 핸드오프 5.1~5.3 (오전, Codex 요청사항)

```text
9f03bff refactor: extract final approach fallback depth search loop
b81f83e refactor: extract final approach orchestration into _execute_final_approach
c28057b refactor: extract measured TCP depth probe loop from grasp search
ddbd810 refactor: extract legacy grasp offset loop from grasp search
5a42fec refactor: move grasp search loop into GraspSearchExecutor module
```

- `_try_final_approach_fallback(...)`, `_execute_final_approach(...)` — `_pick()` 안 final
  approach 전체 시퀀스(precomputed cuRobo → 직선 → fallback depth 탐색 → 실패시 abort)를
  헬퍼로 분리.
- `scripts/grasp_search_executor.py` 신규 — measured-TCP depth probe 루프 + legacy grasp
  offset 루프를 `GraspSearchExecutor` 클래스로 이동(`HarvestGripperClient`와 동일한
  node-dependent client 패턴, 생성자가 `plan_fn=self.plan` 등 콜러블을 받음).
  **이때 CMakeLists.txt에 새 모듈 등록을 빠뜨렸다가 `ros2 run` 시점 `ImportError`로 발견**
  — 이후 모든 신규 모듈은 추가 즉시 CMakeLists 등록 + install space `ls` 확인을 표준
  절차로 추가함.

### 2-B. 실제 버그 수정 (코드 이동이 아닌 진짜 수정)

```text
f44572f fix: correct GRIPPER_CLOSE_SETTLE_SEC import name in gripper_client.py
```

`gripper_client.py`가 존재하지 않는 `GRASP_CLOSE_SETTLE_SEC`를 import하고 있었음(정답은
`GRIPPER_CLOSE_SETTLE_SEC`) — `py_compile`은 import를 실제 실행하지 않아서 이전까지 한
번도 못 잡혔던 버그. **이후 모든 검증에 `python3 -c "import <module>"` 실제 import를
표준 절차로 추가**(아래 7절 검증 명령에 반영).

```text
429db22 debug: log joints after each retreat step
d7a8cc6 fix: route measured-TCP retreat base-frame legs through cuRobo plan
```

**J2 관절한도(±95°) 초과 retreat 사고** — `retreat_step_complete` 디버그 로그를 추가해
재현, 원인이 "2단계 분리 로직 자체의 버그"가 아니라 "OPEN_STEM_DESCENT+DETACH_PULL_DOWN로
72.6mm 내려간 자세에서는 동일 수평거리 후진이 전진보다 3~4배 더 큰 J2 회전을 요구"하는
기구학적 민감도임을 확인. 수정: retreat의 "base" frame leg를 raw `execute_base_relative_line`
(MoveLine, 관절한도 인식 없음) 대신 `self.plan()+execute_spline()`(cuRobo, 관절한도 사전
검증)으로 실행하도록 변경. **실기 검증 결과**: J2 위험(한도초과)은 해소됨(-92.11°로 안전
마진 확보) — 단, cuRobo가 현재 위치를 `whiteboard_wall`과 콜리전으로 판단해 retreat plan
자체를 거절하는 새 차단사유(`INVALID_START_STATE_WORLD_COLLISION`)가 발견됨. 이건
기존에 알려진 `wall_y_clamped` 캘리브레이션 드리프트와 같은 뿌리 문제 — **사용자가 명시적으로
보류 결정**, 미해결로 남음(아래 4절 참고).

### 2-C. place executor 분리 (원래 핸드오프 5.4)

```text
edb67de refactor: move tray place execution into tray_place_executor.py
17cb269 refactor: split tray_place_executor's two large methods further
```

`scripts/tray_place_executor.py` 신규 — `execute_marker_place_after_retreat`(183줄) +
`execute_taught_slot0_place_reference_after_retreat`(408→225줄, row2 분기 포함)를
`curobo_planner_node.py`에서 이동. row2 known 이슈는 그대로 이동(고치지 않음). 이후
추가로 내부 분리: marker place의 clearance/orientation 탐색 루프 → `_search_marker_place_above`
(183→130줄), taught-slot0의 슬롯검증+위치계산 → `_compute_taught_slot_above_target`
(225→194줄).

### 2-D. "전부 다 진행해" — 전체 코드베이스 AST 실측 후 추가 분리

사용자가 "curobo planner가 왜 아직 2000줄이냐"고 재차 확인 → AST로 전체 메서드 줄수
실측, 큰 것부터 순서대로 분리:

```text
1836c92 refactor: move __init__ boilerplate into planner_bootstrap.py
b686cbd refactor: split scan_executor_node._scan_sequence into named phases
c65f29a refactor: split strawberry_fusion_node._loop into named phases
d4f1dce refactor: split _pick into target-prep and grasp-search helpers
8970a55 refactor: split _scan_one_cell into move and detection-processing phases
10f0852 refactor: move fusion node param declare/load into fusion_bootstrap.py
56ab209 refactor: split param declare/load out of scan_executor_node.__init__
9ed5554 refactor: extract startup log banner out of curobo_planner_node.__init__
```

| 대상 | 분리 전 | 분리 후 |
| --- | --- | --- |
| `_pick`(curobo_planner_node.py) | 394줄 | **248줄** |
| `curobo_planner_node.__init__` | 278줄 | 180줄 (`planner_bootstrap.py`+`_log_startup_banner`) |
| `strawberry_fusion_node._loop` | 372줄 | 5단계 분리(가장 큰 블록 241줄) |
| `strawberry_fusion_node.__init__` | 189줄 | 109줄 (`fusion_bootstrap.py` 신규) |
| `scan_executor_node._scan_sequence` | 245줄 | 4단계 분리(127줄) |
| `scan_executor_node._scan_one_cell` | 126줄 | 42줄 |
| `scan_executor_node.__init__` | 165줄 | 105줄 (in-class, 이 패키지는 모듈분리 컨벤션 없음) |

**최종 상태**: 전체 코드베이스 최대 단일 메서드 248줄(`_pick`). 남은 100줄대 메서드들
(`_process_pose_detection` 246줄, `execute_taught_slot0_place_reference_after_retreat`
194줄, curobo `__init__` 180줄 등)은 "독립 실패조건 없는 단일 파이프라인이거나 순수
선언/로딩 나열"이라 더 쪼개면 변수만 분산되고 가독성이 떨어진다고 판단해 **의도적으로
분리 중단**. 추가로 쪼갤 명확한 이유(여러 entry point에서 재사용 등)가 생기기 전까지는
이 상태를 유지할 것.

**검증 방법**: 모든 추출은 원본 블록과 변환 후 블록을 역치환해서 byte-for-byte diff로
0줄 차이를 확인한 뒤에만 실제 파일에 적용했다(이 방법론을 `feedback_debug_before_fix`
원칙과 함께 계속 쓸 것 — 추측성 수정 절대 금지, 기계적 이동은 역치환 diff로 수학적 증명).

### 2-E. 문서 작업

```text
683f863, 2624818, e3404ae, 8becee1, 4e1f3d4, bdbc8e2, 970379a, d4c7780, f102691, 09e689f, 957303a
00a6356 docs: add sim-team handoff doc
c56aa3b docs: add concrete verified launch/run parameter examples to sim handoff doc
```

- `NW_TROUBLESHOOTING_CASE_LOG_20260621.md` 신규 — 발표/포트폴리오용 "상황→시도→결과"
  13개 항목 + 요약표. **팀 PPT 자료 제작에 그대로 재사용 중** — 앞으로 NW 관련 사고/수정이
  생기면 (해결 여부 무관) 반드시 이 문서에 항목 추가할 것.
- `RUNTIME_MODULE_INTERFACE_SPEC_20260620.md` 갱신 — 이번 세션 분리분 전체 반영,
  `scan_executor_node.py`의 Subscriptions/Publishers/Service Clients 표 신규 추가(이전엔
  없었음).
- `SIM_TEAM_HANDOFF_NODE_AND_CONFIG_OVERVIEW_20260621.md` 신규 — 시뮬레이션 담당 팀원
  인계용. 활성 노드 전체 목록, 토픽/서비스/액션 그래프, 하드웨어 종속 부분 6가지(카메라가
  pyrealsense2 SDK 직접호출이라 토픽이 아니라는 점이 가장 중요), 핵심 설정 파일, 검증된
  실행 커맨드 예시(`ros2 run`/`ros2 launch` 파라미터 전부 설명 포함).

## 3. 미해결 이슈 (이 세션에서 손 안 댐, 우선순위순)

1. **wall-collision retreat 차단** (`INVALID_START_STATE_WORLD_COLLISION`) — J2 수정의
   부작용으로 새로 드러남. 후보 3가지 중 미결정: (a) retreat plan 시 wall cuboid 잠깐
   제외, (b) 이 실패 사유일 때만 raw MoveLine fallback(J2 위험 재도입 가능성), (c) wall
   cuboid 등록 위치 자체 보정(`wall_y_clamped`와 같은 뿌리, 범위 큼). **사용자 명시적
   보류 결정** — 재개 시 먼저 이 셋 중 방향을 사용자와 확인할 것.
2. **grasp orientation이 실제 줄기 방향을 무시함** (사용자가 "꺾인 딸기는 다 옆으로
   접근한다"고 관찰, 코드로 확인됨) — `_pick()`이 fusion node가 계산한 `msg.pose.orientation`
   (`stem_grasp_direction_mode: kp0_to_kp1` 기반 실제 줄기 방향)을 로그에만 쓰고 실제
   그리퍼 방향은 항상 `WALL_QUAT_WXYZ` + 고정 variant 라이브러리에서만 고름. **수정 안 함**
   — SW 포함 전체 영향권이라 blast radius 큼, 신중한 설계 필요.
3. **NW 인식률 낮음** — `strawberry_fusion_node.py`의 `stem_keypoint_depth_invalid` 다발
   (지난 배치 219회). fusion node 쪽 문제이고 이번 세션엔 로직을 안 건드림(구조만 분리).
4. **row2(매 3번째 슬롯) place 정확도** — known 이슈, 그대로 이동만 했고 안 고침.
5. **이번 세션 리팩토링 전체 실기 미검증** — `_pick` 분리, fusion `_loop` 분리, scan
   executor 분리, 3개 `__init__` 분리, place executor 추가분리 **전부** py_compile/실제
   import/colcon build만 통과했고 로봇으로 실제 pick/scan을 돌려본 적은 없음. 다음
   실기에서 평소처럼 한 사이클 돌려서 전과 동일하게 동작하는지(특히 인식률, retreat,
   place) 먼저 확인할 것 — 새 버그가 있다면 이 라운드의 어떤 분리 때문인지 좁히기 쉽도록
   한 파일씩 순서대로 확인 권장.

## 4. 다음 작업 추천 순서

1. (최우선, 안전) 이번 세션 분리분 실기 검증 — 저속/단일 target으로 SW 먼저, 이상 없으면
   NW. 회귀 있으면 어느 커밋인지 `git bisect` 가능(전부 작은 단위 커밋이라 좁히기 쉬움).
2. 1번 통과 확인되면 wall-collision retreat 차단(미해결 1번) 방향 결정 후 진행.
3. grasp orientation 무시 버그(미해결 2번)는 blast radius가 크므로 사용자와 범위 합의
   먼저 — 추측성 수정 금지.

## 5. 검증 명령 (이번 세션에서 확립된 표준 절차)

```bash
# 1. 컴파일
python3 -m py_compile <수정한 .py 파일들 전부>

# 2. 실제 import (py_compile은 import 오류를 못 잡음 — gripper_client.py 사례 참고)
source /opt/ros/humble/setup.bash && source ~/doosan_ws/install/setup.bash
python3 -c "import curobo_planner_node"   # 등, 수정한 모듈 각각

# 3. 공백/줄바꿈 점검
git -C /home/user/doosan_ws/src/strawberry_finalproject diff --check

# 4. 빌드
cd /home/user/doosan_ws
colcon build --packages-select e0509_gripper_description strawberry_motion --allow-overriding e0509_gripper_description

# 5. 새 .py 모듈을 추가했다면 install space에 실제로 복사됐는지 반드시 확인
ls ~/doosan_ws/install/e0509_gripper_description/lib/e0509_gripper_description/ | grep <새파일>
```

대형 텍스트 추출(메서드 분리)을 할 때는 추가로:

```text
원본 블록 추출 → 변환(continue/return 등 치환, 들여쓰기 조정) → 변환본을 역치환해서
원본과 diff → 0줄 차이 확인 후에만 실제 파일에 적용.
```

## 6. 작업 시 주의할 점 (기존 + 이번 세션 추가)

- `scripts/측정.py` 절대 수정/삭제/커밋 금지.
- untracked 2개 파일 임의 커밋/삭제/restore 금지.
- `git restore`, `rm`, `reset --hard` 금지.
- 실기 동작 파라미터(거리/속도/순서/로그 이벤트명) 임의 변경 금지 — 순수 코드 이동만.
- 커밋은 finalproject repo에서만.
- 새 `.py` 모듈 추가 시 CMakeLists.txt `install(PROGRAMS ...)` 등록 필수 + install space
  `ls` 확인 필수(빠뜨리면 `colcon build`는 조용히 통과하지만 `ros2 run`에서 `ImportError`).
- 대형 추출은 역치환 diff로 0줄 차이 증명 후에만 적용.
- `NW_TROUBLESHOOTING_CASE_LOG_20260621.md`는 발표 자료로 쓰이고 있으니 새 사고/수정이
  생기면 결과(해결/미해결 무관) 추가 기록할 것.

## 7. Codex에게 바로 시킬 다음 작업 (추천 프롬프트)

```text
docs/HANDOFF_20260621_REFACTOR_COMPLETE_AND_STATUS_FOR_CODEX.md를 먼저 읽어.
공식 repo는 /home/user/doosan_ws/src/strawberry_finalproject이고, 커밋은 반드시 거기서 해.
scripts/측정.py는 절대 건드리지 마.

이번 세션의 리팩토링 전부(_pick 분리, fusion/scan_executor의 _loop·__init__ 분리,
tray_place_executor 추가분리)는 아직 실기 미검증 상태다. 코드를 더 건드리기 전에,
다음 실기 때 SW 먼저 저속/단일 target으로 한 사이클 돌려서 전과 동일하게 동작하는지
확인하는 절차를 준비해줘 (어떤 로그/이벤트를 보면 되는지, 회귀 시 어느 커밋이
의심되는지 매핑까지).

미해결 1번(wall-collision retreat 차단)은 사용자가 방향을 보류한 상태이니 먼저
3가지 후보 중 어느 쪽으로 갈지 사용자에게 물어보고 진행해.
미해결 2번(grasp orientation 무시 버그)은 blast radius가 커서 사용자 동의 없이
손대지 마.
```
