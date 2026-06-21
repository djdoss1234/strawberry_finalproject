# Claude Code 인계서 — NW flat 수확 실패/얕은 파지 + cuRobo planner 본체 리팩토링 (2026-06-21)

이 문서는 2026-06-21 저녁 Codex 세션의 최신 상태만 정리한다.  
기존 긴 문서보다 이 파일을 먼저 읽고, 필요하면 아래 문서들을 보조로 확인할 것.

```text
docs/HANDOFF_20260621_REFACTOR_COMPLETE_AND_STATUS_FOR_CODEX.md
docs/HANDOFF_20260620_RETREAT_FIX_AND_GRASP_RATE_FOR_CODEX.md
docs/RUNTIME_MODULE_INTERFACE_SPEC_20260620.md
```

공식 repo:

```bash
cd /home/user/doosan_ws/src/strawberry_finalproject
git status --short --branch
```

작업 패키지 경로:

```text
/home/user/doosan_ws/src/e0509_gripper_description
  -> /home/user/doosan_ws/src/strawberry_finalproject/ros_packages/e0509_gripper_description
```

절대 건드리지 말 것:

```text
scripts/측정.py
```

## 1. 현재 사용자 관찰

NW flat 단일 scan pose에서 SW처럼 수평 접근을 시도 중이다.  
하지만 최신 실기에서도 딸기 앞까지 충분히 들어가지 못하고, 한참 앞에서 파지 시도 후 빈손이 된다.

사용자 관찰:

```text
- 직진처럼 보이지 않거나, cuRobo spline으로 들어가서 SW baseline처럼 보이지 않음
- 딸기 앞쪽에서 close됨
- present_pos=699, current_raw 낮음 -> 실제 파지 없음
- cuRobo planner 노드가 refactor 후에도 2000줄에 가까워서 유지보수 불가
```

## 2. 최신 실패 로그 핵심

첨부 로그:

```text
/home/user/.codex/attachments/a2430b75-add3-443d-b285-cc822940752c/pasted-text.txt
Runtime JSONL:
logs/runtime/2026-06-21/curobo_planner_node_20260621T192221-3109b00e.jsonl
```

핵심 줄:

```text
Detection Y=804mm > wall surface 672mm — clamped to 672mm
=== PICK raw=(-324,672,784)mm grasp=(-324,672,784)mm det_y=804mm ===
FLAT_GRASP_ONLY: using 0deg wall-normal grasp variant
Plan OK pre goal y=612mm z=814mm
Plan FAIL final y=792/762/742/722/702mm
Plan OK final probe y=682mm, depth=70mm
FLAT_GRASP_TARGET_PLANE_CAP: 180mm -> 80mm
PRE_APPROACH_REACHED before 80mm straight approach
FINAL_APPROACH_PRECOMPUTED_CUROBO depth=70mm
GRASP_POSE_REACHED pre=6cm+70mm
OPEN_STEM_DESCENT 30mm
VERIFY_GRASP: GRASP_UNVERIFIED present_pos=699 current_raw=54
```

해석:

```text
fusion 실제 검출 Y(det_y)는 804mm
planner target은 wall clamp 때문에 672mm
pre-approach는 y=612mm
target-plane distance는 60mm
flat margin 20mm를 넣어 requested final approach는 80mm
그런데 실행은 80mm가 아니라 이전 probe로 찾은 cuRobo 70mm spline을 재사용
결과적으로 실제 딸기보다 앞에서 close
```

즉 현재 실패의 직접 원인은 두 가지가 겹친다.

1. `pick_target_policy.py`가 raw detection Y를 `WALL_SURFACE_Y_M=0.672`로 강제 clamp한다.
2. `curobo_planner_node.py`가 requested 80mm를 계산했지만, precomputed probe 70mm를 최종 접근으로 재사용한다.

## 3. 오늘 적용된 변경

### 3.1 NW flat 단일 scan pose

`config/scan_pose_candidates_refit_candidate.yaml`에 `root/nw_flat` 추가.

사용자 티칭 pose:

```text
joint_deg = [-205.49, 2.38, 42.72, -75.77, 71.08, -46.02]
TCP BASE [mm,deg] = [-245.67, 292.38, 851.58, 87.75, 86.32, -89.49]
```

단, scan executor 쪽에서는 J1 wrap issue 때문에 실제 저장/이동 branch를 주의해야 한다.
최근 작업에서는 overview gate만 J1/J4/J6 wrap 허용, 실제 move target은 J1 branch를 보존하도록 수정했다.

### 3.2 정상 딸기 인식 KPI 로그

`scripts/strawberry_fusion_node.py`에 `perception_candidate_summary` runtime JSONL 이벤트 추가.

목적:

```text
raw detection 수와 실제 pick 가능한 stable candidate 수를 분리 기록
4세부 scan vs 단일 중심 scan 비교 근거 확보
```

기록 항목:

```text
seg_ripe_count
scene_ripe_3d_count
pose_detection_count
valid_stable_pick_candidate_count
stable_track_count
active_target_selected
active_target_pos_m
published_target
```

### 3.3 flat grasp 모드

`scripts/planner_bootstrap.py`

```text
flat_grasp_only
flat_grasp_target_plane_margin_m
```

`scripts/curobo_planner_node.py`

```text
FLAT_GRASP_ONLY: 0deg wall-normal grasp variant만 사용
FLAT_GRASP_TARGET_PLANE_CAP: legacy 180mm 대신 target-plane + margin 사용
```

현재 기본값:

```text
flat_grasp_target_plane_margin_m = 0.020
```

이 패치로 60mm에서 80mm까지는 늘어났지만, 실제 실행은 아직 70mm probe 재사용 때문에 충분하지 않다.

### 3.4 로그 보강

`=== PICK ... ===` 로그에 `det_y` 추가.

예:

```text
raw=(-324,672,784)mm grasp=(-324,672,784)mm det_y=804mm
```

이제 raw/grasp는 clamp 후 값이고, `det_y`는 fusion에서 받은 실제 detection Y라는 것을 구분할 수 있다.

## 4. 지금 실패의 기술적 원인

### 4.1 wall clamp가 깊이를 잘라낸다

파일:

```text
scripts/pick_target_policy.py
```

현재 로직:

```python
detection_raw_y = float(position.y)
raw_y = detection_raw_y
wall_y_clamped = raw_y > WALL_SURFACE_Y_M
if wall_y_clamped:
    raw_y = WALL_SURFACE_Y_M
```

문제:

```text
det_y=804mm인데 raw_y/grasp_y가 672mm로 잘림
실제 딸기 위치보다 132mm 앞쪽을 기준으로 접근 거리 계산
```

이 clamp는 원래 벽 뒤로 들어가는 위험한 target을 막기 위한 guard였지만, 현재 NW flat 실험에서는
실제 파지 깊이를 과도하게 앞쪽으로 제한하고 있다.

### 4.2 requested 80mm와 executed 70mm가 다르다

파일:

```text
scripts/curobo_planner_node.py
```

최신 로그:

```text
FLAT_GRASP_TARGET_PLANE_CAP: 180mm -> 80mm
FINAL_APPROACH_PRECOMPUTED_CUROBO depth=70mm
GRASP_POSE_REACHED pre=6cm+70mm
```

문제:

```text
최종 접근 거리 계산은 80mm
하지만 depth probe 중 성공한 70mm cuRobo plan을 최종 접근으로 재사용
따라서 사용자가 본 것처럼 딸기 앞에서 닫음
```

`_try_precomputed_final_approach()`는 현재 probe depth가 requested보다 깊을 때만 skip한다.
하지만 지금처럼 probe가 requested보다 얕은 경우도 flat grasp에서는 재사용하면 안 된다.

다음 수정 1순위:

```text
flat_grasp_only=True일 때는 precomputed final approach를 최종 실행으로 재사용하지 말 것.
또는 abs(probe_depth - requested_distance) <= 5mm일 때만 재사용.
그 외에는 requested distance를 정확히 계획하거나, 실패하면 실패로 처리해야 함.
```

주의:

```text
80mm exact plan이 IK_FAIL일 수 있다.
그 경우 "70mm까지만 reachable"이 진짜 한계인지, wall collision model이 막는 것인지 구분해야 한다.
```

### 4.3 SW처럼 직선 접근하지 않는 이유

최신 로그에서는 최종 접근이 Doosan MoveLine이 아니라 cuRobo precomputed spline으로 실행됐다.

```text
FINAL_APPROACH_PRECOMPUTED_CUROBO depth=70mm
Spline 12pts ...
```

그래서 사용자가 보는 모션이 "SW처럼 직선"이 아니라 "spline으로 대충 잡으러 감"처럼 보인다.

다음 수정 2순위:

```text
flat_grasp_only=True일 때 final approach는 반드시
1. exact requested depth의 Cartesian/BASE straight line
2. 또는 실패 시 명시적으로 abort
로 제한한다.

precomputed cuRobo final spline은 reachability probe로만 쓰고, 실행하지 않는 옵션이 필요하다.
```

추천 파라미터:

```text
flat_grasp_execute_precomputed_final:=false
```

기본값은 `false`가 안전하다. SW regression을 막기 위해 `flat_grasp_only`일 때만 적용.

## 5. 실행 커맨드 현황

Planner:

```bash
source ~/doosan_ws/install/setup.bash

ros2 run e0509_gripper_description curobo_planner_node.py --ros-args \
  -p measured_tcp_plan_only:=false \
  -p nw_high_target_z_threshold_m:=2.0 \
  -p flat_grasp_only:=true \
  -p flat_grasp_target_plane_margin_m:=0.020 \
  -p direct_curobo_final_approach_for_measured_tcp:=true \
  -p measured_tcp_tool_line_after_curobo_fallback:=true
```

Scan:

```bash
source ~/doosan_ws/install/setup.bash

ros2 launch strawberry_motion workspace_scan.launch.py \
  enable_robot_execution:=true \
  target_cell:=root/nw_flat \
  enable_fusion_detection:=true \
  enable_pick_integration:=true \
  collect_then_pick:=false \
  max_total_picks:=1 \
  scan_movej_vel_deg_s:=10.0 \
  scan_movej_acc_deg_s2:=20.0 \
  overview_return_vel_deg_s:=10.0 \
  overview_return_acc_deg_s2:=20.0
```

Trigger:

```bash
ros2 service call /strawberry/scan/start std_srvs/srv/Trigger "{}"
```

## 6. 다음 코드 수정 추천 순서

### A. precomputed final approach 재사용 금지/제한

파일:

```text
scripts/curobo_planner_node.py
```

함수:

```text
_try_precomputed_final_approach(...)
```

수정 방향:

```text
if self._flat_grasp_only and abs(selected_depth - requested_distance) > 0.005:
    log skip
    return False, False
```

이렇게 하면 requested 80mm인데 70mm probe를 재사용하지 않는다.

예상 로그:

```text
FINAL_APPROACH_PRECOMPUTED_CUROBO_SKIPPED: probe depth 70mm != requested 80mm
FINAL_APPROACH_STRAIGHT_BASE ... 80mm
```

### B. 80mm exact 접근이 안 되면 wall/collision model 진단

만약 80mm exact가 IK_FAIL이면 다음을 비교한다.

```text
target y=672 + 70mm -> OK
target y=672 + 80mm -> FAIL
target y=672 + 90mm -> FAIL
```

이 경우 실제 원인은 딸기 위치가 아니라 cuRobo world의 wall surface/collision guard가 너무 보수적인 것일 수 있다.

진단할 것:

```text
config/environment.yaml의 whiteboard wall 위치/두께
WALL_SURFACE_Y_M=0.672
실제 카메라/로봇 FK 기준 detection det_y=804가 왜 계속 벽 뒤로 나오는지
```

### C. flat grasp에서 최종 접근은 spline 금지

사용자가 원하는 것은 SW처럼:

```text
pre-approach spline
-> final straight approach
-> open descent
-> close
-> detach
```

따라서 flat mode에서는 최종 접근에 cuRobo spline을 쓰면 사용자 기대와 다르다.

정책:

```text
flat_grasp_only=True:
  precomputed cuRobo final = reachability check only
  execution = BASE/TOOL straight line only
  straight line no-motion/timeout이면 abort
```

단, Doosan MoveLine이 success/no-motion을 반환하는 branch가 있었으므로, 이 부분은
`execute_base_relative_line()`과 robot motion service 상태를 같이 봐야 한다.

## 7. cuRobo planner 2000줄 문제

2026-06-21 기준:

```text
scripts/curobo_planner_node.py = 약 1978 lines
```

리팩토링을 했지만 아직 "짤짤이" 수준이고, 진짜 큰 본체가 남아 있다.

이미 분리된 것:

```text
planner_bootstrap.py
grasp_search_executor.py
tray_place_executor.py
approach_retreat_policy.py
pick_target_policy.py
grasp_candidate_policy.py
gripper_client.py
motion_execution_helpers.py
runtime_logging.py
```

하지만 `curobo_planner_node.py`에는 아직 다음 책임이 섞여 있다.

```text
ROS node wiring/subscription/client 생성
pick state machine
target 준비
final approach 실행
detach/retreat 실행
place gate
world/neighbor obstacle 관리
collision diagnostic
fixed pose return
runtime logging orchestration
```

다음 리팩토링은 helper 추가가 아니라 큰 보스 분리여야 한다.

### 7.1 final_approach_executor.py

옮길 대상:

```text
_compute_final_approach_distance
_try_precomputed_final_approach
_execute_final_approach_tool_finish
_try_final_approach_fallback
_execute_final_approach
```

입력:

```text
raw_straw
straw
used_pre_ee_pos
used_approach_dir
used_grasp_offset
used_grasp_variant
ret_grasp
measured_best_depth_m
planner params
callbacks: plan(), execute_spline(), execute_base_relative_line(), execute_tool_z_line()
runtime_log
logger
```

출력:

```text
success/fail
FinalApproachState
executed_distance_m
tool_finish_executed_m
tool_finish_executed_dir
failure_reason
```

### 7.2 pick_sequence_executor.py

옮길 대상:

```text
_pick
_prepare_pick_target_or_abort 호출부
_search_grasp 호출부
open gripper
final approach
open stem descent
grasp verify
detach/retreat
place gate
return scan pose
pick_complete publish
```

입력:

```text
PoseStamped pick target
current_joints
motion_gen/world handles
gripper_client
final_approach_executor
grasp_search_executor
tray_place_executor
runtime_log
publishers
```

출력:

```text
PickResult / result_code
grasp_status
retreat_status
place_status
pick_complete published 여부
```

### 7.3 planner_world_manager.py

옮길 대상:

```text
_register_neighbor_obstacles
_clear_neighbor_obstacles
collision diag
environment/world update wrappers
```

입력:

```text
target position
neighbor detections
environment config
motion_gen world API
```

출력:

```text
world update result
registered obstacle count
collision suspect list
```

### 7.4 RUNTIME_MODULE_INTERFACE_SPEC_20260620.md 최신화 필요

리팩토링을 계속하면 반드시 이 문서도 같이 갱신해야 한다.

갱신해야 할 내용:

```text
새 모듈명
각 모듈 책임
입력/출력 데이터
ROS topic/service/action이 어디에서 소비/발행되는지
runtime JSONL 이벤트
실기 검증 여부
```

특히 `final_approach_executor.py`, `pick_sequence_executor.py`를 추가하면
`RUNTIME_MODULE_INTERFACE_SPEC_20260620.md`에 반드시 표로 넣어야 한다.

## 8. 현재 미해결 문제 목록

우선순위순:

1. NW flat에서 최종 접근이 여전히 얕음
   - requested 80mm인데 precomputed 70mm spline 재사용
   - `present_pos=699`, 빈 파지

2. wall_y_clamped가 계속 발생
   - det_y≈793~804mm
   - planner target y=672mm
   - 실제 깊이와 collision world가 불일치

3. flat mode에서도 최종 접근이 SW식 직선으로 보장되지 않음
   - precomputed cuRobo spline이 실행되면 사용자가 보는 모션이 SW와 다름

4. cuRobo planner 노드가 여전히 1978줄
   - helper는 늘었지만 orchestration 본체가 안 잘림

5. gripper service가 가끔 INITIALIZE status 3 / TCP 20002 refused
   - `scripts/clean_gripper_runtime.sh`
   - `scripts/start_gripper_service_stable.sh`
   - `scripts/restart_gripper_drl_then_start.sh`
   참고

6. SW regression 미검증
   - flat/NW 패치가 SW baseline을 깨지 않는지 확인 필요

## 9. 검증 명령

수정 후 최소 검증:

```bash
cd /home/user/doosan_ws/src/e0509_gripper_description
python3 -m py_compile \
  scripts/curobo_planner_node.py \
  scripts/planner_bootstrap.py \
  scripts/pick_target_policy.py \
  scripts/approach_retreat_policy.py

cd /home/user/doosan_ws/src/strawberry_finalproject
git diff --check

cd /home/user/doosan_ws
colcon build --packages-select e0509_gripper_description --allow-overriding e0509_gripper_description
```

실기 전 확인할 로그:

```text
det_y=...
FLAT_GRASP_ONLY
FLAT_GRASP_TARGET_PLANE_CAP
FINAL_APPROACH_PRECOMPUTED_CUROBO_SKIPPED or FINAL_APPROACH_STRAIGHT_BASE
GRASP_POSE_REACHED pre=6cm+??mm
VERIFY_GRASP present_pos/current_raw
```

## 10. Claude Code에게 바로 줄 지시문

```text
docs/HANDOFF_20260621_NW_FLAT_SHALLOW_AND_REFACTOR_FOR_CLAUDE.md부터 읽어.

최신 실패 원인은 requested final approach 80mm를 계산했는데,
precomputed cuRobo probe 70mm를 최종 접근으로 재사용해서 여전히 앞에서 닫는 것임.

1순위:
flat_grasp_only=True일 때 _try_precomputed_final_approach는
abs(probe_depth - requested_distance) <= 5mm일 때만 실행하게 수정.
그 외에는 skip하고 exact requested distance를 straight/fallback으로 시도.

2순위:
flat_grasp_only=True에서는 최종 접근에 cuRobo spline을 실행하지 않는 옵션
flat_grasp_execute_precomputed_final:=false 추가 검토.
SW처럼 pre-approach spline -> straight line -> open descent -> close 구조를 보장.

3순위:
80mm exact도 IK_FAIL이면 wall_y_clamped/collision world 문제로 분리 진단.
det_y≈804mm인데 target y=672mm로 clamp되는 구조를 확인.

4순위:
curobo_planner_node.py가 아직 약 1978줄이므로
final_approach_executor.py와 pick_sequence_executor.py로 큰 본체를 분리.
짤짤이 helper 추가가 아니라 pick/final approach state machine을 옮겨야 함.

리팩토링 후 docs/RUNTIME_MODULE_INTERFACE_SPEC_20260620.md를 반드시 최신화.
모듈별 입력/출력, ROS topic/service/action, runtime JSONL 이벤트를 표로 정리.

scripts/측정.py는 절대 건드리지 말 것.
커밋은 /home/user/doosan_ws/src/strawberry_finalproject에서만.
```

