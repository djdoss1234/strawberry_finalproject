# Claude Code 인계서 — 수확 런타임 리팩토링 계속하기 (2026-06-21)

## 0. 반드시 먼저 확인할 것

공식 repo는 다음 경로다.

```bash
cd /home/user/doosan_ws/src/strawberry_finalproject
git status --short --branch
```

현재 작업 패키지는 finalproject 내부의 아래 패키지다.

```text
/home/user/doosan_ws/src/strawberry_finalproject/ros_packages/e0509_gripper_description
```

현재 Codex 세션 cwd가 `/home/user/doosan_ws/src/e0509_gripper_description`로 보일 수 있는데,
이 경로는 finalproject 내부 패키지 작업 경로와 연결된 상태다. 커밋은 반드시
`/home/user/doosan_ws/src/strawberry_finalproject`에서 해야 한다.

절대 건드리지 말 것:

```text
scripts/측정.py
```

현재 의도적으로 남아있는 untracked 파일:

```text
config/scan_pose_candidates_depth2.yaml
ros_packages/e0509_gripper_description/config/camera_calibration_eye_in_hand.yaml
```

위 두 파일은 사용자가 별도 지시하기 전까지 커밋하지 말고, 삭제/restore도 하지 않는다.

현재 브랜치:

```text
debug/nw-return-to-depth-good
```

## 1. 이번 리팩토링의 목적

목적은 "동작 최적화"가 아니라 `scripts/curobo_planner_node.py`에 몰린 책임을
조심스럽게 나누는 것이다.

원칙:

- 실기 모션 의미를 바꾸지 않는다.
- 파라미터 값, 거리, 속도, fallback 순서, safety guard를 임의로 바꾸지 않는다.
- 한 번에 대형 재작성하지 않는다.
- 순수 계산/정책/반복 cleanup/로그 helper부터 자른다.
- 변경 후 항상 `py_compile`, `diff --check`, `colcon build`를 통과시킨다.
- 통과하면 작은 단위로 commit/push한다.

검증 루틴:

```bash
cd /home/user/doosan_ws/src/e0509_gripper_description
python3 -m py_compile scripts/curobo_planner_node.py scripts/approach_retreat_policy.py scripts/grasp_candidate_policy.py

git -C /home/user/doosan_ws/src/strawberry_finalproject diff --check

cd /home/user/doosan_ws
colcon build --packages-select e0509_gripper_description strawberry_motion --allow-overriding e0509_gripper_description
```

커밋:

```bash
git -C /home/user/doosan_ws/src/strawberry_finalproject add <수정파일>
git -C /home/user/doosan_ws/src/strawberry_finalproject commit -m "<message>"
git -C /home/user/doosan_ws/src/strawberry_finalproject push
```

## 2. Codex 작업 방식

Codex는 다음 방식으로 진행했다.

1. `curobo_planner_node.py`에서 같은 책임이 반복되는 블록을 찾는다.
2. 동작 변경 위험이 낮은 순서로 자른다.
   - 순수 계산
   - 조건/정책 함수
   - 반복 cleanup
   - 로그+상태 갱신 helper
   - 실행 helper
3. 실제 로봇 motion command의 의미는 유지한다.
4. helper를 만들더라도 기존 로그 이벤트명과 result code는 유지한다.
5. 새 모듈을 만들 때는 가능하면 ROS node에 의존하지 않는 pure function으로 둔다.
6. `curobo_planner_node.py`에서 아직 `self.plan`, `execute_spline`, `execute_tool_z_line` 같은
   실제 실행을 많이 들고 있으므로, 이 부분은 한 번에 분리하지 않고 단계적으로 진행한다.

## 3. 지금까지 분리된 주요 모듈

현재 실행 경로 핵심 모듈:

```text
scripts/curobo_planner_node.py
scripts/approach_retreat_policy.py
scripts/grasp_candidate_policy.py
scripts/gripper_client.py
scripts/doosan_motion_client.py
scripts/curobo_planning_adapter.py
scripts/curobo_kinematics_adapter.py
scripts/scene_obstacle_manager.py
scripts/pick_target_policy.py
scripts/open_stem_descent_policy.py
scripts/place_sequence_policy.py
scripts/tray_place_policy.py
scripts/row2_place_policy.py
scripts/marker_place_orientation_policy.py
scripts/trajectory_guards.py
scripts/harvest_motion_params.py
scripts/harvest_math.py
scripts/harvest_grasp_orientation.py
scripts/harvest_result_policy.py
```

상세 인터페이스 문서:

```text
docs/RUNTIME_MODULE_INTERFACE_SPEC_20260620.md
```

## 4. 이번 Codex 리팩토링에서 완료한 것

최근 주요 커밋:

```text
2fbac96 refactor: extract precomputed final approach branch
e68b1ef refactor: track final approach state explicitly
36370e6 refactor: extract final approach distance calculation
6daa0a0 refactor: extract final approach tool finish step
4d648d5 docs: note pick substep helper extraction
63a2a92 refactor: extract leftmost extra advance step
5920fb7 refactor: extract pick completion return step
4410243 refactor: extract post-retreat place handling
87070f8 refactor: extract detach and retreat step
db48c68 refactor: extract gripper close failure recovery
3d681b7 refactor: extract open descent and nudge steps
```

구체적으로 완료:

- `FinalApproachState` 추가
  - 위치: `scripts/approach_retreat_policy.py`
  - 역할: final approach 거리, grasp ee pos, tool-finish retreat 복구값을 묶음.
- `_execute_final_approach_tool_finish(...)` 추가
  - 위치: `scripts/curobo_planner_node.py`
  - 역할: precomputed branch와 fallback branch에 중복되던 tool-finish 실행 통합.
- `_compute_final_approach_distance(...)` 추가
  - measured TCP adaptive distance, extra, cap logging을 `_pick()` 밖으로 이동.
- `_try_precomputed_final_approach(...)` 추가
  - precomputed cuRobo final approach branch 분리.
- `_execute_leftmost_extra_advance_if_needed(...)` 추가
  - leftmost/depth extra advance 실행/로그/hold 처리 분리.
- `_return_to_pick_start_and_complete(...)` 추가
  - pick-start scan pose 복귀와 pick complete logging 분리.
- `_maybe_execute_place_after_retreat(...)` 추가
  - place gate, place outcome, hold/skip/continue 처리 분리.
- `_execute_detach_and_retreat(...)` 추가
  - detach pull과 straight reverse retreat 실행 분리.
- `_handle_gripper_close_failed(...)` 추가
  - gripper close 실패 시 straight retreat 복구 분리.
- `_execute_open_stem_descent_if_needed(...)` 추가
  - 열린 그리퍼로 KP1까지 BASE -Z 하강 분리.
- `_execute_nw_base_y_nudge_if_needed(...)` 추가
  - NW high target depth correction용 BASE +Y nudge 분리.
- `_abort_pick_with_complete(...)` 추가
  - 동일 cleanup 패턴 일부 통합.

주의: 위 helper들은 대부분 아직 `curobo_planner_node.py` 내부 메서드다.
다음 단계에서 별도 executor 모듈로 옮기기 위한 중간 단계다.

## 5. 현재 남은 큰 덩어리

### 5.1 final approach fallback depth search loop

현재 `_pick()` 안에 아직 남아있는 큰 블록:

```text
if not approach_ok:
    fallback_ok = False
    if measured_tcp_model and ENABLE_CUROBO_FINAL_APPROACH_FALLBACK ...
        depth_candidates = final_approach_fallback_depths(...)
        for depth_m in depth_candidates:
            fallback_target = used_pre_ee_pos + depth_m * used_approach_dir
            self.plan(...)
            self.execute_spline(...)
            optional tool_finish
```

다음 추천 작업:

```text
_try_final_approach_fallback(...)
```

형태:

```python
def _try_final_approach_fallback(
    self,
    final_state,
    requested_final_approach_distance,
    used_pre_ee_pos,
    used_approach_dir,
    used_grasp_quat,
    used_grasp_variant,
):
    ...
    return fallback_ok
```

주의:

- `final_state.distance_m`
- `final_state.grasp_ee_pos`
- `final_state.tool_finish_executed_m`
- `final_state.tool_finish_executed_dir`

위 값이 기존 변수와 동일하게 갱신되어야 한다.

### 5.2 final approach 전체 helper화

fallback loop까지 분리한 뒤 다음 단계:

```text
_execute_final_approach(...)
```

이 함수가 담당할 것:

- precomputed cuRobo final approach 시도
- measured TCP direct MoveLine 시도
- legacy TOOL +Z MoveLine 시도
- 실패 시 fallback depth search
- 실패하면 `_abort_pick_with_complete()`

단, 이 단계는 실행 의미가 커지므로 fallback loop 분리 후 한 번 빌드/커밋하고 진행할 것.

### 5.3 grasp search loop 분리

다음 큰 목표:

```text
scripts/grasp_search_executor.py
```

현재 `grasp_candidate_policy.py`는 정책/컨테이너까지만 분리되어 있고,
실제 `self.plan(...)` 호출이 들어간 loop는 아직 `_pick()` 내부에 남아있다.

바로 전부 빼지 말고, 먼저 다음을 더 줄이는 것을 권장:

- measured probe depth loop helper화
- legacy grasp offset loop helper화
- `GraspSearchResult`를 반환하는 executor로 이동

### 5.4 place executor 분리

place 쪽은 이미 정책은 분리되어 있지만 실행은 아직 planner node 안에 많다.

후보:

```text
scripts/tray_place_executor.py
```

단, place는 실기 검증이 섞여 있고 아직 row2 문제가 남아있으므로, 지금 우선순위는
final approach / grasp search 분리보다 낮다.

## 6. 현재 실기 문제와 리팩토링의 관계

리팩토링은 NW 수확 정확도 문제를 직접 해결한 것이 아니다.
현재 리팩토링 목적은 다음 디버깅을 가능하게 하는 구조 정리다.

현재 남은 실기 문제:

- NW/꺾인 줄기에서 파지점 정확도 낮음
- 일부 target에서 옆으로 접근/빗겨감
- final approach fallback/MoveLine/cuRobo branch가 복잡해서 원인 추적 어려움
- fusion orientation을 planner가 얼마나 잘 쓰는지 추가 검증 필요
- SW regression 확인 필요

중요 발견:

- 예전 문제 중 하나는 `msg.pose.orientation`을 planner가 사실상 충분히 활용하지 못하고,
  고정 wall quaternion + variant 위주로 접근하던 구조였다.
- 현재 `published_roll` 후보가 추가되어 있지만, 실기에서 모든 꺾인 줄기에 충분한지는
  아직 검증 부족이다.

## 7. 실행/검증 명령

리팩토링 검증:

```bash
cd /home/user/doosan_ws/src/e0509_gripper_description
python3 -m py_compile scripts/curobo_planner_node.py scripts/approach_retreat_policy.py scripts/grasp_candidate_policy.py

git -C /home/user/doosan_ws/src/strawberry_finalproject diff --check

cd /home/user/doosan_ws
colcon build --packages-select e0509_gripper_description strawberry_motion --allow-overriding e0509_gripper_description
```

실기 실행은 현재 리팩토링 후 아직 다시 검증하지 않았다.
SW/NW 실기 전에 반드시 저속/단일 target으로 확인할 것.

## 8. 작업 시 주의할 점

- `scripts/측정.py` 절대 수정 금지.
- untracked 2개 파일 임의 커밋 금지.
- `git restore`, `rm`, `reset --hard` 금지.
- 실기 동작 파라미터 임의 변경 금지.
- 커밋은 finalproject repo에서만.
- 한 번에 대규모 파일 이동 금지.
- 새 helper를 만들면 기존 로그 이벤트명은 유지.
- 실패/hold/pick_complete publish 여부가 case별로 다르므로 cleanup을 무리하게 통합하지 말 것.
- `pick_complete`는 성공률이 아니라 sequence 종료 이벤트다.

## 9. Claude Code에게 바로 시킬 다음 작업

추천 프롬프트:

```text
docs/HANDOFF_20260621_REFACTOR_FOR_CLAUDE_CODE.md를 먼저 읽고 그대로 이어가.
공식 repo는 /home/user/doosan_ws/src/strawberry_finalproject이고, 커밋은 반드시 거기서 해.
scripts/측정.py는 절대 건드리지 마.

다음 작업은 curobo_planner_node.py 안에 남은 final approach fallback depth search loop를
_try_final_approach_fallback(...) helper로 분리하는 것이다.

동작 변경 금지:
- 거리/속도/파라미터 값 바꾸지 말 것
- fallback depth 순서 바꾸지 말 것
- 로그 이벤트명 바꾸지 말 것
- pick_complete/hold 동작 바꾸지 말 것

분리 후 아래 검증:
python3 -m py_compile scripts/curobo_planner_node.py scripts/approach_retreat_policy.py scripts/grasp_candidate_policy.py
git -C /home/user/doosan_ws/src/strawberry_finalproject diff --check
colcon build --packages-select e0509_gripper_description strawberry_motion --allow-overriding e0509_gripper_description

통과하면 finalproject에 commit/push.
```

