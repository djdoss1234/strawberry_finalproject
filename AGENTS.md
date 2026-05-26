# Strawberry Final Project 작업 지침

## 목적

이 저장소는 실제에 가까운 딸기 수확 환경에서 동작할 **수확 모션 시스템**
개발을 중심으로 관리합니다. 복잡한 환경에서의 VLA 기반 판단 모듈은
팀원이 개발하며, 본 저장소에서는 모션 측의 안전한 연동 구조를 함께
정의합니다.

## 담당 범위

저장소 소유자의 주 담당 범위:

- 로봇 수확 motion behavior
- planning 및 trajectory execution 통합
- approach, grasp, retreat, transfer, place sequence
- tray localization 연동과 place target 생성
- collision scene, retry 처리, diagnostics, motion 평가

협업 팀원의 주 담당 범위:

- 복잡한 수확 장면에 대한 VLA reasoning
- 가림, 위험, 수확 가능성에 대한 semantic 판단
- target selection과 high-level action 제안

VLA의 출력은 **제안값**으로 취급합니다. 실제 로봇 명령을 내리기 전에
모션 시스템이 target pose 유효성, workspace/collision 제약, 실행 준비
상태, 실패 시 복구 가능성을 검증해야 합니다.

## Baseline Reference

runtime 코드를 이식하거나 과거 해결 내용을 확인할 때는 미니프로젝트
저장소를 먼저 참고합니다.

- https://github.com/djdoss1234/strawberry_miniproject

해당 저장소에는 이전 `RealSense/YOLO/cuRobo/Doosan` prototype,
system architecture, 실험 결과 요약, 회고 및 향후 계획이 들어 있습니다.

## 작업 우선순위

1. 큰 refactoring 전에 실행 가능한 baseline을 보존합니다.
2. 로봇 동작은 성공/실패 label과 함께 측정 가능하게 만듭니다.
3. target 판단과 motion validation/execution의 책임을 분리합니다.
4. calibration, model weight, raw 실험 artifact는 공개 commit에 넣지 않습니다.
5. 통합 디버깅 시 `rqt_graph`, TF tree, RViz scene, experiment log를 근거로 남깁니다.

## 계획 중인 Motion Architecture

```text
perception_or_vla_target
  -> target_validation
  -> pick_place_state_machine
  -> planner_backend
  -> collision_and_safety_check
  -> robot_executor
  -> result_logger
```

구현이 시작되면 다음과 같은 모듈 구성을 우선 검토합니다.

```text
perception/
planning/
task/
tray/
interfaces/
diagnostics/
config/
launch/
docs/
```

처음부터 모든 파일을 쪼개지 말고, 동작하는 baseline을 유지하면서
`tray localization`, `slot manager`, `pick_place_state_machine`,
`planner backend` 순서로 책임을 분리합니다.

## VLA 연동 방향

VLA 측에서 전달받을 structured proposal 후보:

- target identifier
- target pose 또는 target region reference
- 수확 가능성/confidence
- occlusion 또는 risk annotation
- 요청 action: `PICK`, `REOBSERVE`, `SKIP` 등

모션 측에서 반환할 execution result 후보:

- `SUCCESS`
- `INVALID_TARGET`
- `IK_FAIL`
- `PLANNING_FAIL`
- `COLLISION_RISK`
- `GRASP_FAIL`
- `PLACE_FAIL`
- `REOBSERVE_REQUIRED`

message type, topic/action name은 양쪽 runtime 요구사항을 검토하기 전에는
성급하게 확정하지 않습니다.

## 첫 구현 순서

1. 평가 지표, hardware scene, ROS interface, log schema를 정의합니다.
2. marker 기반 tray localization과 slot 생성 기능을 구현합니다.
3. 미니프로젝트의 안정적인 pick & place motion component를 이식합니다.
4. motion sequence를 state machine과 planner module로 분리합니다.
5. 실제형 딸기 모형 환경에서 반복 실험을 수행합니다.
6. VLA proposal/result integration boundary를 추가합니다.

## 세션이 끊긴 뒤 이어서 작업하는 규칙

새 세션의 Codex나 협업자가 이 저장소를 열었을 때는 다음 순서로 맥락을
복원합니다.

1. `README.md`를 읽고 프로젝트 목적, 역할 분담, 현재 milestone을 확인합니다.
2. 이 파일에서 설계 원칙과 작업 우선순위를 확인합니다.
3. `docs/project_scope.md`에서 interface 및 평가 항목을 확인합니다.
4. `docs/development_roadmap.md`에서 전체 단계와 당장 진행할 작업을 확인합니다.
5. `git status`, `git log --oneline --decorate -10`, 최근 commit diff를 확인합니다.
6. 코드가 추가된 이후에는 관련 launch/config/module과 최근 실험 문서를 확인합니다.
7. 이전 구현의 동작 방식이 필요할 때만 `strawberry_miniproject`를 참조합니다.

진행 중 중요한 결정은 다음 세션에서 추측하지 않도록 이 파일 또는
`docs/` 문서와 commit message에 반드시 남깁니다. 예를 들면:

- 실제 사용한 tray marker 종류와 frame 정의
- 선택한 planner baseline과 비교 결과
- VLA와 합의한 request/result interface
- 현재 성공한 demo 조건과 실패 중인 조건
- 보정값이나 장비 의존 설정의 저장 위치
