# 포트폴리오 및 자기소개서 Evidence Bank

## 작성 원칙

- 구현 또는 검증된 사실만 `확보한 근거`에 기록합니다.
- 앞으로 할 기능은 `계획`으로 분리합니다.
- 각 claim에는 가능한 경우 run, issue, commit을 연결합니다.

## 프로젝트 역할

본인은 실제에 가까운 딸기 수확 프로젝트에서 **전체 수확 motion system**을
담당합니다. 범위는 작업영역 탐색을 위한 scan motion, target 검증,
approach/grasp/retreat/transfer/place sequence, planner/executor 통합,
collision/retry 대응과 성능 평가입니다.

팀원은 복잡 환경에서 VLA 기반 수확 판단을 담당하며, VLA의 high-level
proposal은 모션 시스템의 geometry/collision 검증을 통과한 뒤 실행하도록
역할 경계를 정의했습니다.

## 현재까지 확보한 근거

### 1. 미니프로젝트 결과를 최종 프로젝트의 baseline으로 분리

- 기존 `RealSense + YOLO + cuRobo + Doosan E0509` 기반 pick & place
  prototype 결과를 별도 저장소에 보존했습니다.
- 최종 프로젝트는 기존 코드를 무작정 복제하지 않고, 안정된 부분을
  모듈화하여 선별 이식하는 전략으로 시작했습니다.

근거:

- baseline: https://github.com/djdoss1234/strawberry_miniproject
- final project 초기 문서: commit `2f7a5d0`, `8a6b048`

### 2. Quadtree 기반 작업영역 탐색 구조를 첫 구현으로 선정

- 본인의 motion 담당 범위에 맞춰 `어디를 관찰할지`를 관리하는 exploration
  layer를 먼저 구축하기로 결정했습니다.
- VLA는 `무엇을 수확할지` 판단하고, motion layer는 `어디를 보고 실제로
  실행 가능한지`를 담당하도록 역할을 분리했습니다.

근거:

- 개발 방향 결정: commit `7d98045`
- roadmap: `docs/development_roadmap.md`

### 3. Exploration Core와 ROS Visualization Interface 구현

- workspace를 quadtree cell로 분할하고 상태를 관리하는 pure Python core를 구현했습니다.
- ROS 2 visualization node를 추가해 workspace cell과 다음 관찰 cell을
  topic으로 노출했습니다.
- cell 상태 update 후 다음 관찰 대상이 전환되는 흐름을 실행 검증했습니다.

근거:

- implementation: commit `762865b`, `a95ca8e`
- execution record: `RUN-20260526-001`
- verified topics:
  - `/strawberry/exploration/workspace_cells`
  - `/strawberry/exploration/next_cell`
  - `/strawberry/exploration/set_cell_state`
- unit tests: 7개 통과

대표 시각자료 배치 예정 위치:

<!-- VISUAL TODO
asset_id: RUN-20260526-001_rviz_quadtree_cells
capture: quadtree cell 상태와 다음 관찰 cell이 표시된 RViz 화면
source_path: artifacts/RUN-20260526-001/raw/rviz_full.png
public_path: docs/assets/exploration/RUN-20260526-001_rviz_quadtree_cells.png
use_in: 포트폴리오 탐색 모듈 섹션, GitHub README, Notion Evidence Bank
status: NOT_CAPTURED
-->

### 4. 물리 Workspace 실측과 Frame 기준 정의

- 화이트보드 내부 종이 4분할 영역을 실제 quadtree workspace로 사용하기
  위해 치수를 측정했습니다.
- 테이프 교차점을 `cultivation_panel` 원점과 root split으로 정의하고,
  비대칭 outer bounds를 config에 반영했습니다.
- 개별 cell 치수 합계가 outer dimension과 맞지 않는 점은 숨기지 않고
  확인이 필요한 issue로 분리했습니다.

근거:

- testbed record: `docs/testbed_setup.md`
- run: `RUN-20260526-002`
- issue: `ISSUE-20260526-003`
- config: `config/workspace.yaml`

### 5. 구현 중 발견한 문제를 실기 연결 전에 해결

- scan 순서가 문자열 정렬에 흔들릴 가능성과 ROS node 종료 traceback을
  최초 topic 점검 단계에서 발견했습니다.
- scan order를 결정적으로 보장하고 shutdown 조건을 정리한 뒤 다시
  실행 검증했습니다.

근거:

- issue: `ISSUE-20260526-001`
- related commit: `a95ca8e`

수정 결과를 보여줄 자료 배치 예정 위치:

<!-- VISUAL TODO
asset_id: RUN-20260526-001_next_cell_transition
capture: root/nw 상태 갱신 전후 next cell 전환이 확인되는 화면
source_path: artifacts/RUN-20260526-001/raw/state_transition.png
public_path: docs/assets/exploration/RUN-20260526-001_next_cell_transition.png
use_in: 포트폴리오 문제 해결 카드, 자기소개서 첨부 자료, Notion Issue page
status: NOT_CAPTURED
-->

## 아직 하지 않은 것

- RViz 화면에서 cell marker 표시 캡처
- 실제 camera observation pose 생성
- robot scan motion 실행
- detector 결과를 cell 상태에 반영
- tray marker localization 및 자동 place
- planner 비교와 collision/retry 고도화
- 팀원의 VLA module 연동

## 단계별로 확보할 대표 시각자료

| 단계 | 자료 | 상태 | 배치 위치 |
| --- | --- | --- | --- |
| Quadtree visualization | RViz cell/next marker 캡처 | `NOT_CAPTURED` | `docs/assets/exploration/` |
| Quadtree ROS 연결 | `rqt_graph` 캡처 | `NOT_CAPTURED` | `docs/assets/exploration/` |
| Physical workspace 정렬 | 종이 4분할과 overview camera 화면 | `NOT_CAPTURED` | `docs/assets/exploration/` |
| Scan pose 생성 | cell + camera pose RViz 캡처 | `NOT_STARTED` | `docs/assets/exploration/` |
| Robot scan motion | 실제 로봇 관찰 순회 영상 | `NOT_STARTED` | 공개 clip 결정 후 `docs/assets/motion/` 또는 외부 링크 |
| Tray 자동 place | tray 이동 전후 place 영상 | `NOT_STARTED` | `docs/assets/tray/` 또는 외부 링크 |
| Planner/collision 개선 | 수정 전후 비교 그림/영상 | `NOT_STARTED` | `docs/assets/motion/` |
| VLA 통합 | proposal-to-result graph/demo | `NOT_STARTED` | `docs/assets/integration/` |
| 최종 결과 | 전체 demo 영상과 지표표 | `NOT_STARTED` | `docs/assets/results/` 및 README |

## 자소서/면접 문장 초안

> 미니프로젝트에서 카메라에 보이는 딸기를 pick & place하는 pipeline을
> 구현한 뒤, 최종 프로젝트에서는 실제 농장형 환경을 고려해 작업영역
> 탐색부터 모듈화했습니다. 저는 quadtree 기반 workspace state map과
> ROS visualization interface를 구현하여, 관찰이 필요한 영역과 재방문
> 대상 영역을 motion system이 관리할 수 있는 기반을 만들었습니다.
> 초기 실행 과정에서는 scan 순서의 비결정성과 node 종료 오류를 발견해
> unit test와 topic 검증으로 수정했으며, 이후 실제 camera scan motion,
> tray 자동 place, VLA 연동으로 확장할 계획입니다.

이 문장은 현재 검증 범위까지만 반영한 초안이며, robot motion과 실제
반복 실험 결과가 확보되면 수치 중심으로 갱신합니다.
