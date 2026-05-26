# RUN-20260526-001: Quadtree Workspace Marker Node 검증

## 기본 정보

| 항목 | 내용 |
| --- | --- |
| 날짜 | 2026-05-26 |
| 담당자 | djdoss1234 |
| 단계 | exploration visualization |
| scene | 실제 로봇 motion 없음, ROS topic/node 검증 |
| commit | `a95ca8e` |
| 관련 issue | `ISSUE-20260526-001`, `ISSUE-20260526-002` |

## 목적과 완료 기준

목적:

- quadtree workspace cell 상태를 ROS 2 node로 publish할 수 있는지 확인합니다.
- 향후 RViz와 motion planner가 연결될 topic 경계를 먼저 검증합니다.

완료 기준:

- `workspace_marker_node`가 실행됩니다.
- 초기 leaf cell 4개가 구성됩니다.
- `/strawberry/exploration/workspace_cells` publisher가 노출됩니다.
- 초기 next cell이 `root/nw`이며 상태 갱신 후 다음 cell로 진행합니다.

## 입력 조건

- ROS 2: Humble
- workspace: `~/doosan_ws`
- package: `strawberry_motion`
- config: `config/workspace.yaml`
- 임시 frame: `cultivation_panel`
- robot motion: 실행하지 않음

## 실행 절차

```bash
cd ~/doosan_ws
colcon build --packages-select strawberry_motion --symlink-install
source install/setup.bash
ros2 run strawberry_motion workspace_marker_node
ros2 topic echo --once /strawberry/exploration/next_cell std_msgs/msg/String
ros2 topic pub --once /strawberry/exploration/set_cell_state \
  std_msgs/msg/String "{data: 'root/nw=SCANNED_EMPTY'}"
ros2 topic echo --once /strawberry/exploration/next_cell std_msgs/msg/String
```

## 확인한 ROS 연결

```text
/strawberry/exploration/set_cell_state
  -> /workspace_marker_node
  -> /strawberry/exploration/workspace_cells
  -> /strawberry/exploration/next_cell
```

## 관찰 결과

- `workspace_marker_node` 실행 성공: `frame=cultivation_panel`, `leaf_cells=4`
- 초기 `/strawberry/exploration/next_cell`: `root/nw`
- `root/nw=SCANNED_EMPTY` 입력 후 next cell: `root/ne`
- MarkerArray publisher QoS: `TRANSIENT_LOCAL`
- `colcon build` 성공
- 단위 테스트 7개 통과
- 기존 로봇 graph에서 `/dsr01/gripper_service_node` 중복 이름 경고를 발견함
- RViz 실제 화면 표시와 캡처는 아직 수행하지 않음

## 시각자료 계획 및 확보 상태

| 자료 | 필요 장면 | 상태 | 원본 위치 | 공개 위치/사용처 |
| --- | --- | --- | --- | --- |
| ROS graph 캡처 | `/workspace_marker_node`와 exploration topic 연결이 보이는 화면 | `NOT_CAPTURED` | `artifacts/RUN-20260526-001/raw/rqt_graph_full.png` | `docs/assets/exploration/RUN-20260526-001_rqt_graph_workspace_node.png`, GitHub/포트폴리오 |
| RViz 캡처 | 네 개 cell과 next cell marker가 보이는 화면 | `NOT_CAPTURED` | `artifacts/RUN-20260526-001/raw/rviz_full.png` | `docs/assets/exploration/RUN-20260526-001_rviz_quadtree_cells.png`, GitHub/Notion |
| 상태 전환 캡처 | `root/nw` 완료 전후 next cell이 바뀐 표시 | `NOT_CAPTURED` | `artifacts/RUN-20260526-001/raw/state_transition.png` | `docs/assets/exploration/RUN-20260526-001_next_cell_transition.png`, 포트폴리오 |

<!-- VISUAL TODO
asset_id: RUN-20260526-001_rqt_graph_workspace_node
capture: rqt_graph에서 workspace_marker_node와 exploration topic 연결이 드러나는 화면
source_path: artifacts/RUN-20260526-001/raw/rqt_graph_full.png
public_path: docs/assets/exploration/RUN-20260526-001_rqt_graph_workspace_node.png
use_in: GitHub README 진행상태, 포트폴리오 architecture, Notion Experiment Run
status: NOT_CAPTURED
-->

<!-- VISUAL TODO
asset_id: RUN-20260526-001_rviz_quadtree_cells
capture: RViz fixed frame cultivation_panel에서 cell 4개와 next cell marker가 표시된 화면
source_path: artifacts/RUN-20260526-001/raw/rviz_full.png
public_path: docs/assets/exploration/RUN-20260526-001_rviz_quadtree_cells.png
use_in: GitHub README, 포트폴리오 탐색 모듈 설명, Notion Experiment Run
status: NOT_CAPTURED
-->

## 판정

```text
PARTIAL
```

판정 근거:

- ROS topic/state update 기반 exploration visualization interface는 검증했습니다.
- RViz 화면 검증과 실제 scan pose 생성/robot motion 연결은 다음 run이 필요합니다.

## 다음 행동

1. RViz에서 `/strawberry/exploration/workspace_cells` MarkerArray 표시 확인
2. 위 표의 대표 RViz/graph 캡처를 `artifacts/RUN-20260526-001/raw/`에 보관
3. 공개 가능한 두 장을 `docs/assets/exploration/`에 추가
4. `scan_pose_generator` 구현
5. 실제 frame/stand-off distance는 테스트베드 실측 후 확정

## 포트폴리오/자소서 후보 문장

> 실제형 수확 환경으로 확장하기 전, 재배 영역을 quadtree cell로 관리하는
> exploration core와 ROS visualization interface를 분리 구현했습니다.
> cell 상태 입력에 따라 다음 관찰 영역이 갱신되는 흐름을 topic 수준에서
> 검증하여, 이후 scan motion과 VLA 연동을 위한 공간 상태 관리 기반을 마련했습니다.
