# RUN-20260527-001: RViz Quadtree Marker와 물리 Workspace 방향 대응

## 기본 정보

| 항목 | 내용 |
| --- | --- |
| 날짜 | 2026-05-27 |
| 담당자 | djdoss1234 |
| 단계 | exploration visualization / frame validation |
| scene | 종이 4분할 workspace, robot motion 없음 |
| 입력 근거 | `RUN-20260526-002`, `config/recorded_poses.yaml` |

## 목적과 완료 기준

목적:

- RViz의 quadtree 4개 cell이 실제 camera overview의 `NW/NE/SW/SE`와
  같은 의미로 표시되는지 검증합니다.
- 실제 robot motion을 연결하기 전에 workspace frame 축과 marker
  표현이 혼동 없이 설명 가능한지 확인합니다.

현재 코드/물리 기준:

```text
정면에서 보드를 볼 때:
  +X = 오른쪽
  +Y = 위쪽
  origin = 중앙 테이프 교차점

root/nw = X 음수, Y 양수
root/ne = X 양수, Y 양수
root/sw = X 음수, Y 음수
root/se = X 양수, Y 음수
```

완료 기준:

- RViz fixed frame이 `cultivation_panel`입니다.
- 네 cell label과 다음 관찰 marker(`root/nw`)가 표시됩니다.
- 어제 camera overview 화면의 라벨 방향과 RViz cell label이 대응합니다.
- RViz screenshot을 evidence로 남깁니다.

## 안전 조건

- 이 run은 `workspace_marker_node`와 RViz만 실행합니다.
- Doosan motion service, DART jog, viewer robot control을 실행하지 않습니다.
- `base_link -> cultivation_panel` transform은 아직 미정이므로,
  RViz는 독립된 workspace frame에서 geometry 의미만 확인합니다.
- RViz의 `No tf data`를 피하기 위해
  `workspace_visualization_world -> cultivation_panel` identity TF를
  표시용으로만 publish합니다. 이것은 실측 robot/base transform이 아닙니다.

## 실행 절차

```bash
cd ~/doosan_ws
source install/setup.bash
ros2 launch strawberry_motion workspace_rviz.launch.py
```

참고: 이 launch가 생성하는 identity TF는 marker 화면 표시를 위한
standalone 기준점입니다. 실제 로봇 TF tree와 합쳐 쓰지 않습니다.

RViz에서 확인할 것:

1. `Global Options > Fixed Frame`이 `cultivation_panel`인지 확인합니다.
2. `Quadtree Workspace Cells`에서 네 label이 표시되는지 확인합니다.
3. magenta next-scan marker가 `root/nw` 영역에 있는지 확인합니다.
4. 어제 저장한 overview 화면과 비교해 `NW/NE/SW/SE` 방향을 확인합니다.

상태 전환을 함께 캡처할 경우 별도 terminal에서:

```bash
source ~/doosan_ws/install/setup.bash
ros2 topic pub --once /strawberry/exploration/set_cell_state \
  std_msgs/msg/String "{data: 'root/nw=SCANNED_EMPTY'}"
```

이후 next-scan marker가 `root/ne`로 바뀌는지 확인합니다.

## 시각자료 계획 및 확보 상태

| 자료 | 필요 장면 | 상태 | 원본 위치 | 공개 위치/사용처 |
| --- | --- | --- | --- | --- |
| 기본 cell 방향 | 네 cell label과 `root/nw` next marker | `NOT_CAPTURED` | `artifacts/RUN-20260527-001/raw/rviz_cells.png` | `docs/assets/exploration/RUN-20260527-001_rviz_cells.png`, README/포트폴리오 |
| 상태 전환 | `root/nw` 처리 후 `root/ne` next marker | `NOT_CAPTURED` | `artifacts/RUN-20260527-001/raw/rviz_next_ne.png` | `docs/assets/exploration/RUN-20260527-001_rviz_next_ne.png`, 포트폴리오 |

## 판정

```text
READY_TO_EXECUTE_NO_ROBOT_MOTION
```

## 실행 후 기록할 결과

- RViz에 보인 cell 방향:
- initial next cell:
- 상태 전환 후 next cell:
- display-only identity TF로 `No tf data` 해소 여부:
- 실제 overview image와의 대응 판정: `PASS / FAIL`
- screenshot 경로:
- 발견한 문제:

## 다음 행동

1. RViz/physical 대응이 통과하면 evidence asset과 run 결과를 갱신합니다.
2. 그 후 `cultivation_panel`과 `base_link` 사이 transform을 어떤 측정으로
   정의할지 결정합니다.
3. transform이 없는 상태에서 scan pose나 robot motion을 실행하지 않습니다.
