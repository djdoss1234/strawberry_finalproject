# RUN-20260526-002: 4분할 Workspace와 Overview Camera Pose 정렬 계획

## 상태

```text
IN_PROGRESS
```

## 목적

종이 4장을 테이프로 나눈 물리 workspace를 quadtree root의 1차 자식
cell 네 개와 대응시키고, 전체 영역이 보이는 기준 camera pose를 확보합니다.

이 run에서는 실제 수확 motion을 수행하지 않습니다. 먼저 실제 작업영역의
크기, 방향, camera view, cell 대응이 맞는지 검증합니다.

## 왜 전체가 보이는 각도가 먼저 필요한가

현재 구현한 quadtree는 전체 workspace를 기준으로 `root/nw`, `root/ne`,
`root/sw`, `root/se`를 관리합니다. 따라서 시작 시점에는 최소 한 번,
실제 종이 4개 전체가 카메라 영상 안에 들어와야 다음을 확인할 수 있습니다.

- 코드의 cell 방향과 실제 종이 방향이 일치하는지
- 전체 영역 경계가 camera 시야에서 잘리지 않는지
- 이후 어느 cell을 더 가까이 관찰해야 할지 정할 수 있는지

다만 전체가 보이는 pose는 **overview/registration용**입니다. 딸기 검출,
depth 정밀도, 실제 grasp 판단은 각 cell에 더 가까이 접근하는
`cell scan pose`가 필요할 수 있습니다.

## 권장 구조

```text
OVERVIEW_POSE
  -> 종이 4개 전체 확인
  -> root/nw, root/ne, root/sw, root/se 대응 검증
  -> next cell 선택

CELL_SCAN_POSE(root/nw)
  -> 해당 영역을 더 크게 관찰
  -> target 유무/상태 갱신
  -> 다음 cell 또는 pick motion으로 전환
```

## 지금 물리 환경에서 할 일

### 1. 종이 Workspace 고정

- 종이 4장이 하나의 직사각형 영역이 되도록 고정합니다.
- 테이프 교차점을 workspace 중심으로 사용합니다.
- 카메라 영상 기준 위쪽/아래쪽, 로봇 기준 좌/우가 헷갈리지 않도록
  네 영역에 임시 label을 붙입니다.

권장 label:

```text
NW | NE
---+---
SW | SE
```

label은 실제 final scene에 남길 필요는 없고, 좌표계 정렬 검증 동안만
사용합니다.

### 2. 실제 치수 측정

측정 완료된 값:

| 항목 | 측정값 |
| --- | --- |
| 화이트보드 전체/두께 | `1500 x 900 x 20 mm` |
| 전체 workspace 가로 길이 | `1100 mm` |
| 전체 workspace 세로 길이 | `800 mm` |
| 왼쪽 -> 세로 중앙선 | `545 mm` |
| 위쪽 -> 가로 중앙선 | `395 mm` |
| 로봇 base 기준 workspace 대략 위치 | 미측정 |
| 종이 면과 camera 사이 거리 | 미측정 |

`cultivation_panel` 원점은 테이프 교차점으로 유지합니다. 실측값에 따라
outer bounds는 `x=-0.545~+0.555 m`, `y=-0.405~+0.395 m`로
`config/workspace.yaml`에 반영했습니다. outer bounds의 중심과 원점이
각 축에서 `5 mm` 어긋나므로 root 분할은 명시적인 `(0, 0)`을 사용합니다.

측정 확인 필요:

- 개별 cell의 제공 치수(`515/520 mm`, 높이 `365 mm`) 합계가 outer
  dimension보다 가로 `65 mm`, 세로 `70 mm` 작습니다.
- 테이프/겹침/여백을 제외한 값인지 확인하기 전에는 scan target 계산에
  개별 cell 치수를 사용하지 않습니다.
- 관련 issue: `ISSUE-20260526-003`

### 2.1 Config 및 Node 반영 확인

- `config/workspace.yaml`에 whiteboard 크기, outer workspace bounds,
  테이프 교차점 `root_split_m: (0, 0)`을 반영했습니다.
- 기존 quadtree core는 outer bounds의 기하학적 중앙으로 분할했으나,
  실측 비대칭을 반영하기 위해 root의 물리 split point를 별도로 받도록
  수정했습니다.
- 단위 테스트 10개 통과
- `colcon build --packages-select strawberry_motion --symlink-install` 성공
- ROS node 실행 후 초기 `/strawberry/exploration/next_cell`이
  `root/nw`임을 확인했습니다.

### 3. Overview Camera Pose 만들기

조건:

- 네 종이 영역이 모두 영상 안에 들어와야 합니다.
- 바깥 경계가 영상 가장자리에 너무 붙지 않게 여백을 둡니다.
- 카메라는 가능하면 종이 면을 정면에 가깝게 바라보게 합니다.
- 심한 기울기나 원근 왜곡으로 한쪽 cell만 크게 보이는 pose는 피합니다.
- 로봇은 저속 또는 jog 방식으로 이동하며, 종이나 fixture와 충돌하지
  않는 거리에서 멈춥니다.

판정:

```text
PASS: 네 cell 전체와 중앙 교차점이 한 화면에서 명확히 식별됨
FAIL: 한 영역이 잘리거나, 심한 왜곡/반사/거리 문제로 구분이 어려움
```

### 4. Cell별 Scan Pose 필요성 판단

overview pose에서 딸기 모형을 한 cell에 놓고 확인합니다.

- 딸기 존재 여부만 충분히 보임: 탐색용 overview로 사용 가능
- 딸기 위치/깊이/가림 판단이 불안정함: cell별 close-up scan pose 필요

실전 프로젝트에서는 대부분 다음 구조가 유리합니다.

```text
overview pose: 전체 coverage와 영역 상태 관리
cell scan pose: 검출/깊이/수확 가능성 판단
pick pose: 실제 grasp 실행
```

## 실행 순서

1. 종이 4개에 `NW/NE/SW/SE` 임시 label을 붙입니다.
2. 전체 workspace 사진을 정면에서 촬영합니다.
3. 전체 가로/세로 길이를 측정합니다.
4. 로봇 camera를 전체 종이가 보이는 overview pose로 jog 이동합니다.
5. camera RGB 화면에서 전체 경계와 중앙 교차점이 보이는지 확인합니다.
6. 해당 화면을 캡처합니다.
7. RViz marker와 물리 cell 의미가 일치하는지 비교합니다.
8. 결과가 맞으면 다음 구현인 `scan_pose_generator`의 기준 pose로 사용합니다.

## 시각자료 계획 및 확보 상태

| 자료 | 필요 장면 | 상태 | 원본 위치 | 공개 위치/사용처 |
| --- | --- | --- | --- | --- |
| Physical workspace 사진 | 종이 4개, 테이프 중앙선, NW/NE/SW/SE label | `NOT_CAPTURED` | `artifacts/RUN-20260526-002/raw/workspace_board.jpg` | `docs/assets/exploration/RUN-20260526-002_workspace_board.jpg`, testbed 설명 |
| Overview camera 화면 | 네 cell 전체가 카메라 화면에 보이는 장면 | `NOT_CAPTURED` | `artifacts/RUN-20260526-002/raw/overview_camera.png` | `docs/assets/exploration/RUN-20260526-002_overview_camera.png`, 포트폴리오 |
| RViz/physical 대응 비교 | RViz quadtree와 실제 4분할 영역을 나란히 설명할 자료 | `NOT_CAPTURED` | `artifacts/RUN-20260526-002/raw/rviz_physical_alignment.png` | `docs/assets/exploration/RUN-20260526-002_rviz_physical_alignment.png`, GitHub/Notion |

<!-- VISUAL TODO
asset_id: RUN-20260526-002_workspace_board
capture: 종이 4분할 workspace의 전체 모습과 NW/NE/SW/SE 임시 label
source_path: artifacts/RUN-20260526-002/raw/workspace_board.jpg
public_path: docs/assets/exploration/RUN-20260526-002_workspace_board.jpg
use_in: testbed 설명, GitHub README, Notion Milestone
status: NOT_CAPTURED
-->

<!-- VISUAL TODO
asset_id: RUN-20260526-002_overview_camera
capture: eye-in-hand camera에서 네 종이 cell이 여백을 두고 모두 보이는 RGB 화면
source_path: artifacts/RUN-20260526-002/raw/overview_camera.png
public_path: docs/assets/exploration/RUN-20260526-002_overview_camera.png
use_in: quadtree exploration 설명, 포트폴리오, Notion Experiment Run
status: NOT_CAPTURED
-->

## 완료 기준

- [x] 종이 outer bounds와 테이프 교차점 root split을 config에 반영합니다.
- [ ] 종이 4분할 영역을 camera view에서 코드의 four child cell과 대응시킵니다.
- 전체 영역이 보이는 overview camera pose를 한 개 확보합니다.
- [x] 실제 workspace 가로/세로 및 whiteboard 치수를 기록합니다.
- overview pose의 RGB 화면을 확보합니다.
- `scan_pose_generator`에서 사용할 frame/orientation/stand-off를 정하기
  위한 입력 자료를 확보합니다.

## 완료 후 다음 작업

1. 개별 cell 치수의 측정 경계를 재확인
2. overview pose 또는 camera 기준 frame 정의
3. RViz 물리 정렬 자료 기록
4. `scan_pose_generator` 구현
5. cell별 close-up scan pose를 실제 로봇에서 순차 검증
