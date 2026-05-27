# 딸기 수확 로봇 최종 프로젝트 (Strawberry Final Project)

실제에 가까운 딸기 모형 환경에서 반복 가능한 수확 동작을 구현하기 위한
딸기 수확 로봇 최종 프로젝트 저장소입니다.

이 프로젝트는 미니프로젝트에서 검증한 `RealSense + YOLO + cuRobo +
Doosan E0509` 기반 pick & place pipeline을 출발점으로 삼아, 변화하는
환경에서도 측정 가능하고 확장 가능한 수확 시스템으로 고도화합니다.

## 프로젝트 역할 분담

| 영역 | 담당 | 범위 |
| --- | --- | --- |
| 수확 모션 시스템 | djdoss1234 | approach, grasp, retreat, transfer, tray placement, planning/execution 통합, 실패 복구, 성능 평가 |
| 복잡한 환경의 VLA 수확 판단 | 팀원 | 가림/복잡 장면 해석, 수확 대상 판단, high-level action 제안 |
| 통합 실험 | 공동 | target/action interface 합의, 실험 protocol, end-to-end demo |

핵심 원칙은 **VLA가 로봇 trajectory를 직접 실행하지 않는 것**입니다.
VLA는 수확할 대상이나 다음 행동을 제안할 수 있지만, 실제 하드웨어 명령
전에는 모션 시스템이 target pose, workspace, collision, 실행 가능성을
검증합니다.

## 이전 미니프로젝트 기준 구현

기존 구현과 실험 기록은 다음 저장소에 정리되어 있습니다.

- [strawberry_miniproject](https://github.com/djdoss1234/strawberry_miniproject)

미니프로젝트에서 구현하거나 검증한 기반 기능:

- RGB-D 기반 딸기 후보 검출과 3D target 생성
- eye-in-hand coordinate transform
- cuRobo 기반 approach, grasp, retreat planning
- Doosan 로봇 실행과 gripper 제어
- 티칭된 tray slot으로의 place 동작
- 실험 결과, 문제점, 개선 방향 문서화

최종 프로젝트에서는 실험용 코드 전체를 바로 섞지 않고, 안정화된 기능을
필요한 순서대로 옮기며 모듈 구조와 평가 체계를 새로 잡습니다.

## 내 담당 범위: Motion System

이 저장소에서 우선 구현하고 검증할 내용:

- 실제형 딸기 모형에 대한 안정적인 수확 motion sequence
- planning, execution, task sequence 모듈화
- tray 위치 인식과 자동 place target 생성
- 딸기, tray, 장애물, 이미 배치된 과실을 고려한 collision world 관리
- 실패 유형별 retry/recovery policy
- planner 비교 및 trajectory 품질 분석
- 팀원의 VLA 판단 결과를 수신하는 integration interface

## 첫 기능 목표

첫 번째 기능 milestone은 **quadtree 기반 작업영역 탐색과 scan motion
생성**입니다.

> 재배 작업영역을 `workspace_frame` 기준 quadtree cell로 관리하고,
> 아직 관찰하지 않았거나 재관찰이 필요한 영역을 보기 위한 eye-in-hand
> camera scan pose를 생성해 motion system과 연결한다.

이 기능을 먼저 구현하는 이유:

- 카메라에 이미 보이는 딸기만 잡던 구조에서 작업영역을 탐색하는 구조로 확장할 수 있음
- 본인의 motion 담당 범위인 scan pose 생성 및 이동 실행과 직접 연결됨
- VLA와 역할이 겹치지 않게 `어디를 볼지`를 관리하는 계층을 만들 수 있음
- 이후 target detection, tray place, VLA 결과를 동일 상태 map에 연결할 수 있음

두 번째 기능 milestone은 **움직인 계란판에 대한 자동 place**입니다.
AprilTag/ArUco 또는 RGB-D 기반으로 tray pose를 다시 인식하고, 자동
생성한 빈 slot에 딸기 모형을 배치하여 수확 후반 motion까지 확장합니다.

## 초기 진행 순서

1. `workspace_frame`, quadtree cell 상태, 탐색/재관찰 정책을 정의합니다.
2. RViz visualization과 cell 중심 기반 scan pose 생성을 구현합니다.
3. 미니프로젝트에서 안정적으로 동작한 motion 기능을 선별해 이식합니다.
4. quadtree scan motion과 target detection/result 상태 갱신을 연결합니다.
5. `tray_frame` 기준 slot 자동 생성과 자동 place 기능을 구현합니다.
6. planner/collision/retry 개선 후 VLA의 high-level 제안을 연결합니다.

세부 역할, interface 방향, 평가 기준, 첫 sprint는
[docs/project_scope.md](docs/project_scope.md)에 기록합니다.
전체 개발 순서와 단계별 산출물/완료 기준은
[docs/development_roadmap.md](docs/development_roadmap.md)에 기록합니다.
모듈화, `rqt_graph`, Git 진행 기록 방식은
[docs/development_workflow.md](docs/development_workflow.md)에 기록합니다.
실행 단계별 결과, 문제 해결, 포트폴리오/자기소개서 근거 축적 방식은
[docs/project_recording_system.md](docs/project_recording_system.md)에 기록합니다.
사진, 영상, `rqt_graph`, RViz 캡처를 어느 단계에 어떤 이름으로 남길지는
[docs/visual_asset_guide.md](docs/visual_asset_guide.md)에 기록합니다.
현재 물리 테스트베드의 실측값과 frame 기준은
[docs/testbed_setup.md](docs/testbed_setup.md)에 기록합니다.
세션이 끊긴 뒤 빠르게 이어가기 위한 최신 상태 요약은
[docs/CURRENT_STATUS.md](docs/CURRENT_STATUS.md)에 유지합니다.
현재까지 검증된 대외 설명 재료는
[docs/portfolio_evidence.md](docs/portfolio_evidence.md)에 누적합니다.

## 현재 상태

현재는 최종 프로젝트의 시작 단계로, 역할 분담과 motion 중심 개발 범위,
첫 milestone, 공개 저장소 관리 기준을 정의했습니다. quadtree workspace
core와 첫 ROS 2 visualization node를 구현하여 cell 상태와 다음 관찰
대상을 topic/RViz marker로 내보내는 단계까지 진행했습니다. 실제 robot
motion baseline은 scan pose 정의와 함께 선별적으로 추가합니다.

현재 quadtree 탐색에 사용할 물리 workspace:

![종이 4분할 physical workspace](docs/assets/exploration/RUN-20260526-002_workspace_board.jpg)

종이 네 장과 약 `20 mm` 절연테이프 경계를 이용해 1차 cell
`NW/NE/SW/SE`를 구성했습니다. eye-in-hand camera에서 전체 영역이
보이는 overview pose를 확보했으며, 다음 검증은 이를 RViz marker와
대응시키는 단계입니다.

### Camera 중앙 십자선 정렬

overview pose를 현장에서 맞출 때는 저지연 `realsense_alignment_viewer`를
사용합니다. RGB stream 위의 노란 십자선을 종이 중앙 테이프 교차점에
맞추면 `cultivation_panel` 원점을 camera view 중심에 일치시킬 수 있습니다.

![Camera crosshair overlay preview](docs/assets/exploration/RUN-20260526-003_crosshair_overlay_preview.jpg)

위 이미지는 renderer 설명용 preview입니다. 실제 종이판을 camera 중앙에
맞춘 overview reference pose도 확보했습니다.

![실제 overview camera 정렬 결과](docs/assets/exploration/RUN-20260526-002_overview_camera.png)

정렬 순간의 joint/TCP pose는 `config/recorded_poses.yaml`에 저장되어
있으며, 이 값은 자동 motion command가 아니라 RViz/TF 검증을 위한
reference입니다.

실시간 camera 표시용 실행:

```bash
cd ~/doosan_ws
source install/setup.bash
ros2 run strawberry_motion realsense_alignment_viewer
```

이 경로는 RealSense color stream을 직접 열어 OpenCV 화면에 십자선을
표시하므로, ROS image relay와 `rqt_image_view`를 거치는 경로보다
manual alignment에 적합합니다. 화면 하단의 `LIVE FPS`로 체감 상태를
함께 확인합니다. 종료는 `q` 또는 `ESC`입니다.

중요: viewer의 robot Cartesian step control은 실제 로봇에서 joint
limit 사고가 발생해 철회했습니다. `--enable-robot-control` 옵션은
이제 안전 검증이 구현될 때까지 실행을 거부합니다. viewer는 camera
십자선 표시 전용으로만 사용합니다.

ROS topic 연결과 `rqt_graph` 검증이 필요할 때만 아래 경로를 사용합니다.

```bash
ros2 launch realsense2_camera rs_launch.py rgb_camera.color_profile:=640x480x30
ros2 launch strawberry_motion camera_alignment.launch.py
ros2 run rqt_image_view rqt_image_view /strawberry/alignment/overlay_image
```

`pyrealsense2`로 카메라를 직접 여는 기존 perception node와
`realsense_alignment_viewer` 또는 `realsense2_camera`는 같은 시점에
실행하지 않습니다. alignment pose를 먼저 확보하고 종료한 뒤
detection/pick 실험으로 전환합니다.

현재 ROS exploration interface:

| Topic | Type | 역할 |
| --- | --- | --- |
| `/strawberry/exploration/workspace_cells` | `visualization_msgs/msg/MarkerArray` | RViz용 workspace cell 및 상태 표시 |
| `/strawberry/exploration/next_cell` | `std_msgs/msg/String` | 다음 관찰 대상 cell ID publish |
| `/strawberry/exploration/set_cell_state` | `std_msgs/msg/String` | `cell_id=STATE` 형식의 상태 갱신 입력 |
| `/strawberry/alignment/overlay_image` | `sensor_msgs/msg/Image` | overview pose 정렬용 중앙 십자선 RGB 화면 |

초기 visualization node 실행:

```bash
cd ~/doosan_ws
colcon build --packages-select strawberry_motion --symlink-install
source install/setup.bash
ros2 launch strawberry_motion workspace_visualization.launch.py
```

RViz에서는 fixed frame을 현재 임시 기준 frame인 `cultivation_panel`로
설정하고, `MarkerArray` display에
`/strawberry/exploration/workspace_cells`를 선택합니다. 실물 테스트베드
기준 frame이 확정되면 이 frame 이름과 좌표는 실측값으로 교체합니다.

RViz marker 검증과 캡처를 바로 시작할 때는 저장된 화면 구성까지 함께
실행합니다. 이 launch는 robot motion을 호출하지 않습니다.

```bash
source ~/doosan_ws/install/setup.bash
ros2 launch strawberry_motion workspace_rviz.launch.py
```

이 launch는 RViz 표시를 위해
`workspace_visualization_world -> cultivation_panel`의 identity TF를 함께
publish합니다. 실제 `base_link -> cultivation_panel` 측정 transform은
아직 정의되지 않았으며, 이 표시용 TF를 robot motion에 사용하지 않습니다.

상태 갱신 확인 예시:

```bash
ros2 topic pub --once /strawberry/exploration/set_cell_state \
  std_msgs/msg/String "{data: 'root/nw=SCANNED_EMPTY'}"
ros2 topic echo --once /strawberry/exploration/next_cell
```

## 데이터 및 안전 관리

다음 항목은 별도 승인 없이 공개 저장소에 commit하지 않습니다.

- 로봇/카메라 calibration 파일
- 배포 여부가 정해지지 않은 model weight
- raw camera log, rosbag, 실험 영상
- credential, token, 장비별 네트워크 설정

실제 로봇에서 motion 변경을 시험할 때는 속도를 제한하고, collision scene과
실험 조건을 기록한 뒤 pick & place 전체 동작을 실행합니다.

## 다음 세션에서 이어가는 방법

새 대화나 새 개발 환경에서는 이전 채팅 내용을 자동으로 기억하는 방식이
아닙니다. 대신 다음 순서로 저장소 기록을 읽으면 같은 맥락에서 작업을
이어갈 수 있습니다.

1. 이 `README.md`에서 목표와 현재 상태를 확인합니다.
2. `docs/CURRENT_STATUS.md`에서 마지막 완료 상태와 현장 입력 대기 항목을 확인합니다.
3. `AGENTS.md`에서 담당 범위, 설계 원칙, 작업 순서를 확인합니다.
4. `docs/project_scope.md`에서 milestone과 interface/평가 계획을 확인합니다.
5. `docs/development_roadmap.md`에서 전체 순서와 현재 완료할 단계를 확인합니다.
6. `docs/project_recording_system.md`, 최근 `worklogs/runs/issues`를 확인합니다.
7. `docs/portfolio_evidence.md`에서 검증된 성과와 아직 계획인 기능을 구분합니다.
8. `git log`와 issue/commit 기록으로 마지막 실제 변경 내용을 확인합니다.
9. 구현을 옮길 때는 `strawberry_miniproject`를 baseline reference로 확인합니다.

즉, 중요한 결정과 진행 결과를 문서와 commit으로 계속 남기는 것이 이
프로젝트의 기억 장치입니다.
