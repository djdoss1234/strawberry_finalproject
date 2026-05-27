# 현재 진행 상태 및 다음 세션 Handoff

최종 갱신일: 2026-05-27

이 문서는 새 세션에서 가장 먼저 읽을 압축 상태 요약입니다. 상세 근거는
연결된 run, issue, config, evidence 문서를 확인합니다.

## 1. 프로젝트 역할

- 담당자: djdoss1234
- 담당 범위: 전체 딸기 수확 motion system
  - workspace 탐색/scan motion
  - target validation
  - approach, grasp, retreat, transfer, place
  - planner/executor, collision/retry, 평가
- 팀원 범위: 복잡 장면의 VLA 기반 수확 판단
- 통합 원칙: VLA proposal을 motion 측이 geometry/collision 검증 후 실행

## 2. 현재 선택한 개발 순서

1. Quadtree 기반 작업영역 탐색 환경
2. Overview/cell scan pose 생성 및 실제 motion 연결
3. 미니프로젝트 motion baseline 이식
4. Detector 결과와 cell state 연결
5. Tray localization 및 자동 place
6. Collision/retry/planner 비교
7. VLA 통합

## 3. 완료된 구현

- `strawberry_motion` ROS 2 `ament_python` package 구성
- quadtree cell/state pure Python core
- `workspace_marker_node` ROS visualization interface
- publish/subscribe interface:

```text
/strawberry/exploration/set_cell_state
  -> /workspace_marker_node
  -> /strawberry/exploration/workspace_cells
  -> /strawberry/exploration/next_cell
```

- 초기 `root/nw`, 상태 update 후 `root/ne` 진행 검증
- 실측 외곽 geometry와 테이프 교차점 기반 root split 지원
- overview 정렬용 `camera_alignment_node` 구현
  - 입력: `/camera/camera/color/image_raw`
  - 출력: `/strawberry/alignment/overlay_image`
  - 중앙 십자선, 전체 axis, 여백 guide 표시
  - `cv_bridge` 없이 image buffer 직접 처리
- 실시간 수동 정렬 표시용 `realsense_alignment_viewer` 구현
  - `pyrealsense2`로 color stream 직접 display
  - ROS image relay와 `rqt_image_view` 경로를 우회
  - 화면에 `LIVE FPS` 표시
  - camera 표시 전용으로 제한

검증:

- unit test 10개 통과
- `colcon build --packages-select strawberry_motion --symlink-install` 통과
- `workspace_marker_node` 실행 및 `root/nw` publish 확인
- `~/doosan_ws/install/strawberry_motion/share/strawberry_motion/config/workspace.yaml`
  설치본에서 실측 bounds, `root_split_m`, tape 구조 metadata 반영 확인
- build 중 로컬 `vcs_versioning` warning은 남지만 package build 실패는 아님
- unsafe robot-control option 거부 테스트를 포함해 unit test `15개` 통과
- `realsense2_camera`, `rqt_image_view` package 설치 확인
- `camera_alignment_node` ROS interface와 synthetic image overlay publish 확인
- DART 수동 joint 조작과 display-only viewer로 overview pose 정렬 완료
- overview 정렬 screenshot에서 네 cell, 중앙 교차점, `LIVE 30.0 FPS` 확인
- RViz cell/physical direction 검증용 `workspace_rviz.launch.py`와
  `rviz/workspace_exploration.rviz` 추가 준비
- RViz standalone 표시에서 발생한 `No tf data`에 대응해
  `workspace_visualization_world -> cultivation_panel` identity TF를
  visualization launch에서만 publish하도록 보완
- RViz marker topic 설정 오류를 수정하고 cell/axis 방향 대응을 사용자
  현장 확인 및 screenshot evidence로 `PASS` 판정
- 기존 hand-eye calibration과 aligned depth로 `base_link -> cultivation_panel`
  TF candidate를 계산하고 RViz에 적용
- registered panel/cell이 `base_link` 기준으로 표시되는 RViz evidence 확보
- scan pose preview 최초 표시에서 arrow 방향 의미 오류를 발견하고,
  camera candidate에서 cell center를 향하도록 수정

## 4. 물리 Workspace 현재 사실

구성:

- 종이 네 장을 절연테이프로 연결해 4분할 탐색판 제작
- 큰 탐색판을 다시 절연테이프로 화이트보드에 부착
- 종이 면: target을 배치할 usable 영역
- 중앙/외곽 테이프: 약 `20 mm`, 경계/dead zone으로 사용
- 외곽/중앙 테이프 및 `NW/NE/SW/SE`가 보이는 physical workspace
  사진 확보, 공개 사본 metadata 제거 완료

측정값:

| 항목 | 값 |
| --- | ---: |
| 화이트보드 | `1500 x 900 x 20 mm` |
| 종이판 outer workspace | `1100 x 800 mm` |
| 왼쪽 외곽 -> 중앙 세로선 | `545 mm` |
| 위쪽 외곽 -> 중앙 가로선 | `395 mm` |
| 보드 왼쪽 -> 종이판 왼쪽 | `182 mm` |
| 보드 위 -> 종이판 위 | `80 mm` |
| 보드 오른쪽 -> 종이판 오른쪽 | `228 mm` |
| 보드 아래 -> 종이판 아래 | `25 mm` |
| 서쪽 종이 usable 폭 | `515 mm` |
| 동쪽 종이 usable 폭 | `520 mm` |
| 종이 usable 높이 | `365 mm` |
| 절연테이프 폭 | `약 20 mm` |

적용한 frame:

```text
whiteboard_frame
  -> cultivation_panel  # 테이프 교차점 원점
```

적용한 bounds:

```text
x = -0.545 ~ +0.555 m
y = -0.405 ~ +0.395 m
root split = (0.0, 0.0)
```

테이프 해석:

- 방향별 tape band 3개의 예상 점유폭은 약 `60 mm`입니다.
- usable cell 합계와 outer workspace의 차이는 가로 `65 mm`, 세로
  `70 mm`로, 잔차 `5/10 mm` 수준에서 이 구조와 일관됩니다.
- 근사 tape 폭은 dead-zone 판단에만 쓰며, 정밀 motion safety margin에
  자동 적용하지 않습니다.

## 5. 아직 확정하지 않은 것

- motion safety margin용 경계별 tape overlap 정밀 치수
- `cultivation_panel` TF의 물리 위치 오차 정량 검증
- camera stand-off distance와 orientation
- RViz physical alignment 캡처
- 실제 `scan_pose_generator`와 robot scan motion
- joint-limit 사고 당시 alarm/recovery 상세 log는 미기록이며, 이후
  DART 수동 조작으로 overview 정렬이 가능한 상태는 확인

## 6. 현재 필요한 사용자 입력/현장 작업

1. 수정된 RViz scan pose preview에서 노란 camera 위치와 cell 방향을 확인
2. motion margin 구현 시 필요한 tape overlap 폭만 정밀 재측정

자료 경로:

```text
docs/assets/exploration/RUN-20260526-002_workspace_board.jpg  # 확보 완료
docs/assets/exploration/RUN-20260526-002_overview_camera.png  # 정렬 완료 화면
docs/assets/exploration/RUN-20260526-003_crosshair_overlay_preview.jpg  # 기능 preview
artifacts/RUN-20260526-002/raw/rviz_physical_alignment.png
artifacts/RUN-20260527-001/raw/rviz_cells.png
artifacts/RUN-20260527-001/raw/rviz_next_ne.png
docs/assets/exploration/RUN-20260527-001_rviz_cells.png  # 확보 완료
```

확보한 overview reference pose:

```yaml
joint_deg: [97.84, -94.40, 65.95, -10.93, 95.49, -94.79]
tcp_base_mm_deg: [85.91, -385.37, 569.66, 86.05, 67.57, -89.18]
```

상세 저장 위치: `config/recorded_poses.yaml`

이 pose는 여섯 joint가 현재 cuRobo 운용 제한 안에 들어온 중앙 정렬
reference이다. 단, 아직 motion safety validation 전이므로 자동
scan/motion 입력으로 사용하지 않고 registration 관측에만 사용한다.

## 7. 다음 구현

현장 입력을 받은 뒤:

1. `ISSUE-20260526-006` safety incident 후속 및 automated motion safety validation 설계
2. 기존 hand-eye calibration과 depth 기반 `base_link -> cultivation_panel` 후보 검증
3. RViz에서 registered frame 기준 scan pose preview 방향 검증
4. tape dead-zone/margin 설정 추가 여부 결정
5. safety-checked `scan_pose_generator.py` 구현
6. cell center observation pose를 RViz에서 먼저 검증

## 8. 핵심 문서와 Git

- geometry/config: `config/workspace.yaml`
- registration candidate: `config/panel_registration.yaml`
- reference poses: `config/recorded_poses.yaml`
- testbed: `docs/testbed_setup.md`
- 현재 run: `docs/runs/RUN-20260526-002_workspace_overview_alignment_plan.md`
- 다음 run: `docs/runs/RUN-20260527-001_rviz_physical_workspace_alignment.md`
- registration run: `docs/runs/RUN-20260527-002_panel_frame_registration_plan.md`
- 측정 issue: `docs/issues/ISSUE-20260526-003_workspace_measurement_boundary_mismatch.md`
- evidence: `docs/portfolio_evidence.md`
- baseline: https://github.com/djdoss1234/strawberry_miniproject
- final repo: https://github.com/djdoss1234/strawberry_finalproject

최근 완료 commit:

```text
41fb854 feat: apply measured workspace geometry and root split
7c5a87b docs: clarify tape-based workspace construction
8dba0f2 docs: record installed workspace config verification
```
