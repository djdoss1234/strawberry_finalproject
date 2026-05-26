# 현재 진행 상태 및 다음 세션 Handoff

최종 갱신일: 2026-05-26

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
- 실시간 jog 정렬용 `realsense_alignment_viewer` 구현
  - `pyrealsense2`로 color stream 직접 display
  - ROS image relay와 `rqt_image_view` 경로를 우회
  - 화면에 `LIVE FPS` 표시

검증:

- unit test 10개 통과
- `colcon build --packages-select strawberry_motion --symlink-install` 통과
- `workspace_marker_node` 실행 및 `root/nw` publish 확인
- `~/doosan_ws/install/strawberry_motion/share/strawberry_motion/config/workspace.yaml`
  설치본에서 실측 bounds, `root_split_m`, tape 구조 metadata 반영 확인
- build 중 로컬 `vcs_versioning` warning은 남지만 package build 실패는 아님
- overlay/direct viewer 단위 테스트를 포함해 unit test `14개` 통과
- `realsense2_camera`, `rqt_image_view` package 설치 확인
- `camera_alignment_node` ROS interface와 synthetic image overlay publish 확인

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
- 전체가 보이는 eye-in-hand overview camera pose
- `cultivation_panel`과 실제 robot `base_link` 사이 transform
- camera stand-off distance와 orientation
- RViz physical alignment 캡처
- 실제 `scan_pose_generator`와 robot scan motion
- 실제 eye-in-hand stream에서 십자선과 테이프 교차점 정렬 캡처
- direct viewer에서 실제 조그 정렬 시 체감 latency/FPS 확인

## 6. 현재 필요한 사용자 입력/현장 작업

1. `ros2 run strawberry_motion realsense_alignment_viewer` 실행
2. 로봇 카메라를 전체 종이판이 보이는 overview pose로 이동
3. direct viewer에서 노란 십자선을 중앙 tape crossing에 일치
4. 정렬 화면 캡처
5. 가능하면 해당 자세의 joint/TCP pose 저장
6. RViz에서 물리 cell과 marker의 의미가 맞는지 캡처
7. motion margin 구현 시 필요한 tape overlap 폭만 정밀 재측정

자료 경로:

```text
docs/assets/exploration/RUN-20260526-002_workspace_board.jpg  # 확보 완료
docs/assets/exploration/RUN-20260526-003_crosshair_overlay_preview.jpg  # 기능 preview
artifacts/RUN-20260526-002/raw/overview_camera.png
artifacts/RUN-20260526-002/raw/rviz_physical_alignment.png
```

## 7. 다음 구현

현장 입력을 받은 뒤:

1. 저지연 십자선 viewer 실행/캡처 및 overview pose 저장
2. tape dead-zone/margin 설정 추가 여부 결정
3. RViz에서 실측 cell alignment 확인
4. `scan_pose_generator.py` 구현
5. cell center에 대한 observation pose marker publish
6. 저속 실제 scan motion 검증

## 8. 핵심 문서와 Git

- geometry/config: `config/workspace.yaml`
- testbed: `docs/testbed_setup.md`
- 현재 run: `docs/runs/RUN-20260526-002_workspace_overview_alignment_plan.md`
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
