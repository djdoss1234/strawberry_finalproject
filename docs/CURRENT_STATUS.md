# 현재 진행 상태 및 다음 세션 Handoff

최종 갱신일: 2026-05-27 (단일 셀 실기 완료, 4셀 순회 J6 wind-up 수정, SW=overview 문제 발견)

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
- 수정된 scan pose preview의 네 cell 방향 표시를 RViz 캡처로 확인
- cell camera pose를 hand-eye calibration 기준 TCP target으로 변환하는
  geometry-only exporter 구현 및 후보 config 생성
- CUDA 재점검에서 `torch.cuda.is_available() == True`(GPU 1개)로 변경 확인
- `scripts/validate_scan_poses.py` 구현 후 cuRobo `MotionGen` dry-run 실행 완료
- 상세 결과: `docs/runs/RUN-20260527-004_curobo_dry_run.yaml`
- standoff/orientation 격자 탐색으로 SW 전용 포즈 확보:
  - NW: panel-normal standoff 0.65m → PLAN_VALID
  - SW: base -Y 방향 d=0.40m → PLAN_VALID (유효범위 0.30–0.42m)
    panel-normal approach는 x를 -0.41m로 밀어 IK_FAIL; base -Y는 x=-0.147m 유지
- **scan_pose_candidates.yaml v6**: cuRobo ee link(`gripper_rh_p12_rn_base`) 기준
  orientation frame 오류를 수정한 4셀 후보
  - NW/NE/SW: panel-normal `0.65 m`, SE: base-neg-Y `0.40 m`
  - empty-world cuRobo dry-run에서는 전부 `PLAN_VALID`
  - collision sphere와 panel/tray world가 없는 검증이므로 실기 실행 허가는 아님
- VLA 인터페이스 구현 완료 (`src/strawberry_motion/interfaces/approach_proposal.py`):
  - `ApproachDirection`: FRONT/LEFT/RIGHT/UPPER_LEFT/UPPER_RIGHT/REOBSERVE/SKIP/RECOVER_HOME
  - `validate_approach_proposal()`: proposal → MotionValidationResult, no robot motion
  - offline table에 v6 dry-run 결과 반영, 전체 4셀 FRONT = VALID
  - offline `VALID`는 검증 범위 내 결과일 뿐 `is_executable=False`
  - fail-closed gate 테스트 추가 후 전체 테스트 44개 통과
- `scan_executor_node` 초안의 자동 실행 위험을 발견해 fail-closed 구조로 수정
  - 기본 launch는 RViz/preview only
  - executor opt-in 및 `/strawberry/scan/start` 명시 호출 필요
  - collision-aware backend가 검증되기 전에는 실제 motion을 강제 거부
  - detector 결과 전에는 `SCANNED_EMPTY` 대신 `SCAN_POSE_REACHED` 사용
- `config/scan_collision_world.yaml`과
  `scripts/validate_v6_collision_scan_poses.py` 추가
  - 미니프로젝트 robot/tool collision sphere 모델 재사용
  - 현재 `cultivation_panel` 등록값에서 생성한 whiteboard cuboid 활성화
  - `v6` 네 cell 모두 registered-whiteboard dry-run에서 `PLAN_VALID`
  - 결과: `docs/runs/RUN-20260527-007_registered_whiteboard_collision_dryrun.yaml`
  - self-collision, table/tray/cable/human obstacle와 registration 오차는
    아직 미검증이므로 자동 motion은 계속 금지
- panel TF 물리 오차 정량화를 위한 read-only landmark capture/evaluation
  도구 추가 준비
  - 측정점: 중앙 교차점 + outer NW/NE/SW/SE
  - 판정: `RMS <= 10 mm`, `MAX <= 15 mm`는 registration 측정 통과 기준만 의미
  - 절차: `docs/runs/RUN-20260527-008_panel_registration_landmark_validation_plan.md`
  - 1차 캡처 결과: 중앙점 `2.663 mm` 오차이나 외곽점은 검은 테이프
    depth 오측정으로 `131~139 mm` 오차, `RECAPTURE_REQUIRED`
  - raw screenshot은 배경 인물이 있어 Git 공개 asset으로 추가하지 않음
  - retry 과정에서 흰 종이 클릭과 outer tape 좌표를 비교하던 evaluator
    기준 오류를 발견하고 white-paper inner-corner 기준으로 수정
  - refit 후보: `RMS=9.229 mm`, `MAX=10.981 mm`, plane offset max
    `4.125 mm`, `MEASURED_PASS_PENDING_MOTION_MARGIN`
  - RViz 표시 확인 후 refit TF를 시각화/오프라인 검증 기준으로 승격
  - 확인 화면(local): `/home/user/Pictures/Screenshot from 2026-05-27 16-49-14.png`
  - 새 TF로 재생성한 v7 scan collision dry-run 결과:
    `NW/NE/SE=PLAN_VALID`, `SW=IK_FAIL`
  - 결과: `docs/runs/RUN-20260527-010_refit_panel_collision_dryrun.yaml`
  - 이는 자동 motion 허가가 아니며 실행 잠금은 유지
- 첫 실기 scan은 전체 순회가 아니라 `root/ne` 또는 `root/nw` 중 명시한
  단일 cell만 가능하도록 executor gate를 제한
  - 절차: `docs/runs/RUN-20260527-009_single_cell_scan_safety_plan.md`
- `root/sw` refit IK_FAIL 대체 탐색 (`scripts/search_sw_candidates.py`)
  - panel_normal 0.55 m 채택: J5 max 129.6 deg (운용 제한 130 deg 이내)
  - 결과: `docs/runs/RUN-20260527-011_sw_candidate_search.yaml`
- self-collision sphere 검토 및 SE 정책 변경
  - `self_collision_check=True` + refit whiteboard에서 NW/NE/SW PLAN_VALID 확인
  - SE `base_neg_y 0.40 m`은 J6 비결정성 문제로 `panel_normal 0.65 m`으로 변경
  - 최종 정책: NW/NE/SE = panel_normal 0.65 m, SW = panel_normal 0.55 m
  - 4셀 전체 `self_collision_check=True` PLAN_VALID
  - `config/scan_pose_candidates_refit_candidate.yaml` 최종 갱신
  - 결과: `docs/runs/RUN-20260527-012_self_collision_4cell_dryrun.yaml`
  - `use_for_automated_motion: false` 유지
- executor runtime collision backend 완성
  - `_init_motion_gen()`: robot spheres + whiteboard cuboid + self_collision_check
  - `_COLLISION_BACKEND_READY_FOR_MOTION = True`
  - candidates 파일 → `scan_pose_candidates_refit_candidate.yaml`
  - `collision_world_validated_for_motion: true` (YAML 갱신)
  - colcon build + 테스트 52개 통과
  - abort/recovery 절차: `docs/runs/RUN-20260527-013_single_cell_abort_recovery_plan.md`

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
- 실제 robot scan motion: v6 후보는 있으나 collision-aware 검증 전 실행 금지
- joint-limit 사고 당시 alarm/recovery 상세 log는 미기록이며, 이후
  DART 수동 조작으로 overview 정렬이 가능한 상태는 확인

## 6. 현재 필요한 사용자 입력/현장 작업

1. motion margin 구현 시 필요한 tape overlap 폭만 정밀 재측정
2. panel_registration TF 물리 위치 오차 정량 검증 (실제 marker 측정)

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

현재 상태: NE/NW 단일 셀 실기 완료, 4셀 순회 실행 중 두 가지 미해결 이슈 발견.

**다음 세션 우선 해결 항목:**

| 번호 | 항목 | 내용 |
| --- | --- | --- |
| A | SW = overview 문제 | overview FK TCP 계산 → SW TCP와 비교 → SW 스캔 후보 재탐색(standoff ≤ 0.40m) |
| B | SE J1 114° 스윙 | IK seed 또는 standoff 조정으로 J1 스윙 범위 축소, 경로 부드럽게 |
| C | YOLO detector 물리 연동 | pick_pose 토픽 연동 + 딸기 배치 후 TARGET_FOUND 확인 |

이후 순서:

1. ~~refit 이후 `IK_FAIL`이 된 `root/sw`의 standoff/approach 후보를 재탐색~~ **완료**
2. ~~self-collision sphere false positive 원인을 검토하고, scan용으로
   신뢰할 수 있는 robot/tool collision profile을 확정~~ **완료**
3. ~~executor runtime collision backend 연결~~ **완료**
4. ~~단일 셀 실기 실행 (NE, NW)~~ **완료**
5. ~~4셀 순회 J6 wind-up 수정 (inter-cell overview reset)~~ **완료**
6. SW 스캔 후보 재탐색 (**진행 중**)
7. SE IK 경로 안정화 (**진행 중**)
8. detector 결과와 cell state 연결 (YOLO → SCANNED_FOUND/EMPTY)
9. panel registration 오차를 motion margin에 반영
10. tray localization 및 자동 place (미니프로젝트 baseline 이식)

## 8. 핵심 문서와 Git

- geometry/config: `config/workspace.yaml`
- registration candidate: `config/panel_registration.yaml`
- scan TCP candidates: `config/scan_pose_candidates.yaml`
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
019031b feat: SW dedicated scan pose via base-neg-Y approach
d157bf3 feat: SW alternative pose via left_column_center
a684fb5 feat: VLA approach interface + corrected scan pose candidates (v2)
c3a33c9 feat: cuRobo dry-run validation for scan pose candidates
```
