# 현재 진행 상태 — Handoff

최종 갱신일: 2026-06-02 (v12 그리퍼 중심 scan pose 순회 검증 + cuRobo branch optimization dry-run)

이 문서는 새 세션에서 가장 먼저 읽을 압축 상태 요약입니다.

---

## 1. 프로젝트 역할

- 담당자: djdoss1234
- 담당 범위: 전체 딸기 수확 motion system
  - workspace 탐색/scan motion (쿼드트리 기반 4셀 순회)
  - target validation, approach, grasp, retreat, transfer, place
  - planner/executor, collision/retry, 평가, KPI 측정
- 팀원 범위: VLA 기반 수확 판단 + YOLO 딸기/줄기 검출 모델 학습
- 통합 원칙: scan → YOLO 검출 → pickability 검증 → 룰베이스 pick → 실패 시 VLA 이관

---

## 2. 현재 상태 한 줄 요약

> **스캔→픽 통합 시퀀스 구현 완료. v12 그리퍼 중심 scan pose 4셀 재티칭 및 전체 순회 검증 완료. YOLO seg+pose 두 모델 fusion 계약 기록 완료.**
> 다음: 15.8cm 파지 파츠 기준 tool/collision model 재검증 + pose tolerance를 포함한 cuRobo branch/scan pose 최적화 + fusion node에서 `/strawberry/detection/pick_pose` 연결.
> 팀원 dashboard는 `tools/dashboard/`에 로컬 실행 래퍼로 추가됨.

---

## 3. 완료된 구현 (전체 아크)

### 3-A ~ 3-G. 쿼드트리 스캔 구조 완성 (2026-05-26~29)
→ 상세는 이전 버전 참조. v11 전체 자동 순회 활성화, 4셀 물리 검증 완료.

### 3-H. 스캔→픽 통합 시퀀스 (2026-06-01, commit 3b1240c)

**scan_executor_node.py 변경:**
- `_detection_poses: List[PoseStamped]` 추가 — dwell 중 포즈 버퍼링
- `_pick_complete_event: threading.Event` 추가
- detection 입력 토픽 분리:
  - `구독`: `/strawberry/detection/pick_pose` (YOLO → scan_executor)
  - `발행`: `/dsr01/curobo/pick_pose` (scan_executor → curobo_planner, 1개씩)
- `구독` 추가: `/dsr01/curobo/pick_complete` → pick 완료 대기
- `_deduplicate_poses`: 30mm 이내 중복 제거
- `_trigger_picks_for_cell`: TARGET_FOUND 후 포즈 순차 전달 + 120s timeout
- **셀 간 overview reset 제거** → HOME에서 다음 스캔 포즈 직행
- `enable_pick_integration` 파라미터 추가 (기본 True)

**curobo_planner_node.py 변경:**
- `vla_request_pub`: `/strawberry/vla/request` 발행 (파지 실패 시)

**harvest_session_logger.py 신규:**
- KPI 로거: success rate, avg cycle time 실시간 추적
- `docs/harvest_logs/session_YYYYMMDD_HHMMSS.yaml` 저장

### 3-I. 이웃 딸기 손상 최소화 (2026-06-01, commit 66360c8)

**curobo_planner_node.py 변경:**
- `Sphere` 장애물 등록: 이웃 딸기 위치를 cuRobo world에 sphere(r=30mm)로 등록
- `/strawberry/detection/scene_positions` 구독 (Float64MultiArray, flat [x,y,z, ...])
- `_register_neighbor_obstacles(target_pos)`: approach 전에 이웃 sphere 등록
- `_clear_neighbor_obstacles()`: pick 완료/abort 모든 경로에서 해제
- **retreat 방향 수정**: `ee_r += [0, 0, +0.05m]` — 위로 들어올리며 후퇴
- `collision_cache`에 `"sphere": 30` 추가

### 3-J. v12 그리퍼 중심 scan pose 검증 및 모션 최적화 준비 (2026-06-02)

**문제 배경:**
- 이전 카메라 중심 scan pose는 detection 시야는 좋았지만, 실제 수확에서는 그리퍼가 줄기/과실에 접근하기 어려웠다.
- 약 15.8cm 길이의 3D 프린터 출력 딸기 파지 파츠를 그리퍼에 장착하면서, camera optical center와 실제 파지 접근 중심의 차이가 더 중요해졌다.
- 따라서 각 셀을 “카메라 정면”이 아니라 “그리퍼가 셀 중심 작업 영역에 수평으로 접근 가능한 자세”로 다시 티칭했다.

**현재 실행 구조:**
- 대시보드/수동 조정: TCP Cartesian jog로 미세 조정 가능
- 자동 스캔 순회: YAML `endpoint_joints_deg`를 직접 `MoveJoint` 실행
- J1/J4/J6은 동일 물리 자세의 360deg equivalent 중 현재 관절에서 가장 가까운 값을 실행 직전에 선택
- 셀 간 overview reset 없이 direct traversal

**cuRobo 최적화 준비:**
- `scripts/optimize_scan_joint_branches.py` 추가
- 각 셀의 `tcp_transform_base`를 고정하고 cuRobo IK branch 후보를 생성
- scan order 전체에 대해 관절 이동량, wrist 회전량, 관절 한도 근접도, 수동 티칭 이탈량을 비용으로 계산
- dry-run 결과:
  - `docs/runs/RUN-20260602-001_curobo_scan_branch_optimization.yaml`
- 아직 적용하지 않음. exact TCP 고정만으로는 후보가 제한되고, 15.8cm 파지 파츠 기준 collision sphere 재검증이 필요하다.

**runtime cuRobo preview:**
- `enable_runtime_curobo_preview:=true`로 현재 joint state 기준 cuRobo plan을 로그 비교 가능
- preview는 진단용이며 로봇 실행은 여전히 검증된 YAML MoveJoint
- preview가 셀 전환을 막지 않도록 비동기 thread로 변경

---

## 4. 현재 토픽 구조

```
YOLO detector (학습 재진행 중)
  ├─→ /strawberry/detection/pick_pose      (PoseStamped)  → scan_executor (버퍼)
  └─→ /strawberry/detection/scene_positions (Float64MultiArray, [x,y,z×N]) → curobo_planner

scan_executor_node
  ├─→ /dsr01/curobo/pick_pose              → curobo_planner (순차 트리거)
  ├─→ /strawberry/scan/status              → harvest_session_logger, 모니터링
  └─→ /strawberry/exploration/set_cell_state → workspace_marker_node

curobo_planner_node
  ├─→ /dsr01/curobo/pick_complete          → scan_executor (완료 신호)
  └─→ /strawberry/vla/request              → VLA 팀 노드 (파지 실패 이관)
```

---

## 5. KPI 목표

| 항목 | 목표 | 현재 |
|------|------|------|
| 전체 사이클 (1개) | ≤ 35초 | 미측정 (YOLO 대기) |
| 파지 단계 | ≤ 3초 | 미측정 |
| 룰베이스 성공률 | ≥ 80% | 미측정 |
| VLA 이관 비율 | ≤ 20% | 미측정 |

모형 딸기: 정상 15개 + 비정상 15개 = 총 30개

---

## 6. 현재 차단 요소 (blockers)

| Blocker | 상태 | 대기 중인 것 |
|---------|------|------------|
| YOLO .pt 학습 파일 | draft 모델 2개 수령, 재학습 진행 중 | 최종 weight + confidence threshold |
| Seg/Pose fusion | 계약 기록 완료, runtime 미연결 | live frame에서 매칭 품질 검증 |
| 줄기 파지 파라미터 | 미티칭 | .pt 완성 후 DART로 실제 줄기 접근 각도 측정 |
| 스캔 포즈 그리퍼/카메라 각도 이슈 | v12 재티칭 + 전체 순회 검증 완료 | tool/collision model 재검증 후 pick 연동 |
| cuRobo 최적화 | branch dry-run + runtime preview 구현 | pose tolerance sampling과 collision model 정합 |
| griper feedback | 미구현 | FlangeSerialRead reg 281 구현 필요 |

---

## 6-A. YOLO Seg + Pose 모델 계약 (2026-06-01)

현재 참고 모델:

| 모델 | 로컬 경로 | 역할 |
|------|-----------|------|
| Segmentation | `/home/user/Downloads/strawberry_seg_best.pt` | 과실 mask와 상태 class |
| Pose | `/home/user/Downloads/strawberry_pose_best.pt` | 줄기 keypoint 3개 |

Seg class:

```text
0 = ripe
1 = unripe
2 = sick
```

Pose keypoint:

```text
KP0 = stem_base
KP1 = stem_mid
KP2 = stem_tip
```

Fusion 원칙:

- 두 모델을 반드시 함께 실행한다.
- pose box 중심이 segmentation mask 내부에 들어간 경우만 같은 과실로 매칭한다.
- 매칭된 객체 중 `ripe`만 수확 후보로 사용한다.
- `unripe`, `sick`, seg/pose 매칭 실패, keypoint confidence 낮음, mask 면적 이상치는 모두 수확 금지.
- 줄기 방향 벡터는 기본 `KP2 - KP0`, fallback `KP2 - KP1`로 계산한다.
- 설정 기록: `config/detection_models.yaml`

중요: 현재 `.pt`는 재학습 중인 draft asset이므로 Git에 넣지 않고 local path만 기록한다.

---

## 7. 다음 할 일 (우선순위 순)

### 즉시 (YOLO .pt 완성되면)
1. **Seg/Pose fusion viewer 또는 node 구현**
   - seg mask + class, pose bbox + KP0/KP1/KP2를 같은 frame에서 표시
   - pose box center inside mask 매칭 품질 확인
2. **YOLO 출력 포맷 확인**
   - stem vector가 image 2D인지, depth 결합 후 camera/base 3D인지 결정
3. **줄기 접근방향 → 그리퍼 quaternion 변환** 구현
   - 현재 `WALL_QUAT_WXYZ` 고정값 → 줄기 방향별 동적 계산으로 교체
4. **YOLO 노드 연결**: `/strawberry/detection/pick_pose` + `/strawberry/detection/scene_positions` 발행
5. **줄기 파지 DART 티칭**: `GRASP_Z_BIAS`, `GRASP_OFFSET` 재조정
6. **첫 실제 수확 시도** + KPI 기록 시작

### 이후
7. **pickability 필터** 구현 (depth std, 이웃 거리, 가장자리 여백)
8. **그리퍼 피드백** (FlangeSerialRead reg 281 → goal vs actual)
9. **v12 스캔 포즈 single-cell ROS 검증** (그리퍼 중심 정렬 기준)
10. **MoveIt trajectory 검증** (cuRobo 궤적 replay)
11. Place slot 전체 티칭 (현재 slot0만)

---

## 8. 주요 파일 경로

| 파일 | 역할 |
|------|------|
| `config/scan_pose_candidates_refit_candidate.yaml` | v12 그리퍼 중심 스캔 포즈 |
| `config/detection_models.yaml` | YOLO seg+pose 모델/fusion 계약 |
| `src/strawberry_motion/execution/scan_executor_node.py` | 스캔+픽 통합 실행기 |
| `src/strawberry_motion/execution/scan_safety.py` | motion gate 조건 |
| `scripts/harvest_session_logger.py` | KPI 로거 |
| `tools/dashboard/start_local_dashboard.sh` | 로컬 모니터링 dashboard 실행 |
| `tools/dashboard/ros2_bridge.py` | ROS2 상태/카메라 → dashboard state/MJPEG bridge |
| `docs/harvest_logs/` | 세션별 수확 결과 YAML |
| `docs/worklogs/2026-06-01.md` | 오늘 작업 상세 |
| `docs/worklogs/2026-06-02.md` | v12 검증, cuRobo branch 최적화 원리, 파지 파츠 기록 |
| `docs/runs/RUN-20260602-001_curobo_scan_branch_optimization.yaml` | cuRobo branch optimization dry-run 결과 |

**e0509_gripper_description repo:**
| 파일 | 역할 |
|------|------|
| `scripts/curobo_planner_node.py` | pick 시퀀스 (approach→grasp→retreat→place) |
| `scripts/harvest_session_logger.py` | KPI 로거 |

---

## 9. 실행 명령

```bash
# 전체 자동 순회 + pick 통합
ros2 launch strawberry_motion workspace_scan.launch.py \
  enable_robot_execution:=true target_cell:=all

# 전체 자동 순회만 검증 + runtime cuRobo preview 로그
ros2 launch strawberry_motion workspace_scan.launch.py \
  enable_robot_execution:=true \
  target_cell:=all \
  enable_pick_integration:=false \
  enable_runtime_curobo_preview:=true \
  runtime_curobo_preview_retries:=2 \
  scan_movej_vel_deg_s:=60.0 \
  scan_movej_acc_deg_s2:=90.0

# KPI 로거 (별도 터미널)
python3 ~/doosan_ws/src/strawberry_finalproject/scripts/harvest_session_logger.py

# 로컬 dashboard (별도 터미널)
cd ~/doosan_ws/src/strawberry_finalproject
bash tools/dashboard/start_local_dashboard.sh

# 시작 트리거
ros2 service call /strawberry/scan/start std_srvs/srv/Trigger {}

# 수동 detection 발행 (YOLO 없이 테스트용)
ros2 topic pub --once /strawberry/detection/pick_pose geometry_msgs/msg/PoseStamped \
  "{pose: {position: {x: 0.05, y: 0.36, z: 0.45}}}"

# 이웃 장애물 테스트
ros2 topic pub --once /strawberry/detection/scene_positions std_msgs/msg/Float64MultiArray \
  "{data: [0.1, 0.35, 0.5, 0.15, 0.37, 0.45]}"
```

---

## 10. 테스트 현황

```
53 tests passed (2026-05-29, 마지막 확인)
```

---

## 11. 핵심 Git commits

```
3b1240c  feat: integrate scan→pick sequence with direct inter-cell traversal (2026-06-01)
5b4db6c  scan executor: raise default speed 40→80°/s (2026-05-29)
0f19039  v11: enable automated traversal — all 4 cells physically validated
471c57b  v11: fix SW J4 angle representation -130.5 → +229.5
e8c9265  fix: update SE/SW to J1-aligned IK branch (J1 swing 54° vs 233°/127°)
513c4f1  fix: bypass cuRobo IK at scan time — use YAML endpoint joints directly

e0509_gripper_description repo:
66360c8  feat: neighbor sphere obstacles + upward retreat for damage minimization
47b0b57  feat: publish /strawberry/vla/request on grasp failure
```
