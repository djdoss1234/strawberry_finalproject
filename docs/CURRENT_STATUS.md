# 현재 진행 상태 — Handoff

최종 갱신일: 2026-05-29 (v11 전체 자동 순회 활성화, 4셀 물리 검증 완료)

이 문서는 새 세션에서 가장 먼저 읽을 압축 상태 요약입니다.

---

## 1. 프로젝트 역할

- 담당자: djdoss1234
- 담당 범위: 전체 딸기 수확 motion system
  - workspace 탐색/scan motion (쿼드트리 기반 4셀 순회)
  - target validation, approach, grasp, retreat, transfer, place
  - planner/executor, collision/retry, 평가
- 팀원 범위: 복잡 장면의 VLA 기반 수확 판단
- 통합 원칙: VLA proposal → motion 측 geometry/collision 검증 후 실행

---

## 2. 현재 상태 한 줄 요약

> **쿼드트리 4분면 스캔 순회 완성.** NW→NE→SE→SW 자동 순회 실행 가능.
> 다음은 YOLO 검출 결과를 cell state에 연동하는 것.

---

## 3. 완료된 구현 (전체 아크)

### 3-A. 쿼드트리 + ROS 기반 탐색 구조 (2026-05-26~27)

- `QuadtreeMap`, `WorkspaceBounds`, `RegionState` — ROS 비의존 순수 Python
- `workspace_marker_node` — RViz visualization
- `camera_alignment_node` / `realsense_alignment_viewer` — overview 정렬 도구
- `scan_pose_tcp_preview_node` — RViz TCP 화살표 preview
- `scan_pose_target_exporter` — cell center → TCP target 변환
- publish/subscribe 인터페이스:
  ```
  /strawberry/exploration/set_cell_state
  /strawberry/exploration/workspace_cells
  /strawberry/exploration/next_cell
  ```

### 3-B. 패널 등록 및 collision world (2026-05-27)

- `panel_landmark_capture` — RealSense depth 기반 5점 측정
- `panel_registration_validator` — RMS/MAX 판정, refit 후보 생성
- `config/panel_registration.yaml` — base_link 기준 cultivation_panel TF
- `config/scan_collision_world.yaml` — whiteboard cuboid 포함
- refit 결과: RMS=9.2mm, MAX=11.0mm (MEASURED_PASS)

### 3-C. Scan pose 후보 탐색 (2026-05-27~28)

- `scripts/search_all_cells_camera_centered.py` — v8 해석적 카메라 중심 탐색
- `scripts/search_sw_candidates.py` / `search_se_candidates.py` — 개별 셀 탐색
- `scripts/validate_scan_poses.py` / `validate_v6_collision_scan_poses.py` — cuRobo MotionGen dry-run
- **v8까지 해석적 접근 실패** (FK/calibration 오차, SE/SW 빈 벽 촬영)
  → DART 수동 티칭으로 전환 결정

### 3-D. Workspace Reset + DART 수동 티칭 (2026-05-28~29)

- 물리 workspace 재배치: 종이판 중심 ↔ base_link 중심선 정렬
- DART로 4셀 스캔 포즈 직접 티칭 후 ROS 싱글셀 검증
- 카메라 앵글 4셀 모두 확인 완료
- 기록: `docs/runs/RUN-20260529-002_manual_scan_pose_teaching.md`

### 3-E. IK 비결정성 수정 (2026-05-28, commit 513c4f1)

- cuRobo가 runtime에서 오프라인 탐색과 다른 IK 해를 선택해 로봇이 엉뚱한 위치로 이동
- **해결**: scan executor에서 cuRobo 완전 제거, YAML `endpoint_joints_deg` 직접 MoveJoint
- `_SCAN_MOVEJ_VEL_DEG_S`, `_SCAN_MOVEJ_ACC_DEG_S2` 통합 상수로 단순화
- 기록: `docs/runs/RUN-20260528-006_ik_nondeterminism_fix.md`

### 3-F. SE/SW IK Branch 최적화 (2026-05-29, commits e8c9265 ~ 471c57b)

문제:
- SE: DART 티칭 조인트 J1=-145° → overview(J1=+88°) 대비 swing 233°
- SW: DART 티칭 조인트 J1=-39° → swing 127°
- J3도 각각 255° swing (NW/NE는 J3 swing < 5°)

해결:
- `scripts/search_se_sw_better_branch.py` — cuRobo IKSolver(128 seeds × 30 batches)로 동일 TCP에서 J1-aligned branch 탐색
- SE 신규: J1=+34.4° (swing 54°, **4.3배 개선**), J3 swing 4° (**64배 개선**)
- SW 신규: J1=+141.3° (swing 53°, **2.4배 개선**), J3 swing 4° (**64배 개선**)
- SW J4: -130.5° → +229.5° (동일 물리 위치, MoveJoint 방향 버그 수정)
- `tcp_transform_base` 모두 FK 결과로 재계산 (로봇 실제 도달 위치와 일치)
- SE 물리 싱글셀 검증 완료 / SW 물리 싱글셀 검증 완료

### 3-G. 전체 자동 순회 활성화 (2026-05-29, commit 0f19039)

- `use_for_automated_motion: true` (전역 + 4셀 per-cell)
- `collision_world_validated_for_motion: true`
- `curobo_status: PHYSICAL_VIEW_CONFIRMED_AFTER_WORKSPACE_RESET` (4셀 모두)
- 속도 기본값 40→80°/s, acc 60→100°/s², service timeout 5→30s, inter-cell wait 15→45s

---

## 4. 현재 활성 설정

| 항목 | 값 |
|------|----|
| YAML 버전 | `v11_se_sw_ik_branch_optimized` |
| 실행 파일 | `config/scan_pose_candidates_refit_candidate.yaml` |
| 자동 순회 | 허가됨 (`use_for_automated_motion: true`) |
| 스캔 속도 | 80°/s, 100°/s² |
| 복귀 속도 | 80°/s, 100°/s² |
| 순회 순서 | NW → NE → SE → SW |

### Overview 포즈

```yaml
joints_deg: [87.98, -94.92, 129.89, 175.94, -31.34, 93.42]
```

### 4셀 Endpoint (v11)

| Cell | J1 | J1 swing | J3 swing | 상태 |
|------|----|----------|----------|------|
| root/nw | +139.76° | 51.8° | ~209° (복귀 경로) | 검증 완료 |
| root/ne | +34.82° | 53.2° | ~212° (복귀 경로) | 검증 완료 |
| root/sw | +141.3° | 53.3° | 4.4° | 검증 완료 |
| root/se | +34.4° | 54.0° | 4.0° | 검증 완료 |

### 실행 명령

```bash
# 전체 자동 순회
ros2 launch strawberry_motion workspace_scan.launch.py \
  enable_robot_execution:=true target_cell:=all

# 단일 셀 검증
ros2 launch strawberry_motion workspace_scan.launch.py \
  enable_robot_execution:=true \
  manual_validation_mode:=true \
  target_cell:=root/nw

# 시작 트리거 (별도 터미널)
ros2 service call /strawberry/scan/start std_srvs/srv/Trigger {}
```

---

## 5. 해결한 주요 문제들

| 문제 | 원인 | 해결 |
|------|------|------|
| SE/SW 빈 벽 촬영 | 해석적 FK/calibration 오차 누적 | DART 수동 티칭으로 전환 |
| cuRobo runtime IK 비결정성 | seed 기반 stochastic IK, runtime≠offline | scan executor에서 cuRobo 제거, YAML 직접 실행 |
| SE J1 swing 233° / SW 127° | DART가 반대쪽 IK branch 선택 | cuRobo IKSolver branch search, 동일 TCP에서 J1-aligned 해 탐색 |
| SW J4 306° 회전 버그 | -130.5°와 +229.5°는 같은 위치이나 부호로 역방향 | +229.5°로 수정 (53.6° 전진) |
| tcp_transform_base 미세 오차 | DART 티칭 TCP ≠ 새 조인트의 FK 결과 | FK로 재계산해서 완전 일치 |
| inter-cell overview 미도착 ABORT | 40°/s에서 J3=209° 이동 시 timeout 경합 | 80°/s + 45s timeout |

---

## 6. 미해결 / 다음 할 일

### 즉시 (다음 세션)

1. **YOLO 검출 → cell state 연동**
   - 현재: scan dwell 동안 `/strawberry/pick_pose` 수신 개수로 대체 중
   - 필요: YOLO 검출기 실제 토픽 연결 후 TARGET_FOUND/SCANNED_EMPTY 검증

2. **그리퍼 파지 피드백 구현**
   - 현재: write-only (`FlangeSerialWrite`만), `USE_GRASP_CHECK = False`
   - 가능: `FlangeSerialRead`로 Present Position (reg 281) 읽어 goal vs actual 비교
   - goal=700이지만 actual=450 → 물체 잡힘 / actual≈700 → 허공 파지

3. **4셀 자동 순회 → pick 시퀀스 연결**
   - 순회 중 TARGET_FOUND된 셀에서 pick 트리거
   - `curobo_planner_node.py`의 pick 시퀀스 (approach→grasp→retreat→place) 연결

### 이후

4. Place slot 전체 티칭 (현재 slot0 above/release만 저장됨)
5. Occupied slot 관리 로직
6. cuRobo MotionGen false self-collision 수정 (overview J3=129.89°/J4=175.94° 구면 모델 오탐)
7. 전체 수확 루프 (scan→pick→place×N) 연속 실행

---

## 7. 주요 파일 경로

| 파일 | 역할 |
|------|------|
| `config/scan_pose_candidates_refit_candidate.yaml` | v11 스캔 포즈 (현재 기준) |
| `config/panel_registration.yaml` | base_link → cultivation_panel TF |
| `config/scan_collision_world.yaml` | whiteboard cuboid collision |
| `config/workspace.yaml` | 셀 bounds, split point |
| `src/strawberry_motion/exploration/quadtree_map.py` | 쿼드트리 pure Python core |
| `src/strawberry_motion/execution/scan_executor_node.py` | 스캔 실행기 (666줄) |
| `src/strawberry_motion/execution/scan_safety.py` | motion gate 조건 |
| `scripts/search_se_sw_better_branch.py` | IK branch 탐색 스크립트 |
| `docs/runs/RUN-20260528-006_ik_nondeterminism_fix.md` | IK 비결정성 근본원인 |
| `docs/runs/RUN-20260529-002_manual_scan_pose_teaching.md` | 수동 티칭 결과 |
| `docs/runs/RUN-20260529-003_se_sw_branch_search.yaml` | IK branch 탐색 결과 |

---

## 8. 테스트 현황

```
53 tests passed (2026-05-29)
```

---

## 9. 핵심 Git commits

```
5b4db6c  scan executor: raise default speed 40→80°/s (2026-05-29)
0f19039  v11: enable automated traversal — all 4 cells physically validated
471c57b  v11: fix SW J4 angle representation -130.5 → +229.5
aa98380  v11: sync SE/SW tcp_transform_base to cuRobo FK result
e8c9265  fix: update SE/SW to J1-aligned IK branch (J1 swing 54° vs 233°/127°)
890976e  feat: add IK branch search script for SE/SW
513c4f1  fix: bypass cuRobo IK at scan time — use YAML endpoint joints directly
```
