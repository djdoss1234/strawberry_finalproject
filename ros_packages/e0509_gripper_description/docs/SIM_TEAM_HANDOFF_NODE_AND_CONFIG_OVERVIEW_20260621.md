# 시뮬레이션 팀 인계 — 핵심 노드/설정 개요 (2026-06-21)

이 문서는 이 프로젝트를 처음 보는 시뮬레이션 담당자가 "지금 실제로 뭐가 돌고 있고,
시뮬레이션으로 옮기려면 뭘 대체해야 하는지"를 빠르게 파악하기 위한 상위 개요다.
각 모듈/노드의 함수 시그니처·내부 분리 구조 등 더 깊은 레벨은
`RUNTIME_MODULE_INTERFACE_SPEC_20260620.md`를 참고한다.

---

## 1. 프로젝트 한 줄 요약

벽면에 거치된 모형 딸기를 RealSense eye-in-hand 카메라 + YOLO(seg+pose 듀얼 모델)로
검출하고, Doosan E0509 + 커스텀 그리퍼로 cuRobo GPU 모션플래닝을 이용해 줄기를 파지/
분리한 뒤 트레이에 놓는 전체 자동 수확 파이프라인. 스캔(쿼드트리 셀 순회) → 검출 →
검증(가드) → 파지 → 분리(retreat) → place 순서로 동작한다.

---

## 2. 레포/패키지 구조

- 공식 git 레포: `~/doosan_ws/src/strawberry_finalproject` (이 안에서만 커밋한다)
- 이 레포 자체가 ROS2 패키지 `strawberry_motion`이다 (`package.xml`이 레포 루트에 있음,
  소스는 `src/strawberry_motion/`)
- 그 안에 서브패키지 `ros_packages/e0509_gripper_description`이 있다 — 이건 별도
  ROS2 패키지(`e0509_gripper_description`)이며, `~/doosan_ws/src/e0509_gripper_description`은
  이 서브패키지로의 **symlink**다 (둘은 동일한 파일, 별개 디렉터리 아님)
- 빌드는 두 패키지 모두 `colcon build`로 `~/doosan_ws`에서 진행

이 레포 **밖에** 있는 외부 의존 패키지 (시뮬레이션에 직접 영향):

| 패키지 | 역할 | 위치 |
| --- | --- | --- |
| `dsr_bringup2`, `dsr_msgs2` | Doosan 공식 ROS2 드라이버 — 로봇 컨트롤러 emulator(`run_emulator`, virtual/real mode), `MoveJoint`/`MoveLine`/`MoveSplineJoint`/`ChangeOperationSpeed` 서비스, `/dsr01/joint_states` 발행 | 레포 외부, 별도 빌드됨 |
| `dsr_gripper_tcp`, `dsr_gripper_tcp_interfaces` | 그리퍼 서비스 레이어 — `/gripper_service/set_position`, `/get_state`, `/safe_grasp` action 등 제공. 실제 시리얼/DRL 통신을 캡슐화 | `~/doosan_ws/src/dsr_gripper_tcp*` |
| `e0509_gripper_moveit_config` | MoveIt 설정(병행 planning-scene 검증용, 옵션) | 레포 외부 |
| cuRobo (`curobo`), `ultralytics` (YOLO) | GPU 모션플래닝, seg+pose 검출 모델 | pip 패키지 |

---

## 3. 활성 노드 전체 목록

`launch/workspace_scan.launch.py` + `ros_packages/.../launch/bringup_dsr.launch.py`가
실제 운영에 쓰는 launch 조합이다 (다른 launch 파일은 demo/legacy — 7절 참고).

| 노드 | 패키지 | 실행파일 | 역할 | 기본 launch |
| --- | --- | --- | --- | --- |
| `run_emulator` | `dsr_bringup2` | `run_emulator` | Doosan 컨트롤러(virtual/real) | `bringup_dsr.launch.py` |
| `gripper_joint_publisher` | `e0509_gripper_description` | `gripper_joint_publisher.py` | 그리퍼 joint state publish (RViz) | `bringup_dsr.launch.py` |
| `gripper_service_node`(e0509) | `e0509_gripper_description` | `gripper_service_node.py` | `/dsr01/gripper/open\|close\|position_cmd\|stroke` — 저수준 flange serial(DRL) 제어 | `bringup_dsr.launch.py` |
| `gripper_service_node`(dsr_gripper_tcp) | `dsr_gripper_tcp` | `gripper_service_node` | `/gripper_service/*` — set_position/get_state/safe_grasp action. **이름이 같은 별개 노드이니 혼동 주의** | 별도 실행(이 레포 밖) |
| `strawberry_fusion_node` | `e0509_gripper_description` | `strawberry_fusion_node.py` | RealSense 직접 캡처 + YOLO seg+pose fusion → 파지 target 계산/publish | `workspace_scan.launch.py` (`enable_fusion_detection:=true`) |
| `scan_executor_node` | `strawberry_motion` | `scan_executor_node` | 쿼드트리 셀 순회 스캔, dwell 중 detection 수집, pick trigger | `workspace_scan.launch.py` (`enable_robot_execution:=true`) |
| `curobo_planner_node` | `e0509_gripper_description` | `curobo_planner_node.py` | pick/place state machine, cuRobo planning, 그리퍼 파지/검증 | **launch 파일에 없음** — 수동 `ros2 run` (8절 참고) |
| `workspace_marker_node` | `strawberry_motion` | `workspace_marker_node` | RViz 셀 상태 마커 시각화 | `workspace_scan.launch.py` |
| `scan_pose_tcp_preview_node` | `strawberry_motion` | `scan_pose_tcp_preview_node` | RViz TCP/카메라 프리뷰 마커 | `workspace_scan.launch.py` |
| `rviz2` | — | — | 시각화 | `workspace_scan.launch.py` |
| `move_group`(MoveIt) | `e0509_gripper_moveit_config` | — | 병행 planning-scene 검증(옵션) | `workspace_scan.launch.py` (`enable_moveit:=true`) |

**죽은 코드(실행되지 않음)**: `strawberry_yolo_node.py`(구버전 단일모델, `curobo_vision.launch.py`
demo에만 잔존), `pick_place_node.py`, `object_tracking_node.py`, `digital_twin_bridge.py`,
`marker_tracking_node.py` — 어떤 launch 파일에도 없음. 시뮬레이션 작업 시 무시해도 된다.

---

## 4. 데이터 흐름

```text
RealSense RGB-D (strawberry_fusion_node가 pyrealsense2로 직접 캡처, 토픽 아님)
  -> strawberry_fusion_node.py (YOLO seg+pose, 줄기 keypoint 기반 grasp target 계산)
       -> /dsr01/curobo/pick_pose         (PoseStamped, 검출 시마다 1개씩)
       -> /strawberry/detection/scene_positions (Float64MultiArray, 이웃 딸기 장애물 등록용)

scan_executor_node (쿼드트리 셀 순회, YAML endpoint_joints_deg로 MoveJoint)
  -> /strawberry/exploration/set_cell_state -> workspace_marker_node (RViz)
  -> /strawberry/scan/status                (사람이 보는 진행 로그)
  -> /dsr01/curobo/pick_pose                (dwell 중 모은 detection을 1개씩 순차 전달)
       -> curobo_planner_node (cuRobo plan -> MoveSplineJoint/MoveLine 실행
                                -> gripper_service set_position/safe_grasp
                                -> detach + retreat -> (옵션) tray place)
       -> /dsr01/curobo/pick_complete       -> scan_executor_node (다음 target 트리거)
```

`curobo_planner_node`가 launch에 안 들어있는 이유: 실험적 파라미터가 많고(아래 8절) GPU
warmup(~30초)이 있어 별도 터미널에서 수동 기동하는 현재 워크플로우를 따름. 자세한 명령은
8절.

---

## 5. 토픽/서비스/액션 그래프

| Topic/Service/Action | Type | Publisher/Client | Subscriber/Server |
| --- | --- | --- | --- |
| `/dsr01/joint_states` | `sensor_msgs/JointState` | `dsr_bringup2`(컨트롤러) | curobo_planner_node, scan_executor_node, strawberry_fusion_node |
| `/dsr01/curobo/pick_pose` | `geometry_msgs/PoseStamped` | strawberry_fusion_node **또는** scan_executor_node | curobo_planner_node |
| `/dsr01/curobo/pick_complete` | `std_msgs/Empty` | curobo_planner_node | scan_executor_node |
| `/dsr01/curobo/target_pose` | `geometry_msgs/PoseStamped` | (수동 테스트용) | curobo_planner_node |
| `/dsr01/curobo/obstacles` | `std_msgs/String`(JSON) | (수동/디버그) | curobo_planner_node |
| `/strawberry/detection/pick_pose` | `geometry_msgs/PoseStamped` | YOLO/fusion | scan_executor_node (dwell 버퍼) |
| `/strawberry/detection/scene_positions` | `std_msgs/Float64MultiArray` | strawberry_fusion_node | curobo_planner_node (이웃 장애물) |
| `/strawberry/scan/status` | `std_msgs/String` | scan_executor_node | 모니터링 |
| `/strawberry/exploration/set_cell_state` | `std_msgs/String` | scan_executor_node | workspace_marker_node |
| `/strawberry/exploration/workspace_cells`, `/next_cell` | `MarkerArray`/`String` | workspace_marker_node | RViz |
| `/strawberry/scan_poses/tcp_preview` | `MarkerArray` | scan_pose_tcp_preview_node | RViz |
| `/dsr01/gripper/position_cmd` | `std_msgs/Int32` | curobo_planner_node, scan_executor_node, safe_grasp 관련 노드 | gripper_service_node(e0509) |
| `/dsr01/gripper/open`, `/close` | `std_srvs/Trigger` | (수동) | gripper_service_node(e0509) |
| `/dsr01/motion/move_spline_joint` | `dsr_msgs2/srv/MoveSplineJoint` | curobo_planner_node, scan_executor_node(진단) | `dsr_bringup2` |
| `/dsr01/motion/move_joint` | `dsr_msgs2/srv/MoveJoint` | curobo_planner_node, scan_executor_node | `dsr_bringup2` |
| `/dsr01/motion/move_line` | `dsr_msgs2/srv/MoveLine` | curobo_planner_node | `dsr_bringup2` |
| `/dsr01/motion/change_operation_speed` | `dsr_msgs2/srv/ChangeOperationSpeed` | curobo_planner_node | `dsr_bringup2` |
| `/gripper_service/set_position` | `dsr_gripper_tcp_interfaces/srv/SetPosition` | curobo_planner_node | `dsr_gripper_tcp` gripper_service_node |
| `/gripper_service/get_state` | `dsr_gripper_tcp_interfaces/srv/GetState` | curobo_planner_node | `dsr_gripper_tcp` gripper_service_node |
| `/gripper_service/safe_grasp` | `dsr_gripper_tcp_interfaces/action/SafeGrasp` | curobo_planner_node | `dsr_gripper_tcp` gripper_service_node |
| `/strawberry/scan/start` | `std_srvs/srv/Trigger` | (수동 트리거) | scan_executor_node |

---

## 6. 하드웨어 종속 부분 — 시뮬레이션 시 주의/대체 필요

1. **카메라 입력이 ROS 토픽이 아니다.** `strawberry_fusion_node.py`는 `pyrealsense2`
   SDK로 RealSense 디바이스를 직접 연다(`rs.pipeline().start(cfg)`). 시뮬레이션에서는
   이 노드를 그대로 못 쓴다 — 둘 중 하나 필요: (a) Gazebo/Isaac 카메라 센서를 RealSense
   SDK처럼 보이게 만드는 래퍼, 또는 (b) `strawberry_fusion_node`의 캡처 부분만 토픽
   구독으로 바꾼 시뮬용 변형 노드 작성. 후자가 더 현실적 — `_capture_frame_and_guards()`
   메서드 하나만 교체하면 됨(이번 세션에 분리됨, `RUNTIME_MODULE_INTERFACE_SPEC` 참고).
2. **로봇 컨트롤러는 이미 시뮬레이션 모드를 지원한다.** `dsr_bringup2`의 `run_emulator`가
   `mode:=virtual`(기본값)로 실행되면 실제 하드웨어 없이 `MoveJoint`/`MoveLine`/
   `MoveSplineJoint` 서비스와 `/dsr01/joint_states`를 흉내낸다. `bringup_gazebo.launch.py`/
   `bringup_real_gazebo.launch.py`는 Gazebo 물리 시뮬레이션 + ros2_control까지 연결한
   버전이 이미 존재한다.
3. **그리퍼도 virtual mode가 있다.** `gripper_service_node.py`(e0509, `/dsr01/gripper/*`)는
   `mode` 파라미터로 `real_robot_mode`를 끌 수 있다(시리얼 통신 skip). 다만 `dsr_gripper_tcp`의
   `/gripper_service/*`(set_position/get_state/safe_grasp, `curobo_planner_node`가 실제로
   쓰는 쪽)는 이 레포 밖 패키지라 virtual mode 여부는 별도 확인 필요.
4. **cuRobo 충돌 월드는 실측 좌표로 손티칭된 모델이다.** `config/environment.yaml`의
   `whiteboard_wall` cuboid는 실측 티치펜던트 좌표 기반(8절 참고)이고, 알려진 calibration
   drift 이슈가 있음(`wall_y_clamped`, 미해결, `NW_TROUBLESHOOTING_CASE_LOG` 항목9). 시뮬
   좌표계로 옮길 때 이 cuboid 위치를 시뮬 환경의 실제 벽 위치와 다시 맞춰야 한다.
5. **그리퍼 모델은 두 가지 cuRobo robot config 중 하나로 전환된다** —
   `tool_model_profile` 파라미터(`measured_tcp_260mm`(기본) 또는 `legacy_160mm`)가
   `config/curobo/e0509_gripper_measured_tcp.yml` 또는 `e0509_gripper.yml`을 선택한다.
   실제로 장착된 약 15.8cm 파지 파츠를 반영한 게 `measured_tcp_260mm` 쪽 — 시뮬 그리퍼
   메시/충돌구를 만들 때 이 모델(`config/curobo/e0509_spheres.yml`, `e0509_gripper.urdf`)을
   기준으로 맞춰야 한다.
6. **GPU 의존.** cuRobo MotionGen은 CUDA가 필요하다(`build_curobo_motion_gen`에서
   `cuda:0` 고정). GPU 없는 시뮬 환경에서는 `curobo_planner_node` 자체를 못 띄운다.

---

## 7. 핵심 설정 파일

| 파일 | 의미 |
| --- | --- |
| `config/environment.yaml` | cuRobo 충돌 월드(whiteboard_wall cuboid 등). 실측 좌표, calibration drift 알려진 이슈 있음 |
| `config/curobo/e0509_gripper_measured_tcp.yml` / `e0509_gripper.yml` | cuRobo robot config — 그리퍼 파츠 길이 모델 2종(`tool_model_profile`로 전환) |
| `config/curobo/e0509_gripper.urdf`, `e0509_spheres.yml` | 로봇 URDF + 충돌 구(sphere) 모델 |
| `ros_packages/.../config/camera_calibration_eye_in_hand.yaml`, `calibration_eye_in_hand_1.npz` | RealSense eye-in-hand 외부파라미터(카메라→그리퍼 변환). **이 두 파일은 untracked로 유지되며 절대 커밋/삭제하지 않는다** |
| `config/scan_pose_candidates_refit_candidate.yaml` | 실제 스캔에 쓰는 셀별 `endpoint_joints_deg`, `tcp_transform_base` (scan_executor_node가 로드) |
| `config/scan_pose_candidates_depth2.yaml` | 추가 깊이의 스캔 후보(untracked, 절대 커밋/삭제하지 않는다) |
| `config/panel_registration.yaml` | RViz `cultivation_panel` 프레임 TF — **시각화/오프라인 충돌검토용일 뿐**, `use_for_automated_motion: false`로 실제 로봇 동작에는 안 씀 |
| `config/workspace.yaml` | 물리 작업영역(화이트보드) 치수, 쿼드트리 셀 분할 기준 |
| `config/place_slots.yaml` | 트레이 place slot 좌표(티칭/계산) |
| `ros_packages/.../scripts/harvest_motion_params.py` | 로봇 동작 튜닝 상수 전체(약 70개) — 파지 오프셋, 속도, 관절한도, NW_HIGH_TARGET 보정값 등. 코드이지만 사실상 설정 파일 |

**로봇 관절 한도** (`OPERATIONAL_JOINT_LIMITS_DEG`, `harvest_motion_params.py`):
J1 ±225°, **J2 ±95°**, J3 ±135°, J4 ±360°, J5 ±130°, J6 ±225°. J2가 가장 좁고, 실기에서
이 한도를 넘어 컨트롤러가 멈춘 사고가 두 차례 있었음(원인 분석은 `NW_TROUBLESHOOTING_CASE_LOG`
항목8~9) — 시뮬 충돌/한도 검증에서 J2를 우선 확인 권장.

---

## 8. 수동 기동 절차 (현재 워크플로우)

1. `ros2 launch e0509_gripper_description bringup_dsr.launch.py mode:=virtual` (또는 `real`)
   — 컨트롤러 emulator + 그리퍼 저수준 서비스
2. `dsr_gripper_tcp`의 `gripper_service_node` 실행 (별도 패키지, `/gripper_service/*` 제공)
3. `curobo_planner_node` 수동 실행 (8-A 참고, GPU warmup 약 30초 소요)
4. `ros2 launch strawberry_motion workspace_scan.launch.py ...` (8-B 참고) — 시각화 +
   fusion 검출 + scan executor를 한 launch에서 켬
5. `ros2 service call /strawberry/scan/start std_srvs/srv/Trigger "{}"` 로 스캔 시작

### 8-A. `curobo_planner_node` 실행 파라미터 (검증된 예시, 2026-06-20 기준)

```bash
ros2 run e0509_gripper_description curobo_planner_node.py --ros-args \
  -p measured_tcp_plan_only:=false \
  -p direct_curobo_final_approach_for_measured_tcp:=true \
  -p measured_tcp_max_approach_m:=0.200 \
  -p measured_tcp_tool_line_after_curobo_fallback:=true \
  -p debug_dump_plan_calls:=true
```

| 파라미터 | 의미 |
| --- | --- |
| `measured_tcp_plan_only` | **false=실제로 로봇 움직임.** true면 plan만 하고 motion dispatch 안 함(안전점검 모드) |
| `direct_curobo_final_approach_for_measured_tcp` | 최종 접근(final approach) 일부를 직선(MoveLine) 대신 cuRobo plan으로 먼저 시도 |
| `measured_tcp_max_approach_m` | 최종 접근 최대 진입 깊이(m). 기본보다 늘려서 더 깊은 타겟을 허용 |
| `measured_tcp_tool_line_after_curobo_fallback` | cuRobo가 일부 깊이까지만 풀리면 남은 구간을 직선(TOOL Z) 이동으로 채움 |
| `debug_dump_plan_calls` | 매 `plan()` 호출을 파일로 덤프 — 오프라인 재생(replay)으로 실패 원인 분석 가능하게 함 |

### 8-B. `workspace_scan.launch.py` 실행 파라미터 (검증된 예시)

```bash
ros2 launch strawberry_motion workspace_scan.launch.py \
  enable_robot_execution:=true target_cell:=root/nw \
  enable_fusion_detection:=true enable_pick_integration:=true \
  collect_then_pick:=true collect_pick_ready_cell:=root/nw/pick_ready \
  fusion_pick_target_max_z_m:=0.95 \
  scan_movej_vel_deg_s:=20.0 scan_movej_acc_deg_s2:=30.0 \
  overview_return_vel_deg_s:=20.0 overview_return_acc_deg_s2:=30.0
```

| 파라미터 | 대상 노드 | 의미 |
| --- | --- | --- |
| `enable_robot_execution` | (gate) | **`scan_executor_node`를 실제로 띄움.** false면 시각화만 되고 노드 자체가 안 뜸 |
| `target_cell` | scan_executor | 어느 셀만 스캔할지(`all`이면 전체 순회) |
| `enable_fusion_detection` | (gate) | **`strawberry_fusion_node`를 실제로 띄움** |
| `enable_pick_integration` | scan_executor | dwell 중 감지된 pose를 실제로 pick 시퀀스로 넘길지 |
| `collect_then_pick` | scan_executor | 서브셀 전체를 먼저 다 스캔해 후보를 모은 다음, 한 군데(pick-ready pose)에서 한 개씩 pick |
| `collect_pick_ready_cell` | scan_executor | collect_then_pick 후 이동할 pick 전용 pose 셀 ID |
| `fusion_pick_target_max_z_m` | strawberry_fusion_node | 이 높이(base_link 기준, m) 위 후보는 잎/꼭대기로 간주해 거절 |
| `scan_movej_vel_deg_s` / `scan_movej_acc_deg_s2` | scan_executor | 셀 간 이동 MoveJoint 속도/가속도 |
| `overview_return_vel_deg_s` / `overview_return_acc_deg_s2` | scan_executor | overview 복귀 속도/가속도 |

`/strawberry/scan/start`는 `scan_executor_node`가 직접 제공하는 서비스 서버다 — launch만으로는
자동으로 안 움직이고 이 서비스를 명시적으로 호출해야 스캔이 시작되는 안전장치다.

**파라미터 두 그룹의 구분**: `ros2 run curobo_planner_node` 파라미터는 *그 노드 하나의*
접근/안전 동작 튜닝이고, `ros2 launch workspace_scan.launch.py` 파라미터는 *어느 노드를
켤지 + scan_executor/fusion의 동작 옵션*이다 — 서로 다른 프로세스에 붙는 별개 설정이며
같은 launch/run 안에서 섞이지 않는다.

---

## 9. 참고 문서

- `RUNTIME_MODULE_INTERFACE_SPEC_20260620.md` — 모듈별 함수 시그니처, 입출력 상세, 리팩토링 진행상황
- `NW_TROUBLESHOOTING_CASE_LOG_20260621.md` — 실기에서 겪은 상황별 시도/결과 기록(발표용)
- `docs/CURRENT_STATUS.md` — 더 오래된(2026-06-02) 프로젝트 진행 요약, 일부 내용은 이 문서로 대체됨

**주의**: `scripts/측정.py`는 절대 수정/삭제/커밋하지 않는다. `config/scan_pose_candidates_depth2.yaml`,
`ros_packages/e0509_gripper_description/config/camera_calibration_eye_in_hand.yaml`은 untracked
상태를 유지해야 한다(커밋/삭제/restore 금지).
