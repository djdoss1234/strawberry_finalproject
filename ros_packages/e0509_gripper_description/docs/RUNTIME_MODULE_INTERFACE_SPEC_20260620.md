# 딸기 자동수확 런타임 모듈/인터페이스 명세 (2026-06-20, 2026-06-21 갱신)

이 문서는 실기 자동수확에 실제로 관여하는 핵심 파일을 기준으로 작성한다.
로그 분석/라벨링/회고 문서 등 실행 경로 밖의 파일은 별도 도구로 분류한다.

시뮬레이션 팀 인계용 상위 개요(활성 노드 전체 목록/launch 구조/외부 패키지 의존성/
설정 파일 의미)는 별도 문서 `SIM_TEAM_HANDOFF_NODE_AND_CONFIG_OVERVIEW_20260621.md`를
참고. 이 문서는 모듈/인터페이스 상세(토픽·서비스·액션 타입, 함수 시그니처) 전용이다.

## 1. 현재 주 실행 파이프라인

```text
RealSense RGB-D
 -> strawberry_motion workspace scan / camera pose 이동
 -> strawberry_fusion_node.py
 -> /dsr01/curobo/pick_pose
 -> curobo_planner_node.py
 -> cuRobo plan + Doosan MoveSplineJoint/MoveLine
 -> gripper_service SafeGrasp / SetPosition / GetState
 -> /dsr01/curobo/pick_complete
 -> scan executor가 다음 target 또는 재스캔 결정
```

## 2. 핵심 파일 책임

| 파일 | 역할 | 현재 상태 |
| --- | --- | --- |
| `scripts/curobo_planner_node.py` | pick/place state machine, cuRobo planning, runtime JSONL | 주 실행 노드. 2026-06-21 기준 **1,926 lines**. `_pick()`은 `_prepare_pick_target_or_abort()`(target 준비+x/z 가드) → `_search_grasp()`(quat variant 탐색) → 실행 시퀀스로 분리, 394→248줄. `__init__`(180줄)도 cuRobo 부트스트랩/파라미터는 `planner_bootstrap.py`로, 시작 로그 배너는 `_log_startup_banner()`로 분리됨 — 남은 부분은 subscription/client/executor 생성 wiring(`self` 의존 필수) |
| `scripts/planner_bootstrap.py` | `__init__`에서 1회 실행되는 cuRobo MotionGen 구성(`build_curobo_motion_gen`)과 ~40개 ROS 파라미터 declare/load(`declare_and_load_params`) | 신규 분리 모듈(2026-06-21). 반복 실행되는 런타임 동작이 없어 클래스가 아닌 plain function 2개로 구성 |
| `scripts/fusion_bootstrap.py` | `strawberry_fusion_node.__init__`에서 1회 실행되는 ~28개 ROS 파라미터 declare/load(`declare_and_load_fusion_params`), `_as_bool` 헬퍼 | 신규 분리 모듈(2026-06-21). `planner_bootstrap.py`와 동일 패턴 |
| `scripts/approach_retreat_policy.py` | measured-TCP final approach 거리, fallback depth, tool-finish 방향, retreat step 계산 | 신규 분리 모듈. J2 한도초과 방지용 2단계 retreat와 tilted branch 수평 tool-finish 정책을 순수 함수로 분리 |
| `scripts/doosan_motion_client.py` | Doosan `MoveSplineJoint`/`MoveJoint`/`MoveLine` service 호출, timeout/no-motion guard, motion logging | 신규 분리 모듈. 기존 motion 동작 보존용 thin wrapper |
| `scripts/gripper_client.py` | `/gripper_service/set_position`, `/get_state`, `/safe_grasp` 호출, approach open, close+verify logging, grasp result 판정 | 신규 분리 모듈. SafeGrasp + position fallback 동작 보존 |
| `scripts/grasp_search_executor.py` | grasp_quat_variant별 measured-TCP depth probe와 legacy grasp offset 탐색 실행(`self.plan()` 호출 포함) | 신규 분리 모듈(2026-06-21). `HarvestGripperClient`와 동일한 node-dependent client 패턴 |
| `scripts/tray_place_executor.py` | marker-place와 taught-slot0-grid place 시퀀스 실행(`self.plan()`/`execute_spline()`/`execute_base_z_relative()` 호출 포함, row2 분기 포함) | 신규 분리 모듈(2026-06-21), 같은 날 추가로 내부 분리됨: `execute_marker_place_after_retreat`(183→130줄)에서 clearance/orientation 후보 탐색 루프를 `_search_marker_place_above()`로, `execute_taught_slot0_place_reference_after_retreat`(225→194줄)에서 슬롯 검증+위치 계산을 `_compute_taught_slot_above_target()`으로 분리. row2 known 이슈는 그대로 이동(미수정) |
| `scripts/tray_place_policy.py` | marker tray JSON 로딩, Slot0/1/3 기반 grid pitch 보정, slot offset/release target 계산 | 신규 분리 모듈. 로봇 I/O 없이 place target만 생성 |
| `scripts/trajectory_guards.py` | operational joint limit, equivalent joint normalization, spline jump/swing reject | 신규 분리 모듈. cuRobo trajectory 실행 전 안전 필터 |
| `scripts/curobo_planning_adapter.py` | cuRobo Cartesian/joint-space planning 호출, plan success/fail logging, start collision diagnostic | 신규 분리 모듈. MotionGen 호출부와 planner reject logging 분리 |
| `scripts/curobo_kinematics_adapter.py` | cuRobo FK ee pose 조회, trajectory Cartesian line deviation 계산 | 신규 분리 모듈. FK/diagnostic 계산을 planner 본문에서 분리 |
| `scripts/scene_obstacle_manager.py` | dynamic obstacle JSON, detection scene positions, neighbor sphere 등록/해제, cuRobo world update logging | 신규 분리 모듈. collision world 상태 관리 |
| `scripts/grasp_candidate_policy.py` | target별 grasp offset/quat variant pose/depth probing 후보, measured-TCP tie-break, `GraspSearchResult` | 신규 분리 모듈. 실패가 많던 후보 선택 정책을 planner 본문에서 분리 |
| `scripts/harvest_result_policy.py` | grasp result 기반 place gate, block reason, pick sequence result code 정책 | 신규 분리 모듈. KPI/result taxonomy 변경 시 planner 본문 수정 최소화 |
| `scripts/open_stem_descent_policy.py` | 열린 그리퍼 상태에서 KP1까지 내려가는 BASE -Z 거리 계산 | 신규 분리 모듈. NW/SW 파지 높이 조정 로직을 독립 계산식으로 분리 |
| `scripts/pick_target_policy.py` | pick PoseStamped 위치에서 raw/grasp target, wall-Y clamp, NW-high 판정, x/z guard 계산 | 신규 분리 모듈. target preparation/guard 정책 분리 |
| `scripts/place_sequence_policy.py` | place executor status를 state-machine action/result code로 변환 | 신규 분리 모듈. place 후 hold/skip/continue 정책 분리 |
| `scripts/row2_place_policy.py` | row2 tray place descent/ascent Cartesian line deviation 판정 | 신규 분리 모듈. row2 place 안전 threshold 판정 분리 |
| `scripts/harvest_math.py` | quaternion/vector 순수 수학 함수 | 신규 분리 모듈 |
| `scripts/harvest_grasp_orientation.py` | perception이 보낸 줄기 방향을 wall-normal roll 후보로 변환 | 신규 분리 모듈 |
| `scripts/harvest_motion_params.py` | 실험 상수, 티칭 pose, 속도/거리/한계값 | 신규 분리 모듈. 값 자체는 debug branch 현행값 유지 |
| `scripts/marker_place_orientation_policy.py` | marker place orientation 후보, clearance 후보, measured-TCP contact 보정 target 계산 | 신규 분리 모듈. place orientation 탐색의 순수 계산 분리 |
| `scripts/strawberry_fusion_node.py` | YOLO/keypoint/depth 기반 target fusion, stem direction orientation 생성, pick pose publish | NW 인식/깊이 실패가 많은 현재 병목. 2026-06-21 기준 994 lines. `_loop()`(매 프레임 호출, 372줄)을 `_capture_frame_and_guards`/`_run_or_reuse_inference`/`_draw_seg_overlays`/`_publish_scene_positions`/`_process_pose_detection`(241줄, 가장 큰 블록) 5단계로 분리 |
| `src/strawberry_motion/execution/scan_executor_node.py` | 쿼드트리 셀 순회 스캔, dwell 중 detection 수집, pick trigger 순차 전달, collect-then-pick 버퍼링 | `strawberry_motion` 패키지(별도 ROS 패키지, 이 디렉터리 밖). 2026-06-21 기준 1,261 lines. `_scan_sequence()`(245줄)를 `_compute_scan_order`/`_scan_one_cell`/`_finish_collect_then_pick`/`_finish_scan_sequence`로, `_scan_one_cell`(126줄)을 다시 `_move_to_scan_cell_and_wait`/`_process_cell_detections`로, `__init__`(165줄)의 파라미터 블록을 `_declare_and_load_params()`로 분리. 이 패키지엔 아직 policy/executor 모듈 분리 컨벤션이 없어 전부 in-class 메서드로 유지(새 파일 미생성) |
| `scripts/strawberry_yolo_node.py` | 이전/단일 카메라 YOLO baseline, pick pose publish | **확인된 죽은 코드**(2026-06-21) — 활성 `workspace_scan.launch.py`는 `strawberry_fusion_node.py`만 실행, 이 파일은 미니프로젝트 시절 Grounding DINO 데모용 `curobo_vision.launch.py`에만 남아있음 |
| `scripts/joint_jog_control.py` | 수동 joint/pose 이동, TCP 확인, gripper 위치 테스트 | 실기 티칭/복구 도구 |
| `scripts/runtime_jsonl_logger.py` | 각 노드 runtime JSONL 저장 | KPI/디버깅의 원자료 |
| `scripts/summarize_runtime_kpis.py` | runtime JSONL 요약 | 자동 KPI 추출 |
| `scripts/generate_harvest_kpi_report.py` | KPI 리포트/그래프 생성 | 반복 실험 후 사용 |
| `scripts/prepare_harvest_label_sheet.py` | 수동 라벨 CSV 생성/갱신 | 사람이 실험 후 입력 |
| `scripts/run_safe_grasp_trial.py` | SafeGrasp 단독 전류/position 테스트 | 그리퍼 파지 판정 threshold 보정용 |
| `scripts/clean_robot_runtime.sh` | stale ROS/그리퍼 프로세스 정리 | gripper_service 재시작 전 사용 |
| `config/environment.yaml` | whiteboard/table 등 cuRobo world obstacle source | 현재 wall cuboid 중심 |
| `config/curobo/e0509_gripper_measured_tcp.yml` | measured TCP용 cuRobo robot config | 현재 planner 기본 모델 |
| `config/place_slots.yaml` | 티칭/계산된 tray slot 정보 | place 실험용 |

## 3. `curobo_planner_node.py` 인터페이스

### Subscriptions

| Topic | Type | 사용 |
| --- | --- | --- |
| `/dsr01/joint_states` | `sensor_msgs/JointState` | 현재 joint state, planning start state |
| `/dsr01/curobo/target_pose` | `geometry_msgs/PoseStamped` | 일반 cuRobo target test |
| `/dsr01/curobo/pick_pose` | `geometry_msgs/PoseStamped` | 수확 target. position은 grasp target, orientation은 per-target stem direction 후보 |
| `/dsr01/curobo/obstacles` | `std_msgs/String` JSON | 동적 cuboid obstacle 갱신 |
| `/strawberry/detection/scene_positions` | `std_msgs/Float64MultiArray` | 이웃 딸기 sphere obstacle 등록 |
| `/strawberry/scan/status` | `std_msgs/String` | scan 상태 로그 |
| `/strawberry/exploration/set_cell_state` | `std_msgs/String` | cell 상태 로그 |

### Publishers

| Topic | Type | 의미 |
| --- | --- | --- |
| `/dsr01/curobo/pick_complete` | `std_msgs/Empty` | pick sequence 종료 알림. 성공률이 아니라 “시퀀스 종료” 이벤트 |

### Service Clients

| Service | Type | 사용 |
| --- | --- | --- |
| `/dsr01/motion/move_spline_joint` | `dsr_msgs2/srv/MoveSplineJoint` | cuRobo trajectory 실행 |
| `/dsr01/motion/move_joint` | `dsr_msgs2/srv/MoveJoint` | fixed joint pose 이동, place/scan 복귀 |
| `/dsr01/motion/move_line` | `dsr_msgs2/srv/MoveLine` | TOOL/BASE 직선 진입, 하강, retreat |
| `/dsr01/motion/change_operation_speed` | `dsr_msgs2/srv/ChangeOperationSpeed` | operation speed 변경 |
| `/gripper_service/set_position` | `dsr_gripper_tcp_interfaces/srv/SetPosition` | gripper open/release position 명령 |
| `/gripper_service/get_state` | `dsr_gripper_tcp_interfaces/srv/GetState` | fallback grasp state read |

### Action Clients

| Action | Type | 사용 |
| --- | --- | --- |
| `/gripper_service/safe_grasp` | `dsr_gripper_tcp_interfaces/action/SafeGrasp` | close 중 position/current 기반 파지 감지 |

### 주요 ROS Parameters

| Parameter | 의미 |
| --- | --- |
| `measured_tcp_plan_only` | true면 motion dispatch 없이 plan만 확인 |
| `direct_curobo_final_approach_for_measured_tcp` | final approach 일부를 cuRobo로 먼저 계획 |
| `measured_tcp_max_approach_m` | measured TCP final approach 최대 진입 거리 |
| `measured_tcp_tool_line_after_curobo_fallback` | cuRobo가 일부 깊이까지만 풀리면 남은 구간을 직선 실행 |
| `use_published_grasp_orientation` | fusion orientation 기반 roll 후보 사용 |
| `published_grasp_roll_align_axis` | stem direction에 맞출 gripper axis (`x` 또는 `y`) |
| `nw_high_target_final_extra_m` | NW high/tilted branch 깊이 추가 |
| `enable_marker_place_sequence` | pick 후 marker/tray place 시퀀스 실행 |
| `use_safe_grasp_action` | gripper close를 SafeGrasp action으로 수행 |

## 4. `strawberry_fusion_node.py` 인터페이스

### Inputs

| 입력 | 의미 |
| --- | --- |
| RealSense color/depth stream | RGB-D detection source |
| `/dsr01/joint_states` | eye-in-hand transform 계산용 현재 joint |

### Outputs

| Topic | Type | 의미 |
| --- | --- | --- |
| `/dsr01/curobo/pick_pose` | `geometry_msgs/PoseStamped` | 안정화된 수확 target. position은 base target, orientation은 stem direction 기반 |
| `/strawberry/detection/scene_positions` | `std_msgs/Float64MultiArray` | 이웃 과실 위치, planner obstacle 등록용 |

### 주요 Parameters

| Parameter | 의미 |
| --- | --- |
| `stem_grasp_direction_mode` | `kp0_to_kp1` 등 줄기 방향 계산 기준 |
| `stem_grasp_offset_from_kp0_m` | KP0 기준 grasp target offset |
| `pick_target_min_z_m`, `pick_target_max_z_m` | leaf/top 후보 필터 |
| `prefer_lower_z_target` | 낮은 줄기 후보 우선 |
| `target_position_window_size` | target 안정화 window |
| `target_position_max_spread_m` | 안정 target spread 제한 |

## 5. `scan_executor_node.py` 인터페이스 (strawberry_motion 패키지)

### Subscriptions

| Topic | Type | 사용 |
| --- | --- | --- |
| `/dsr01/joint_states` | `sensor_msgs/JointState` | 현재 joint state, 도착 확인 |
| `/strawberry/detection/pick_pose` | `geometry_msgs/PoseStamped` | YOLO/fusion → scan_executor (dwell 중 버퍼링) |
| `/dsr01/curobo/pick_complete` | `std_msgs/Empty` | planner pick 완료 신호, 다음 target 트리거 |

### Publishers

| Topic | Type | 의미 |
| --- | --- | --- |
| `/dsr01/curobo/pick_pose` | `geometry_msgs/PoseStamped` | scan_executor → planner, target 1개씩 순차 전달 |
| `/dsr01/gripper/position_cmd` | `std_msgs/Int32` | 스캔 이동 시작 전 그리퍼 pre-close |
| `/strawberry/exploration/set_cell_state` | `std_msgs/String` | cell 상태(SCANNING/TARGET_FOUND/HARVESTED/SCANNED_EMPTY 등) → `workspace_marker_node` |
| `/strawberry/scan/status` | `std_msgs/String` | 사람이 읽는 진행 로그 |

### Services

| Service | Type | 역할 |
| --- | --- | --- |
| `/strawberry/scan/start` | `std_srvs/srv/Trigger` (server) | 스캔 시퀀스 명시적 시작(이 노드가 server) |
| `/dsr01/motion/move_spline_joint` | `dsr_msgs2/srv/MoveSplineJoint` (client) | (runtime cuRobo preview 진단용, 실행은 MoveJoint 사용) |
| `/dsr01/motion/move_joint` | `dsr_msgs2/srv/MoveJoint` (client) | YAML `endpoint_joints_deg` 직접 이동 (실제 셀 이동 수단) |

### 주요 ROS Parameters

| Parameter | 의미 |
| --- | --- |
| `execute_motion` | false면 등록만 하고 실제 이동 없음(dry-run) |
| `target_cell` | 순회 대상(`all` 또는 `root/nw` 등 단일 셀) |
| `manual_validation_mode` | 단일 셀 MoveJoint 검증만 허용 |
| `enable_pick_integration` | dwell 중 감지된 pose를 pick으로 전달할지 |
| `collect_then_pick` | true면 서브셀 전체 스캔 후 한 곳에서 모아 pick |
| `scan_dwell_sec` | 각 셀에서 안정 target을 기다리는 최대 시간 |
| `enable_runtime_curobo_preview` | 실행 전 cuRobo plan을 로그만 남기는 진단 모드 |

## 6. Gripper 서비스 계층

현재 권장 계층은 `dsr_gripper_tcp`의 `/gripper_service/*`다.

| Interface | 의미 |
| --- | --- |
| `/gripper_service/state` | position/current/status 주기 publish |
| `/gripper_service/set_position` | position 명령. 접근/놓기 시 600, 닫기 목표 700 |
| `/gripper_service/get_state` | present_position/current read |
| `/gripper_service/safe_grasp` | current delta/position 기반 close 중 grasp detect |

현재 한계:

- serial/DRL 초기화가 불안정하면 `set_position`/`SafeGrasp` timeout 발생.
- 얇은 줄기는 current delta가 낮아서 threshold 보정이 필요.
- `GRASP_EMPTY`는 jaw가 700까지 닫힌 상태를 의미하며, 실제 수확 실패에 가깝다.
- `GRASP_UNVERIFIED`는 통신 실패 또는 current/position 판정 불확실 상태다.

## 7. 런타임 로그/KPI

| 파일/도구 | 역할 |
| --- | --- |
| `logs/runtime/YYYY-MM-DD/*.jsonl` | planner/fusion raw runtime events |
| `reports/harvest_kpi/manual_labels_root_nw.csv` | 사람이 입력하는 최종 라벨 |
| `scripts/summarize_runtime_kpis.py --cell root/nw` | plan, cycle, result 자동 요약 |
| `scripts/generate_harvest_kpi_report.py --cell root/nw` | 표/그래프 리포트 생성 |

자동 기록 가능:

- plan latency
- IK fail / plan OK count
- selected grasp variant
- final approach distance/depth
- SafeGrasp result, present_position/current
- pick cycle time

사람 입력 필요:

- 실제 딸기가 분리됐는지
- 다른 줄기/잎을 같이 잡았는지
- retreat 중 유지됐는지
- tray place 성공 여부

## 8. 현재 리팩토링 진행상황

완료:

- `harvest_math.py` 분리: quaternion/vector helpers
- `harvest_grasp_orientation.py` 분리: published stem orientation -> roll-only candidate
- `harvest_motion_params.py` 분리: planner 상단의 실험 상수/티칭 pose/속도/거리값
- `doosan_motion_client.py` 분리: MoveSplineJoint/MoveJoint/MoveLine service wrapper
- `approach_retreat_policy.py` 분리: measured-TCP tool-finish undo 후 main approach reverse 계산
- `gripper_client.py` 분리: SetPosition/GetState/SafeGrasp wrapper와 grasp result 판정
- `gripper_client.py` 확장: horizontal approach 전 approach-open command/logging 흡수
- `gripper_client.py` 확장: close+verify command/result logging 흡수
- `tray_place_policy.py` 분리: marker tray target 로딩, taught grid pitch 보정, slot offset 계산
- `trajectory_guards.py` 분리: operational limit, J4/J6 equivalent normalization, spline jump/swing reject
- `curobo_planning_adapter.py` 분리: Cartesian/joint-space MotionGen plan 호출, plan logging, collision diagnostic
- `curobo_kinematics_adapter.py` 분리: FK ee pose와 trajectory line deviation diagnostic 계산
- `scene_obstacle_manager.py` 분리: dynamic cuboid, neighbor sphere, scene position, world update logging
- `grasp_candidate_policy.py` 분리: grasp offset/variant/depth probe/tie-break 정책
- `grasp_candidate_policy.py` 확장: 다음 `grasp_search_executor.py` 분리를 위한 `GraspSearchResult` 컨테이너 추가
- `grasp_candidate_policy.py` 확장: variant quaternion, approach direction, pre-approach endpoint 계산 분리
- `grasp_candidate_policy.py` 확장: legacy grasp endpoint 계산 분리
- `grasp_candidate_policy.py` 확장: measured-TCP best probe 갱신, legacy grasp 선택 상태 저장, leftmost depth-limited 판정 분리
- `harvest_result_policy.py` 분리: grasp result에서 place gate와 sequence result code를 결정하는 정책
- `open_stem_descent_policy.py` 분리: reached TCP Z와 KP1 Z 차이 기반 open-stem descent 계산
- `pick_target_policy.py` 분리: wall-Y clamp, bias 적용, NW high target, x/z guard 계산
- `place_sequence_policy.py` 분리: place status에서 hold/skip/continue state-machine action 결정
- `row2_place_policy.py` 분리: row2 place line deviation threshold 판정
- `marker_place_orientation_policy.py` 분리: marker place orientation/clearance 후보와 measured-TCP contact 보정 target 계산
- `approach_retreat_policy.py` 확장: measured-TCP adaptive approach distance, tilted branch horizontal tool-finish direction, cuRobo final fallback depth 후보 계산 분리
- `curobo_planner_node.py` 내부 helper화: open-stem descent, NW BASE+Y nudge, gripper-close failure recovery, detach/retreat, post-retreat place handling, return-to-scan completion, leftmost extra advance를 `_pick()` 본문에서 분리
- `curobo_planner_node.py`는 위 모듈을 import하도록 변경. 1차 분리로 약 1200줄 감소

**2026-06-21 (Claude Code) 추가 완료** — `HANDOFF_20260621_REFACTOR_FOR_CLAUDE_CODE.md` 5절 기준:

- `_try_final_approach_fallback(...)`: final approach 직선 진입 실패 시 cuRobo shallow fallback
  depth 탐색 루프를 `_pick()`에서 분리 (`9f03bff`)
- `_execute_final_approach(...)`: precomputed cuRobo 시도 → 직선 MoveLine → fallback 탐색 →
  실패시 abort까지 final approach 전체 시퀀스를 한 번에 묶음 (`b81f83e`)
- `grasp_search_executor.py` 신규 분리 (`5a42fec`): measured-TCP depth probe 루프와 legacy
  grasp offset 루프를 `GraspSearchExecutor` 클래스로 이동(node-dependent client 패턴).
  진행 중 `_run_measured_tcp_depth_probe`/`_try_legacy_grasp_offsets`를 먼저
  `curobo_planner_node.py` 내부 메서드로 분리(`c28057b`, `ddbd810`)한 뒤 모듈로 옮김
- 위 작업 중 CMakeLists.txt `install(PROGRAMS ...)`에 새 모듈을 안 넣으면 `colcon build`는
  통과해도 install space에 안 복사돼 `ros2 run` 시 `ImportError`가 나는 것을 발견·수정.
  새 모듈 추가 시 반드시 `ls ~/doosan_ws/install/.../lib/e0509_gripper_description/`로
  설치 확인할 것
- 전부 py_compile/diff --check/colcon build 통과 후 push 완료. **실기 미검증**(코드 이동만,
  로직/거리/속도/로그 이벤트명 무변경)
- `tray_place_executor.py` 신규 분리 (`edb67de`, 2026-06-21): `_execute_marker_place_after_retreat`
  + `_execute_taught_slot0_place_reference_after_retreat`(408줄, row2 분기 포함)를
  `TrayPlaceExecutor` 클래스로 이동. row2 known 이슈는 그대로 이동(고치지 않음). slot index
  증가(`self._marker_place_slot_idx += 1`)는 executor가 mutate하지 않고 호출부
  (`_maybe_execute_place_after_retreat`)로 끌어올림 — executor는 getter로만 읽음. 이제 안 쓰는
  `_load_marker_place_target`/`_taught_grid_slot_offset_m` wrapper와 import(`marker_place_orientation_policy`,
  `row2_place_policy`, `SciR`)도 함께 정리(`curobo_planner_node.py` 2,461→2,069 lines).
  py_compile/실제 import/diff --check/colcon build 전부 통과, **실기 미검증**.

**2026-06-21 (같은 날, 두 번째 라운드) — 전체 코드베이스 "남은 큰 메서드" 전수 분리**:

사용자가 "전부 다 진행해" 요청 → AST로 전체 파일 메서드 줄수 실측, 큰 것부터 순서대로 추가 분리:

- `_pick`(394줄, 이 시점 코드베이스 최대 단일 메서드) → `_prepare_pick_target_or_abort` +
  `_search_grasp`로 분리, 394→**248줄** (`d4f1dce`)
- `tray_place_executor.py`의 두 메서드(위 표 참조) — 183→130줄, 225→194줄 (`17cb269`)
- `scan_executor_node.py`의 `_scan_one_cell`(126줄) → `_move_to_scan_cell_and_wait` +
  `_process_cell_detections`로 분리, 126→**42줄** (`8970a55`)
- `__init__` 3종: `strawberry_fusion_node.py`(189→109줄, `fusion_bootstrap.py` 신규 —
  `planner_bootstrap.py`와 동일 패턴) (`10f0852`), `scan_executor_node.py`(165→105줄,
  in-class `_declare_and_load_params`, 이 패키지는 아직 모듈 분리 컨벤션 없음) (`56ab209`),
  `curobo_planner_node.py`(278→180줄, 순수 로깅만 `_log_startup_banner`로 분리, wiring
  180줄은 `self` 의존 심해 분리 보류 재확인) (`9ed5554`)

**최종 상태**: 전체 코드베이스 최대 단일 메서드 248줄(`_pick`). 남은 100줄대 메서드
(`_process_pose_detection` 246줄, `execute_taught_slot0_place_reference_after_retreat` 194줄,
curobo `__init__` 180줄, `declare_and_load_params` 140줄, `execute_marker_place_after_retreat`
130줄)는 공통적으로 "독립 실패조건 없는 단일 파이프라인이거나 순수 선언/로딩 나열"이라 더
쪼개면 변수만 여러 메서드에 분산되고 가독성은 오히려 떨어진다고 판단, 의도적으로 분리 보류.
모든 분리는 원본↔변환본 역치환 diff로 0줄 차이 확인 + py_compile/실제 import/git diff
--check/colcon build/install space 확인 전부 통과. **실기 전부 미검증.**

주의:

- `scripts/측정.py`는 절대 수정/커밋하지 않는다.
- SW 회귀 검증 없이 `main` 머지는 금지.
- 현재 debug branch는 NW motion debug용이다.
