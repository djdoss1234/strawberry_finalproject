# miniproject -> finalproject 복구 기록 — 2026-06-18

## 배경

실제 로봇 수확 코드와 실험 기록 일부가 `strawberry_miniproject` remote에 계속 쌓여 있었고, 최종 기준 레포인 `strawberry_finalproject`에는 2026-06-03~2026-06-17 구간의 핵심 수확 진행상황이 비어 있었다.

기준점은 `e0509_gripper_description` 레포의 `afef927 feat: organize strawberry harvest mini-project portfolio` 커밋이다. 이 커밋 이후 변경분을 final project에 포함해야 할 실전 프로젝트 진행분으로 본다.

## 복구 방식

기존 `strawberry_motion` 패키지와 경로가 겹치지 않도록 실제 수확 패키지 전체 tracked snapshot을 아래 위치로 복구했다.

```text
ros_packages/e0509_gripper_description/
```

이 방식은 finalproject 루트의 기존 `package.xml`, `launch/`, `src/strawberry_motion/` 파일을 덮어쓰지 않는다. 이후 하나의 colcon workspace에서 finalproject repo를 사용할 때 `strawberry_motion`과 `e0509_gripper_description`을 함께 빌드할 수 있게 하는 목적이다.

## 복구된 패키지 스냅샷

- tracked files copied: 94
- files changed after afef927: 67
- excluded: untracked local files such as `scripts/측정.py`, build/install/log artifacts, raw runtime logs

## afef927 이후 핵심 변경 파일 목록

- `CMakeLists.txt`
- `README.md`
- `config/curobo/e0509_gripper.urdf`
- `config/curobo/e0509_gripper.yml`
- `config/curobo/e0509_gripper_measured_tcp.yml`
- `config/curobo/e0509_spheres.yml`
- `config/environment.yaml`
- `docs/GRIPPER_BIDIRECTIONAL_DIAGNOSIS_20260615.md`
- `docs/HANDOFF_20260611_MEASURED_TCP.md`
- `docs/HANDOFF_20260611_PLACE_CUROBO.md`
- `docs/HANDOFF_20260614_PLACE_TRAY_GRID.md`
- `docs/HANDOFF_20260615_SAFEGRASP_NW_NEXT.md`
- `docs/HANDOFF_20260617_NW_MOTION_GRIPPER_STATUS.md`
- `docs/HANDOFF_20260618_NW_MOTION_DEBUG.md`
- `docs/HARVEST_EXPERIMENT_OPERATION_PLAN_20260615.md`
- `docs/NOTION_UPDATE_AFTER_SW_HARVEST_20260615.md`
- `docs/NW_OCCLUSION_KPI_AND_GRASP_DIRECTION_20260615.md`
- `docs/SAFE_GRASP_STANDALONE_TEST_20260615.md`
- `docs/SIMULATION_INTERFACE_SPEC_20260618.md`
- `docs/close_range_perception_limit_20260604.md`
- `docs/experiment_results.md`
- `docs/gripper_automatic_grasp_verification.md`
- `docs/harvest_kpi_input_guide.md`
- `docs/harvest_motion_session_20260607.md`
- `docs/harvest_motion_session_20260608.md`
- `docs/harvest_motion_session_20260609.md`
- `docs/harvest_motion_session_20260611.md`
- `docs/harvest_motion_session_20260612.md`
- `docs/harvest_test_strategy_20260604.md`
- `docs/project_retrospective_portfolio_roadmap.md`
- `docs/runs/RUN-20260607-001_sw_horizontal_straight_approach.log`
- `docs/runs/RUN-20260609-001_sw_runtime_summary.png`
- `docs/runs/RUN-20260609-001_sw_runtime_summary_ko.png`
- `docs/runtime_pipeline_and_simulation_logs.md`
- `docs/sw_single_strawberry_harvest_notion_20260609.md`
- `docs/sw_single_strawberry_harvest_portfolio_20260609.md`
- `docs/sw_single_strawberry_harvest_retrospective_20260609.md`
- `docs/sw_single_strawberry_harvest_work_retrospective_20260609.md`
- `docs/system_architecture.md`
- `docs/tool_geometry_measurement_20260611.md`
- `docs/tray_localization_20260604.md`
- `include/e0509_gripper_description/modbus_rtu.hpp`
- `launch/bringup.launch.py`
- `package.xml`
- `reports/harvest_kpi/manual_labels_root_nw.csv`
- `scripts/check_harvest_logging.py`
- `scripts/clean_robot_runtime.sh`
- `scripts/collect_gripper_feedback.py`
- `scripts/curobo_planner_node.py`
- `scripts/diagnose_gripper_read.py`
- `scripts/generate_harvest_kpi_report.py`
- `scripts/generate_runtime_summary_plot.py`
- `scripts/joint_jog_control.py`
- `scripts/label_harvest_attempt.py`
- `scripts/prepare_harvest_label_sheet.py`
- `scripts/prime_gripper_serial_drl.py`
- `scripts/run_safe_grasp_trial.py`
- `scripts/runtime_jsonl_logger.py`
- `scripts/safe_grasp_ros_adapter.py`
- `scripts/set_experiment_context.py`
- `scripts/strawberry_fusion_node.py`
- `scripts/strawberry_yolo_node.py`
- `scripts/summarize_harvest_kpis.py`
- `scripts/summarize_runtime_kpis.py`
- `scripts/validate_runtime_jsonl.py`
- `src/gripper_service_node.cpp`
- `urdf/e0509_with_gripper.urdf.xacro`

## 핵심 내용 요약

- SW 단일 딸기 수확 모션 안정화: 2-step pre-approach, TOOL 직선 진입, extra advance, detach/retreat
- 계란판/tray place 시도: marker place, taught slot0, tray grid, row2 tilt/correction 실험
- measured TCP 전환: 실측 파지점/파츠 길이 반영, measured TCP profile 및 접근 거리 보정
- gripper/SafeGrasp/KPI: dsr_gripper_tcp 연동, position/current 기반 파지 판정 실험, KPI label/report 도구
- NW 잎/줄기 가림 셀 디버그: measured TCP final approach probing, MoveLine no-motion 진단, scan candidate handling
- 문서/포트폴리오 자료: SW 수확 회고, Notion 페이지, runtime summary, simulation interface spec, handoff 문서

## 남은 정리

1. `docs/worklogs/`에 2026-06-04~2026-06-17 날짜별 요약을 추가 백필한다.
2. finalproject README/architecture에서 `ros_packages/e0509_gripper_description` 포함 사실을 명시한다.
3. 중복 문서가 많아지면 finalproject 상위 docs에는 요약본을 두고, 상세 실험 문서는 package 내부 docs로 유지한다.
4. 이후 실전 프로젝트 기록은 `strawberry_finalproject`를 기준으로 남긴다.

## Canonical 작업 경로 전환

복구 후 실제 ROS workspace가 finalproject의 패키지를 사용하도록 다음과 같이 정리했다.

```text
/home/user/doosan_ws/src/e0509_gripper_description
 -> /home/user/doosan_ws/src/strawberry_finalproject/ros_packages/e0509_gripper_description
```

기존 miniproject 기반 작업 폴더는 다음 경로에 보존했다.

```text
/home/user/doosan_ws/src/e0509_gripper_description_legacy_miniproject_20260618
```

백업 폴더는 `COLCON_IGNORE`로 colcon discovery에서 제외했다. 앞으로 `e0509_gripper_description` 관련 새 수정과 커밋은 `strawberry_finalproject` 원격만 기준으로 한다.
