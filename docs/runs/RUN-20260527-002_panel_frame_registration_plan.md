# RUN-20260527-002: Cultivation Panel Frame Registration 계획

## 상태

```text
RVIZ_DISPLAY_CONFIRMED_PENDING_PHYSICAL_ERROR_VALIDATION
```

## 목적

종이판 중앙 테이프 교차점을 원점으로 하는 `cultivation_panel`의
`base_link` 기준 transform 후보를 획득한다.

```text
base_link -> cultivation_panel
```

## 방식: 기존 Hand-Eye Calibration + Depth

보드에는 추가 marker를 붙이지 않는다. 미니프로젝트에서 이미 사용했던
eye-in-hand calibration과 overview alignment pose를 그대로 활용한다.

```text
저장된 overview joint pose
  -> E0509 FK
  -> T_base_camera = T_base_tcp @ T_tcp_camera
  -> 중앙 십자선 / 우측 / 상측 depth point를 base_link로 변환
  -> cultivation_panel 원점과 +X/+Y/+Z 축 추정
```

좌표축은 기존 정의를 유지한다.

```text
+X = 보드 정면에서 오른쪽
+Y = 보드 정면에서 위쪽
+Z = 보드 면에서 camera/robot 쪽
```

## 안전 조건

- 이 도구는 RealSense 영상을 읽고 transform 후보를 출력만 한다.
- robot service, jog, planner, trajectory 실행을 호출하지 않는다.
- 로봇 자세 복귀가 필요하면 DART에서 수동으로 수행한다.
- 출력값은 RViz 검증 전 자동 motion 입력에 사용하지 않는다.

## 실행 순서

1. DART에서 저장된 overview pose로 맞추고, 십자선과 테이프 교차점이 맞는지 확인한다.
2. 다른 RealSense 사용 프로세스를 종료한다.
3. 아래 명령으로 read-only capture viewer를 실행한다.

```bash
cd ~/doosan_ws
source install/setup.bash
ros2 run strawberry_motion panel_frame_capture -- \
  --calibration-file ~/doosan_ws/src/e0509_gripper_description/config/calibration_eye_in_hand_1.npz
```

4. 화면 정렬 상태에서 `p`를 눌러 transform 후보를 출력한다. `q`로 종료한다.
5. 후보 transform을 RViz의 panel/cell marker에 적용해 방향과 위치를 검증한다.

## 완료 기준

- 동일 overview pose에서 panel TF 후보가 출력된다.
- 사용한 calibration 파일, pose, 출력 matrix, 화면 캡처를 run evidence로 저장한다.
- RViz에서 실제 보드의 좌우/상하 방향과 cell 배치가 일치한다.
- 이 단계에서는 자동 scan motion을 실행하지 않는다.

## 측정 결과

운용 제한 안의 overview pose에서 `panel_frame_capture` 출력값을 확보했다.

```yaml
translation_m: [0.112833, 0.597764, 0.643978]
rotation_matrix:
  - [0.965583, -0.018735, -0.259418]
  - [-0.259467, -0.000147, -0.965752]
  - [0.018056, 0.999824, -0.005003]
rotation_xyzw: [0.701914023, -0.099086927, -0.085965950, 0.700077382]
```

- 저장 위치: `config/panel_registration.yaml`
- 현재 상태: RViz에서 `base_link` 기준 panel/cell 표시 확인, 물리 오차 정량 검증 전
- 사용 제한: 자동 motion 입력 금지

## 시각자료 계획

| 자료 | 상태 | 저장 위치 |
| --- | --- | --- |
| 십자선과 중앙 교차점 정렬 화면 | `CAPTURED` | `docs/assets/exploration/` |
| TF 후보 출력 terminal 값 | `RECORDED` | `config/panel_registration.yaml` |
| TF 적용 후 RViz 비교 화면 | `CAPTURED` | `docs/assets/exploration/RUN-20260527-002_registered_panel_rviz.png` |
