# RUN-20260527-008: Panel Registration Landmark Error 측정 계획

## 상태

```text
READY_TO_CAPTURE
```

## 목적

현재 `base_link -> cultivation_panel` 후보가 실제 테이프 기준점과 몇 mm
차이나는지 정량화한다. RViz에서 대략 맞아 보이는 것과 motion safety
margin으로 사용할 수 있는 정확도는 구분한다.

## 측정점

| Landmark | `cultivation_panel` 좌표 (m) | 화면에서 찍을 점 |
| --- | --- | --- |
| `origin_crossing` | `[0.000, 0.000, 0.000]` | 중앙 테이프 교차점 바로 옆 종이 표면 |
| `outer_nw` | `[-0.545, 0.395, 0.000]` | 좌상 외곽 tape/paper 경계 표면 |
| `outer_ne` | `[0.555, 0.395, 0.000]` | 우상 외곽 tape/paper 경계 표면 |
| `outer_sw` | `[-0.545, -0.405, 0.000]` | 좌하 외곽 tape/paper 경계 표면 |
| `outer_se` | `[0.555, -0.405, 0.000]` | 우하 외곽 tape/paper 경계 표면 |

검은 테이프 자체보다 테이프 경계에 인접한 흰 종이 표면을 클릭한다.
테이프의 반사/깊이 결측으로 생기는 오차를 줄이기 위함이다.

## 안전 조건

- Ubuntu 쪽 도구는 RGB-D 읽기와 YAML 저장만 한다.
- 로봇 자세가 필요하면 Windows DART에서 수동 조작한다.
- `panel_landmark_capture`는 motion topic/service를 호출하지 않는다.
- 결과가 좋아도 `use_for_automated_motion: false`를 유지한다.

## 실행 절차

1. DART에서 저장된 overview pose를 수동으로 맞춘다.

```yaml
joint_deg: [97.84, -94.40, 65.95, -10.93, 95.49, -94.79]
```

2. 다른 RealSense 사용 프로그램을 닫은 뒤, Ubuntu에서 실행한다.

```bash
cd ~/doosan_ws
source install/setup.bash
ros2 run strawberry_motion panel_landmark_capture -- \
  --calibration-file ~/doosan_ws/src/e0509_gripper_description/config/calibration_eye_in_hand_1.npz \
  --output ~/doosan_ws/src/strawberry_finalproject/docs/runs/RUN-20260527-008_panel_landmark_observations.yaml
```

3. 화면 지시에 따라 `origin_crossing -> outer_nw -> outer_ne -> outer_sw -> outer_se`
   순서로 클릭한다.
4. 잘못 찍었으면 `r`로 초기화한다. 다섯 점이 끝나면 `s`로 저장한다.
5. 저장된 `RMS error`와 `MAX error`를 worklog에 반영한다.

## 판정 기준

| 조건 | 판정 | 다음 행동 |
| --- | --- | --- |
| 5점 확보, RMS `<= 10 mm`, MAX `<= 15 mm` | `MEASURED_PASS_PENDING_MOTION_MARGIN` | 보드 margin 설정 검토 |
| 점 누락 또는 기준 초과 | `MEASUREMENT_INSUFFICIENT_OR_REQUIRES_RECAPTURE` | pose/depth/click 재측정 |

이 판정은 registration 관측 품질 기준이며, collision world 전체 또는
실제 motion 안전 승인 기준은 아니다.

<!-- VISUAL TODO
asset_id: RUN-20260527-008_panel_landmark_clicks
capture: 다섯 landmark가 표시된 read-only capture 화면과 terminal RMS/MAX 출력
source_path: artifacts/RUN-20260527-008/raw/
public_path: docs/assets/exploration/
use_in: GitHub README, 포트폴리오, Notion Run page
status: NOT_CAPTURED
-->
