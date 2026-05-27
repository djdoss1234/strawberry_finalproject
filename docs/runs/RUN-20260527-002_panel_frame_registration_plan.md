# RUN-20260527-002: Cultivation Panel Frame Registration 계획

## 상태

```text
PROCEDURE_DEFINED_PENDING_MARKER_SETUP
```

## 목적

현재 `cultivation_panel`은 종이판 중앙 테이프 교차점을 원점으로 하는
물리 작업영역 frame이지만, 아직 robot `base_link`에서 그 위치와 방향을
표현하는 transform은 없습니다.

이 run의 목적은 다음 transform을 안전하게 획득하고 검증하는 것입니다.

```text
base_link -> cultivation_panel
```

## 중요한 구분

전일 저장한 overview pose:

```yaml
tcp_base_mm_deg: [73.02, -122.02, 520.19, 86.27, 64.30, -89.05]
```

이 값은 카메라 화면 중심이 중앙 테이프 교차점과 일치했던 **로봇 자세**
기록입니다. TCP와 camera optical frame 사이 offset/rotation, camera에서
보드까지의 depth가 포함되지 않았으므로, 이 값을 곧바로
`base_link -> cultivation_panel` transform으로 쓰지 않습니다.

## 선택한 Registration 방식: Panel Marker Baseline

실제 프로젝트에서 보드 위치가 바뀌거나 scene을 재구성할 가능성을
고려해, 종이판에 고정한 AprilTag 또는 ArUco marker를 기준으로 frame을
재구성하는 방식을 baseline으로 사용합니다.

```text
camera image/depth
  -> marker pose in camera frame
  -> eye-in-hand camera transform
  -> marker pose in base_link
  -> known marker-to-panel offset
  -> base_link -> cultivation_panel
```

장점:

- 보드가 이동해도 다시 인식해 registration을 갱신할 수 있습니다.
- tray marker localization과 같은 원리로 확장할 수 있습니다.
- TCP로 보드에 직접 접근하거나 접촉하여 점을 찍는 위험을 피합니다.

## 준비할 것

1. 출력 가능한 AprilTag 또는 ArUco marker 1장
2. 실제 출력된 marker의 한 변 길이 측정값
3. marker를 붙일 위치:
   - 권장: 중앙 교차점 주변을 가리지 않는 화이트보드 여백
   - marker 중심에서 `cultivation_panel` 원점까지의 `X/Y` offset을 측정
4. marker가 overview camera 화면에 잘 보이는지 확인
5. 사용 중인 RealSense camera intrinsic과 eye-in-hand transform 확인

## Marker 부착 기준

marker는 종이 cell usable 영역이나 중앙 테이프 교차점을 가리지 않도록
화이트보드의 여백에 붙입니다. 부착 후 다음 값을 측정합니다.

```yaml
panel_marker:
  family_or_dictionary:
  marker_id:
  printed_size_m:
  center_offset_from_panel_origin_m:
    x:
    y:
    z: 0.0
```

축 부호는 기존 `cultivation_panel` 정의를 따릅니다.

```text
+X = 정면에서 볼 때 오른쪽
+Y = 정면에서 볼 때 위쪽
+Z = 보드 면에서 camera/robot 쪽
```

## 실행 및 검증 순서

1. marker를 고정하고 크기와 panel origin까지의 offset을 측정합니다.
2. overview pose에서 marker와 네 cell이 함께 보이는 RGB 화면을 확보합니다.
3. marker detector node는 pose를 publish만 하고 robot motion을 호출하지
   않도록 구현합니다.
4. camera/hand-eye TF를 이용해 `base_link -> cultivation_panel` 후보를
   계산합니다.
5. RViz에서 panel outline/cell marker를 해당 transform 아래에 표시합니다.
6. camera 화면의 실제 cell 방향과 RViz marker 방향을 비교합니다.
7. 별도의 저속 motion 검토는 frame 검증과 safety validation 이후로 미룹니다.

## 완료 기준

- marker 종류, ID, 출력 크기, panel origin과의 offset이 기록됩니다.
- marker detector가 camera frame 기준 pose를 안정적으로 제공합니다.
- 계산된 `base_link -> cultivation_panel` transform이 config/log에
  저장됩니다.
- RViz에서 물리 panel과 cell 방향/크기 대응이 설명 가능합니다.
- 이 단계에서는 자동 scan motion을 실행하지 않습니다.

## 시각자료 계획

| 자료 | 상태 | 저장 위치 |
| --- | --- | --- |
| marker 부착 위치와 측정 기준 사진 | `NOT_CAPTURED` | `docs/assets/exploration/` 후보 |
| marker가 함께 보이는 overview RGB | `NOT_CAPTURED` | `docs/assets/exploration/` 후보 |
| frame registration 후 RViz 비교 | `NOT_CAPTURED` | `docs/assets/exploration/` 후보 |
