# ISSUE-20260526-004: Camera Overlay의 cv_bridge / NumPy Compatibility

## 상태

```text
MITIGATED
```

## 발견 상황

`camera_alignment_node`의 image 처리 방식을 정하기 위해 ROS image
dependency를 확인하는 과정에서, 현재 실행 환경의 `NumPy 2.x`와
ROS Humble에 설치된 `cv_bridge` binary 사이 compatibility 오류 메시지가
발생했습니다.

## 영향

십자선 overlay만을 위해 `cv_bridge`를 바로 의존하면, 현장 camera
정렬 단계가 Python package 조합에 의해 중단될 수 있습니다.

## 대응

- `camera_alignment_node`는 `cv_bridge`를 의존하지 않습니다.
- `sensor_msgs/Image`의 common 8-bit encoding buffer를 직접 읽고,
  OpenCV drawing 결과를 `bgr8` image message로 publish합니다.
- renderer를 ROS transport와 분리해 unit test가 가능하도록 구성했습니다.

## 검증

- overlay renderer unit test 통과
- 전체 unit test `12개` 통과
- 실제 RealSense topic 연결 검증은 physical alignment 실행 시 수행 예정

## 후속 판단

향후 detector/tray vision module에서 `cv_bridge`가 필요해지면 Python
환경과 ROS binary package의 NumPy ABI 조합을 먼저 정리합니다. 현재
overview alignment 단계에서는 기능 범위를 좁혀 우회하는 것이 타당합니다.
