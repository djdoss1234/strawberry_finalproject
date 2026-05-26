# ISSUE-20260526-005: ROS Overlay 화면 지연으로 Manual Alignment 곤란

## 상태

```text
MITIGATED_PENDING_PHYSICAL_TEST
```

## 발견 상황

overview pose를 현장에서 맞추는 중, `realsense2_camera`와
`camera_alignment_node`, `rqt_image_view`를 잇는 ROS image 경로의
화면이 끊겨 실시간으로 robot pose를 계속 조정하기 어렵다는 피드백이
발생했습니다.

기존 경로:

```text
RealSense
  -> ROS driver image publish
  -> camera_alignment_node image copy/overlay/re-publish
  -> rqt_image_view display
```

## 영향

- 작업자가 jog로 camera pose를 조정할 때 visual feedback이 늦습니다.
- 중앙 tape crossing에 십자선을 정확히 맞추는 작업이 불편해집니다.
- 정렬 보조 기능 자체는 맞더라도 현장 usability가 낮아집니다.

## 대응

실시간 manual alignment 전용으로 `realsense_alignment_viewer`를
추가했습니다.

```text
RealSense color stream
  -> pyrealsense2 direct capture
  -> OpenCV crosshair rendering/display
```

- ROS image serialization/re-publish와 `rqt_image_view`를 거치지 않습니다.
- color stream만 `640 x 480 @ 30 fps`로 열어 정렬 중 처리량을 줄입니다.
- 같은 `alignment_overlay.py` renderer를 사용하므로 십자선 의미는
  기존 ROS overlay와 동일합니다.
- 화면 하단에 관측 display FPS를 표시합니다.

## 역할 분리

| 도구 | 사용 목적 |
| --- | --- |
| `realsense_alignment_viewer` | 실제 robot jog 중 저지연 정렬 |
| `camera_alignment_node` | ROS topic 연결, 향후 기록/통합 검증 |

두 경로 모두 RealSense camera를 사용하는 다른 process와 동시에 실행하지
않습니다. 특히 기존 `pyrealsense2` 기반 detector와 direct viewer는
동시에 camera를 열 수 없습니다.

## 검증

- viewer argument/default unit test 추가
- 전체 unit test `14개` 통과
- 문법 검사 통과
- 실제 camera에서 체감 지연 및 FPS 확인은 다음 physical alignment에서 수행

## 다음 확인

1. `ros2 run strawberry_motion realsense_alignment_viewer`를 실행합니다.
2. robot jog 중 화면 지연이 alignment 작업에 충분히 낮은지 확인합니다.
3. 정렬 완료 화면과 joint/TCP pose를 저장합니다.
4. 여전히 지연이 크면 `848x480@60` 지원 여부 또는 USB/RealSense
   stream 상태를 추가 확인합니다.
