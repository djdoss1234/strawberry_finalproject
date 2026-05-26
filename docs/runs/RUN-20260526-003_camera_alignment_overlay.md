# RUN-20260526-003: Camera 중앙 십자선 Overlay 구현

## 상태

```text
ROS_VERIFIED_PENDING_PHYSICAL_ALIGNMENT
```

## 목적

eye-in-hand RealSense 화면의 중심과 종이 workspace의 중앙 테이프
교차점을 쉽게 일치시키기 위해 RGB image 위에 정렬용 십자선을
표시합니다. 이 단계는 `scan_pose_generator`를 구현하기 전에 overview
camera pose를 반복 가능하게 확보하기 위한 보조 기능입니다.

## 구현 내용

추가 module:

- `visualization/alignment_overlay.py`: BGR image에 중앙 axes, 노란
  crosshair, view margin guide, 중심 pixel 안내 문구를 그리는 순수 renderer
- `visualization/camera_alignment_node.py`: ROS image input을 받고
  overlay image를 publish하는 node
- `visualization/realsense_alignment_viewer.py`: manual alignment 중
  latency를 낮추기 위해 RealSense color stream을 직접 표시하는 viewer
- `launch/camera_alignment.launch.py`: input/output topic을 변경할 수 있는
  실행 launch

ROS interface:

```text
/camera/camera/color/image_raw
  -> /camera_alignment_node
  -> /strawberry/alignment/overlay_image
```

parameters:

| parameter | 기본값 | 역할 |
| --- | --- | --- |
| `input_topic` | `/camera/camera/color/image_raw` | RealSense RGB 입력 |
| `output_topic` | `/strawberry/alignment/overlay_image` | 십자선 포함 출력 |
| `crosshair_length_px` | `60` | 중심 십자선 길이 |
| `line_thickness_px` | `2` | 중심 십자선 두께 |
| `guide_margin_ratio` | `0.08` | 외곽 여백 guide 비율 |

## 설계 판단

dependency 확인 중 ROS Humble의 `cv_bridge` binary와 현재 Python 환경의
`NumPy 2.x` 사이 compatibility 오류가 출력되는 것을 확인했습니다.
overview 정렬 보조 기능은 단순 RGB overlay이면 충분하므로, 이 node는
`cv_bridge`를 사용하지 않고 일반적인 `sensor_msgs/Image` encoding
(`bgr8`, `rgb8`, `bgra8`, `rgba8`, `mono8`)의 byte buffer를 직접
변환합니다.

또한 기존 미니프로젝트 perception node는 `pyrealsense2`로 camera를
직접 열기 때문에, alignment 실행 시 사용하는 `realsense2_camera`
driver와 동시에 실행하지 않습니다.

## 실행 방법

실제 DART 수동 조작 중 overview pose 정렬 화면에는 저지연 direct viewer를
사용합니다.

```bash
cd ~/doosan_ws
colcon build --packages-select strawberry_motion --symlink-install
source install/setup.bash

ros2 run strawberry_motion realsense_alignment_viewer
```

이 화면에서 노란 십자선을 종이 중앙 tape crossing에 맞추고 `q` 또는
`ESC`로 종료합니다.

아래 ROS image 경로는 topic/graph 통합 검증이나 overlay publish 증거가
필요할 때 사용합니다. 현장에서 지속적으로 자세를 조절하는 화면으로는
지연이 발생할 수 있습니다.

```bash
# Terminal 1: RGB topic publish
ros2 launch realsense2_camera rs_launch.py rgb_camera.color_profile:=640x480x30

# Terminal 2: ROS overlay publish
ros2 launch strawberry_motion camera_alignment.launch.py

# Terminal 3: ROS overlay view
ros2 run rqt_image_view rqt_image_view /strawberry/alignment/overlay_image
```

카메라 topic이 다르면:

```bash
ros2 launch strawberry_motion camera_alignment.launch.py \
  input_topic:=/your/color/image_raw
```

## 현재 검증

- `realsense2_camera`, `rqt_image_view` package 설치 확인
- renderer unit test 2개 추가
- 전체 unit test `12개` 통과
- 문법 검사 통과
- `colcon build --packages-select strawberry_motion --symlink-install` 성공
- `ros2 launch strawberry_motion camera_alignment.launch.py --show-args` 로딩 확인
- ROS node interface 확인:

```text
/camera/camera/color/image_raw
  -> /camera_alignment_node
  -> /strawberry/alignment/overlay_image
```

- synthetic `2x2 bgr8` image publish 후 overlay output이 `bgr8`,
  `height=2`, `width=2`, `step=6`으로 publish됨을 확인
- output data에 노란 crosshair pixel `(B,G,R)=(0,255,255)`가 포함됨을 확인
- ROS overlay 화면이 manual jog에서 끊긴다는 현장 feedback을 반영해
  direct viewer를 추가하고 `ISSUE-20260526-005`에 기록
- direct viewer CLI/default unit test 추가 후 전체 unit test `14개` 통과
- 기존 public workspace 사진에 renderer를 적용해 표시 결과 preview 생성

preview는 기능 설명용입니다. 실제 eye-in-hand 정렬 결과는
`RUN-20260526-002`에 기록했습니다.

![십자선 overlay renderer preview](../assets/exploration/RUN-20260526-003_crosshair_overlay_preview.jpg)

## 실기 확인 결과

- `realsense_alignment_viewer` direct stream으로 실제 eye-in-hand 영상을
  표시했습니다.
- Doosan DART 수동 조작으로 십자선과 종이 중앙 tape crossing을
  정렬했고 `RUN-20260526-002`에 결과 화면과 joint/TCP pose를 저장했습니다.
- 결과 화면에서 `LIVE 30.0 FPS`가 표시되어, 정렬 작업용 display
  응답성은 확보했습니다.

## 아직 확인하지 않은 것

- RViz quadtree marker와 camera view 대응
- ROS overlay topic을 실제 RealSense ROS stream과 연결한 evidence 화면
