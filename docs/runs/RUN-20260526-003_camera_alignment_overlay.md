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

```bash
cd ~/doosan_ws
colcon build --packages-select strawberry_motion --symlink-install
source install/setup.bash

# Terminal 1
ros2 launch realsense2_camera rs_launch.py rgb_camera.color_profile:=640x480x30

# Terminal 2
ros2 launch strawberry_motion camera_alignment.launch.py

# Terminal 3
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
- 기존 public workspace 사진에 renderer를 적용해 표시 결과 preview 생성

preview는 기능 설명용이며 실제 eye-in-hand 정렬 검증은 아닙니다.

![십자선 overlay renderer preview](../assets/exploration/RUN-20260526-003_crosshair_overlay_preview.jpg)

## 아직 확인하지 않은 것

- 실제 RealSense stream에서 overlay image publish 및 표시
- 십자선과 종이 중앙 tape crossing 정렬 결과 화면
- 정렬 완료 pose의 robot joint/TCP 값
- RViz quadtree marker와 camera view 대응

## 다음 현장 절차

1. 기존 `pyrealsense2` 기반 perception node가 실행 중이면 종료합니다.
2. RealSense ROS driver와 `camera_alignment_node`를 실행합니다.
3. overlay 화면에서 종이 네 cell 외곽이 여백 안에 들어오게 합니다.
4. 노란 십자선을 중앙 테이프 교차점에 맞춥니다.
5. overlay 화면과 현재 joint/TCP pose를 저장합니다.
6. 결과를 `RUN-20260526-002` overview 정렬 기록과 연결합니다.
