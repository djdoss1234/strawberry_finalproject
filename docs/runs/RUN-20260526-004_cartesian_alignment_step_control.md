# RUN-20260526-004: Camera 정렬용 Cartesian Step Control 구현

## 상태

```text
IMPLEMENTED_PENDING_PHYSICAL_MOTION_TEST
```

## 문제

종이 workspace 중앙 교차점을 camera 십자선에 맞추기 위해 joint를 하나씩
조작하면, TCP가 어느 방향으로 움직일지 직관적이지 않고 posture도 쉽게
흐트러집니다. 정렬 단계에서는 end-effector orientation을 유지한 채
화면의 좌우/상하/거리 방향만 조금씩 보정하는 조작이 필요합니다.

## 선택한 방법

`realsense_alignment_viewer` 화면에 Doosan `MoveLine` 기반 relative
Cartesian step control을 추가했습니다.

```text
RealSense direct viewer + crosshair
  -> keyboard step command
  -> /dsr01/motion/move_line (relative, DR_BASE)
  -> TCP position만 소량 이동, orientation 유지
```

MoveIt interactive marker나 연속 `Jog`를 바로 쓰지 않은 이유:

- 현재 목표는 trajectory 계획이 아니라 overview pose의 수동 정렬입니다.
- 연속 jog는 키 release를 놓치면 보드 가까이에서 위험할 수 있습니다.
- relative `MoveLine` step은 한 입력당 정해진 mm만 움직이고 완료를
  기다려, 초기 물리 검증에 더 보수적입니다.

## Key Mapping

기존 whiteboard 측정 기준에서 보드 면과 가까운 base axis 관계를 사용합니다.

| key | 상대 이동 | 정렬 의미 |
| --- | --- | --- |
| `A` | `base X -` | 화면 좌측 후보 |
| `D` | `base X +` | 화면 우측 후보 |
| `W` | `base Z +` | 화면 위쪽 후보 |
| `S` | `base Z -` | 화면 아래쪽 후보 |
| `R` | `base Y +` | 보드 거리 조정 후보 |
| `F` | `base Y -` | 보드 거리 조정 후보 |
| `P` | 없음 | 현재 TCP pose 출력 |
| `Q` / `ESC` | 없음 | viewer 종료 |

카메라 장착 방향과 로봇의 실제 자세에 따라 화면 기준 좌우 또는 거리
부호가 반대로 느껴질 수 있으므로, 첫 실기에서는 반드시 보드에서 충분히
떨어진 상태에서 `2 mm` step 한 번씩으로 방향을 확인합니다.

## 실행 방법

기존 camera 사용 process와 joint jog process를 종료한 뒤 실행합니다.

```bash
cd ~/doosan_ws
source install/setup.bash
ros2 run strawberry_motion realsense_alignment_viewer -- \
  --enable-robot-control \
  --step-mm 2 \
  --linear-velocity-mm-s 10 \
  --linear-acceleration-mm-s2 20
```

축 방향을 확인하고 충분히 안전하면 `--step-mm 5`로 재실행해 정렬을
마무리합니다.

## 현재 검증

- `/dsr01/motion/move_line` service 제공 확인
- `/dsr01/aux_control/get_current_posx` service 제공 확인
- keyboard -> relative displacement mapping unit test 추가
- 전체 unit test `15개` 통과
- 실물 로봇 이동 검증은 안전상 사용자가 화면을 확인하는 현장 실행에서 수행

## 현장 완료 기준

1. `2 mm` step으로 `A/D`, `W/S`, `R/F` 실제 화면 방향을 확인합니다.
2. viewer에서 중앙 십자선을 tape crossing에 일치시킵니다.
3. `P`로 TCP pose를 출력하고 기록합니다.
4. 정렬 화면 캡처와 pose를 overview registration evidence로 저장합니다.
