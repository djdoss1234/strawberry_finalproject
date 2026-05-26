# RUN-20260526-004: Camera 정렬용 Cartesian Step Control 구현

## 상태

```text
WITHDRAWN_AFTER_SAFETY_INCIDENT
```

## 문제

종이 workspace 중앙 교차점을 camera 십자선에 맞추기 위해 joint를 하나씩
조작하면, TCP가 어느 방향으로 움직일지 직관적이지 않고 posture도 쉽게
흐트러집니다. 정렬 단계에서는 end-effector orientation을 유지한 채
화면의 좌우/상하/거리 방향만 조금씩 보정하는 조작이 필요합니다.

## 시도했던 방법

`realsense_alignment_viewer` 화면에 Doosan `MoveLine` 기반 relative
Cartesian step control을 추가했으나, 실기 사고 이후 철회했습니다.

```text
RealSense direct viewer + crosshair
  -> keyboard step command
  -> /dsr01/motion/move_line (relative, DR_BASE)
  -> TCP position만 소량 이동, orientation 유지
```

당시 의도는 아래와 같았지만 안전 검증이 빠진 설계였습니다.

- 현재 목표는 trajectory 계획이 아니라 overview pose의 수동 정렬입니다.
- 연속 jog는 키 release를 놓치면 보드 가까이에서 위험할 수 있습니다.
- relative `MoveLine` step은 한 입력당 정해진 mm만 움직이고 완료를
  기다리도록 했으나, 이 조건만으로 joint limit 안전은 보장되지 않습니다.

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

## 실행 금지

robot control 옵션을 포함한 이전 실행 방법은 폐기했습니다. 현재
viewer는 십자선 camera 화면만 표시하며, motion 옵션을 전달하면 즉시
실행을 거부합니다.

## 현재 검증

- `/dsr01/motion/move_line` service 제공 확인
- `/dsr01/aux_control/get_current_posx` service 제공 확인
- 철회 전 keyboard -> relative displacement mapping test가 있었음
- 전체 unit test `15개` 통과
- 실물 로봇에서 실행 중 joint limit 충돌로 로봇이 꺼지는 사고가
  발생했습니다.

## 철회 결정

이 구현은 joint limit, IK branch, collision safety guard 없이
`MoveLine`을 호출하므로 사용하지 않습니다.

- 관련 safety issue: `ISSUE-20260526-006`
- immediate fix: viewer에서 robot motion 호출 제거 및
  `--enable-robot-control` fail-closed 처리
- 이후 방향: planner/safety validation을 통과하는 별도 motion
  interface를 설계하기 전까지 viewer는 camera 표시 전용으로 유지
