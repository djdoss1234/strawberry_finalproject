# ISSUE-20260526-002: 기존 Robot Graph의 Gripper Service Node 이름 중복

## 상태

```text
OPEN
```

## 문제 현상

- `RUN-20260526-001`에서 ROS graph를 점검하던 중 다음 경고를 관찰했습니다.
- `/dsr01/gripper_service_node`가 동일한 이름으로 두 번 노출되었습니다.

## 영향

- 현재 quadtree visualization 기능에는 직접 영향이 확인되지 않았습니다.
- 향후 motion/gripper 실행을 통합할 때 topic/service ownership을 혼동하거나
  예상하지 못한 command 처리 문제가 생길 수 있습니다.

## 확인할 내용

1. 기존 bringup/launch에서 동일 node가 중복 실행되는지 확인합니다.
2. node name만 같은지, 실제 service endpoint도 중복되는지 확인합니다.
3. motion baseline 이식 전에 어느 node를 사용할지 확정합니다.

## 검증 근거

- 발견 run: `RUN-20260526-001`
- 관련 commit: `dd3ce8d`의 worklog 기록

## 다음 조치

- motion baseline을 최종 저장소에 연결하기 전에 기존 launch와
  `ros2 service list/info`를 확인합니다.

## 시각자료 계획

| 자료 | 필요 장면 | 상태 | 원본 위치 | 사용처 |
| --- | --- | --- | --- | --- |
| 중복 node 증거 | `rqt_graph` 또는 node list에서 duplicate 경고/연결 확인 | `NOT_CAPTURED` | `artifacts/RUN-20260526-002/raw/duplicate_gripper_node.png` | Issue 분석, Notion |

<!-- VISUAL TODO
asset_id: ISSUE-20260526-002_duplicate_gripper_node
capture: 기존 bringup 상태에서 gripper_service_node 중복을 확인할 수 있는 graph 또는 terminal 화면
source_path: artifacts/RUN-20260526-002/raw/duplicate_gripper_node.png
public_path: docs/assets/motion/ISSUE-20260526-002_duplicate_gripper_node.png
use_in: 문제 해결 문서, 공개 필요성 판단 후 포트폴리오
status: NOT_CAPTURED
-->
