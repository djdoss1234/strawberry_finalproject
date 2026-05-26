# ISSUE-20260526-001: Scan 순서와 ROS Node 종료 오류

## 상태

```text
RESOLVED
```

## 문제 현상

- `workspace_marker_node` 첫 실행 점검 중 발견했습니다.
- 초기 scan cell 선택이 문자열 정렬에 의존하면 설계한 quadrant 순서와
  달라질 수 있었습니다.
- node를 `Ctrl-C`로 종료했을 때 `rcl_shutdown already called` traceback이
  발생했습니다.

## 영향

- scan 순서가 불명확하면 이후 실제 robot motion의 재현성과 log 해석이 깨집니다.
- 정상 종료 시 traceback이 남으면 실제 node 실패와 사용자 종료를 구분하기 어렵습니다.

## 원인 분석

- next cell selection key에 `cell_id` 문자열이 포함되어 삽입 순서보다
  알파벳 순서가 우선될 수 있었습니다.
- ROS signal 처리 후 `finally` 블록이 다시 `rclpy.shutdown()`을 호출할 수 있었습니다.

## 해결

- 같은 priority/depth에서는 quadtree 생성 순서를 유지하도록 selection
  로직을 수정했습니다.
- node 종료 시 `rclpy.ok()`일 때만 shutdown하도록 수정했습니다.
- 초기 next cell이 `root/nw`인지 unit test assertion을 추가했습니다.

## 검증 근거

- 발견 run: `RUN-20260526-001`
- 해결 검증 run: `RUN-20260526-001`
- 관련 commit: `a95ca8e`
- 검증 결과: 초기 `root/nw`, 상태 갱신 후 `root/ne`, 종료 traceback 미발생

## 포트폴리오/면접에서 설명할 포인트

- 실제 motion에 연결하기 전에 scan scheduling의 결정성과 node lifecycle을
  먼저 검증해, 이후 하드웨어 실험에서 원인 분석이 흔들리지 않도록 했습니다.
