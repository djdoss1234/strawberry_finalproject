# ISSUE-20260527-007: Scan Executor 실행 안전 게이트 누락

## 발견 내용

`scan_executor_node` 초안은 `/dsr01/joint_states`가 처음 들어오면 별도
승인 없이 `MoveJoint`와 `MoveSplineJoint`를 호출하도록 구현되어 있었다.
그런데 `v6` scan pose 결과는 collision sphere와 panel/tray obstacle이
없는 empty-world cuRobo dry-run 결과이고,
`config/scan_pose_candidates.yaml`도 `use_for_automated_motion: false`다.

## 위험

- RViz 확인 목적의 launch 실행이 실제 robot motion으로 이어질 수 있음
- empty-world `PLAN_VALID`가 실제 충돌 안전 검증으로 오해될 수 있음
- detector가 연결되지 않았는데 dwell 후 cell을 `SCANNED_EMPTY`로 기록할 수 있음
- overview pose에 도착했는지 확인하지 않고 다음 경로의 start state로 가정함

## 수정

- `workspace_scan.launch.py` 기본 실행은 visualization only로 변경
- executor는 `enable_robot_execution:=true`로 명시해야만 launch됨
- executor의 자동 시작을 제거하고 `/strawberry/scan/start` 명시 요청 추가
- 현재 collision-aware backend 미구현 상태에서는 motion authorization을
  항상 거부하도록 잠금
- 초기 자동 overview 이동 제거, 시작 시 실측 joint가 overview pose와
  `1.0 deg` 이내인지 확인
- detector 미연결 단계의 도착 상태를 `SCAN_POSE_REACHED`로 분리
- VLA interface의 offline `VALID`는 실기 실행 허가가 아님을 계약에 반영

## 현재 결론

`v6`는 자세 방향과 empty-world 계획 가능성 확인 자료로만 사용한다.
실기 scan 실행은 robot/tool collision model, panel world, start state
검증과 저속 단일 cell 절차가 완료되기 전까지 허용하지 않는다.
