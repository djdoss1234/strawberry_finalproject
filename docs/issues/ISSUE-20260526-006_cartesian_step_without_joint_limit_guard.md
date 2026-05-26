# ISSUE-20260526-006: Cartesian Step Control의 Joint Limit Guard 누락

## 상태

```text
CRITICAL_MITIGATED_IN_CODE_PENDING_ROBOT_RECOVERY
```

## 사고 내용

camera alignment를 편하게 하려는 목적으로
`realsense_alignment_viewer --enable-robot-control`에 Doosan
`MoveLine` relative translation을 연결했으나, 실행 전에 필요한
joint-limit, IK branch, collision 검증을 포함하지 않았습니다.

실제 로봇에서 alignment key 입력 후 joint limit 충돌로 로봇이 꺼지는
사건이 발생했습니다.

## 원인

- TCP의 작은 Cartesian translation이라도 현재 자세가 joint limit
  근처이면 IK 결과가 한계를 넘을 수 있습니다.
- 기존 미니프로젝트에서 겪었던 joint limit/IK branch 문제를 alignment
  utility에도 적용했어야 했지만 반영하지 않았습니다.
- `MoveLine` service가 요청을 받는다는 사실은 안전한 motion이라는
  보장이 아닙니다.

## 즉시 대응

- 해당 viewer에서 모든 robot motion service 호출을 제거합니다.
- `--enable-robot-control` 옵션은 이전 명령이 재사용되더라도 실행을
  거부하도록 fail-closed 처리합니다.
- direct viewer는 camera crosshair 표시 전용으로만 유지합니다.
- 로봇 상태 복구 및 alarm reset은 현장 안전 확인 후 별도로 수행하며,
  이 수정 작업 중 자동 복구 명령은 전송하지 않습니다.

## 재도입 조건

Cartesian UI 또는 MoveIt interactive marker 기반 조작을 다시 연결하려면
최소한 다음이 선행되어야 합니다.

1. 현재 joint state와 operational joint margin 검증
2. 목표 TCP pose에 대한 IK 해와 branch continuity 검증
3. wall/robot collision check
4. 실패 시 motion command 차단과 명확한 사용자 표시
5. simulation 또는 RViz 검증 후 저속 실기 승인

## 포트폴리오 기록 원칙

이 기능은 성공 기능으로 제시하지 않습니다. 현장 usability 개선을
서두르다가 safety validation boundary를 넘긴 실패이며, motion layer가
어떤 명령이든 실행 전 검증을 책임져야 한다는 설계 원칙으로 환류합니다.
