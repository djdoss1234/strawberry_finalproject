# RUN-20260527-009: 저속 단일 Cell Scan 실기 검증 계획

## 상태

```text
BLOCKED_PENDING_RUNTIME_COLLISION_BACKEND
```

2026-05-27 업데이트: SW 정책 재생성 완료 (panel_normal 0.55 m, RUN-011),
self-collision 검증 완료 (4셀 전체 PLAN_VALID, RUN-012), SE 정책도 panel_normal
0.65 m으로 변경. 남은 블로커: executor runtime collision backend 연결.

## 목적

자동 quadtree 순회를 열기 전에, 관절 margin이 상대적으로 여유 있는
`root/nw` 또는 `root/ne` 중 하나만 대상으로 저속 scan motion을 검증한다.

## 현재 실행 차단 조건

- `config/scan_pose_candidates.yaml`: `use_for_automated_motion: false`
- `config/scan_collision_world.yaml`: `use_for_automated_motion: false`
- executor collision-aware runtime backend: 아직 잠금 상태
- panel landmark refit: 흰 종이 기준 `RMS=9.229 mm`,
  `MAX=10.981 mm`; RViz 표시 확인 후 offline 기준 TF로 반영
- v7 refit + self-collision dry-run (RUN-012): 4셀 전체 `PLAN_VALID`
  SW=panel_normal 0.55 m, SE=panel_normal 0.65 m

## 후보 우선순위

| 순서 | Cell | Offline 결과 (RUN-012) | 이유 |
| --- | --- | --- | --- |
| 1 | `root/ne` | `PLAN_VALID` | initial gate 허용 cell |
| 2 | `root/nw` | `PLAN_VALID` | initial gate 허용 cell |
| 보류 | `root/sw` | `PLAN_VALID` | 초기 gate 대상 아님 (J5 129.6 deg) |
| 보류 | `root/se` | `PLAN_VALID` | 초기 gate 대상 아님 (J6 -219 deg) |

## 구현된 fail-closed 규칙

- 기본 launch는 RViz/preview만 수행한다.
- motion executor는 명시적으로 켜도 `/strawberry/scan/start` 전에는 정지한다.
- 시작 pose가 overview joint와 `1.0 deg` 이내가 아니면 거부한다.
- 첫 실기 단계에서는 `target_cell:=root/nw` 또는 `root/ne` 하나만 허용한다.
- current config/backend 승인 플래그가 false인 동안 start 요청은 계속 거부한다.

## 실행 허용 전 완료해야 할 일

1. ~~panel TF 오차를 반영한 collision margin 결정~~ RMS=9.229 mm 확인,
   motion margin은 executor 연결 시 결정
2. ~~`root/sw`의 새 standoff/approach 정책을 생성하고 offline 검증~~
   panel_normal 0.55 m PLAN_VALID (RUN-011, RUN-012)
3. executor가 `scan_collision_world.yaml`과 robot/tool sphere를 runtime에도
   실제로 로드하도록 연결  **← 현재 블로커**
4. 실행 전 offline 재검증 결과와 중단 절차 기록
5. 현장 E-stop/DART 정지 담당자와 clear-space 확인

이 문서는 실기 실행 명령서가 아니라, 실행 허가 전에 충족해야 하는
체크리스트다.
