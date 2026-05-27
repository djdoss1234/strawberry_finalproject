# RUN-20260527-009: 저속 단일 Cell Scan 실기 검증 계획

## 상태

```text
BLOCKED_PENDING_PANEL_ERROR_AND_RUNTIME_COLLISION_BACKEND
```

## 목적

자동 quadtree 순회를 열기 전에, 관절 margin이 상대적으로 여유 있는
`root/nw` 또는 `root/ne` 중 하나만 대상으로 저속 scan motion을 검증한다.

## 현재 실행 차단 조건

- `config/scan_pose_candidates.yaml`: `use_for_automated_motion: false`
- `config/scan_collision_world.yaml`: `use_for_automated_motion: false`
- executor collision-aware runtime backend: 아직 잠금 상태
- panel landmark RMS/MAX error: 1차 측정 실패 (`RMS=120.874 mm`,
  `MAX=139.001 mm`), 흰 종이 면 기준 재측정 필요

## 후보 우선순위

| 순서 | Cell | Offline 결과 | 이유 |
| --- | --- | --- | --- |
| 1 | `root/ne` | `PLAN_VALID` | 주요 joint range가 운용 한계에서 비교적 여유 있음 |
| 2 | `root/nw` | `PLAN_VALID` | J1 이동량은 있으나 SW/SE보다 margin 양호 |
| 보류 | `root/sw` | `PLAN_VALID` | `J1=203.77 deg`, `J5=124.70 deg`로 limit 인접 |
| 보류 | `root/se` | `PLAN_VALID` | `J6=-223.67 deg`로 limit 인접 |

## 구현된 fail-closed 규칙

- 기본 launch는 RViz/preview만 수행한다.
- motion executor는 명시적으로 켜도 `/strawberry/scan/start` 전에는 정지한다.
- 시작 pose가 overview joint와 `1.0 deg` 이내가 아니면 거부한다.
- 첫 실기 단계에서는 `target_cell:=root/nw` 또는 `root/ne` 하나만 허용한다.
- current config/backend 승인 플래그가 false인 동안 start 요청은 계속 거부한다.

## 실행 허용 전 완료해야 할 일

1. `RUN-20260527-008` landmark error 측정 완료
2. panel TF 오차를 반영한 collision margin 결정
3. executor가 `scan_collision_world.yaml`과 robot/tool sphere를 runtime에도
   실제로 로드하도록 연결
4. 실행 전 offline 재검증 결과와 중단 절차 기록
5. 현장 E-stop/DART 정지 담당자와 clear-space 확인

이 문서는 실기 실행 명령서가 아니라, 실행 허가 전에 충족해야 하는
체크리스트다.
