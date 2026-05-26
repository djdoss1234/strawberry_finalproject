# ISSUE-20260526-003: Workspace 외곽 치수와 개별 Cell 치수 불일치

## 상태

```text
OPEN
```

## 문제 현상

종이 4분할 workspace의 outer dimension과 개별 cell dimension을 함께
측정한 결과, 합계가 일치하지 않습니다.

| 방향 | outer dimension | cell 합계 | 차이 |
| --- | ---: | ---: | ---: |
| 가로 | 1100 mm | `515 + 520 = 1035 mm` | 65 mm |
| 세로 | 800 mm | `365 + 365 = 730 mm` | 70 mm |

또한 화이트보드 안 종이 여백 합계는 보드 크기에 대해 가로 `+10 mm`,
세로 `+5 mm`의 차이가 있습니다.

## 영향

- outer workspace와 테이프 교차점은 scan 영역 정의에 사용할 수 있습니다.
- 개별 cell 치수를 그대로 사용하면 cell center 또는 scan target이
  실제 영역과 어긋날 수 있습니다.

## 현재 처리

- `config/workspace.yaml`에는 outer dimension과 테이프 교차점 기준 bounds를 반영합니다.
- root 분할은 `(0, 0)` 테이프 교차점을 사용합니다.
- 개별 cell 치수는 의미가 확인되기 전까지 motion target 계산에 사용하지 않습니다.

## 확인할 내용

1. `515/520/365 mm`가 종이 한 장 자체 치수인지, 실제 탐색 가능 영역인지 확인합니다.
2. 테이프 폭과 종이 사이 gap/겹침을 측정합니다.
3. 정면 사진에 측정 기준선을 표시해 outer boundary와 cell boundary를 재확인합니다.

## 검증 근거

- 발견 run: `RUN-20260526-002`
- 관련 config: `config/workspace.yaml`
- 부분 검증: outer bounds 및 tape split 반영 후 unit test 10개와
  `workspace_marker_node` 실행/초기 `root/nw` publish 확인
- 해결 검증 run: 개별 cell 측정 경계 재확인 후 갱신 예정

## 시각자료 계획

<!-- VISUAL TODO
asset_id: ISSUE-20260526-003_workspace_measurement_boundaries
capture: 종이 4분할 외곽선, 중앙 테이프선, 개별 cell 측정 기준을 표시한 정면 사진
source_path: artifacts/RUN-20260526-002/raw/workspace_measurement_annotated.jpg
public_path: docs/assets/exploration/RUN-20260526-002_workspace_measurement_annotated.jpg
use_in: testbed 설계 기록, 필요 시 문제 해결 설명
status: NOT_CAPTURED
-->
