# ISSUE-20260526-003: Workspace 외곽 치수와 개별 Cell 치수 불일치

## 상태

```text
PARTIALLY_RESOLVED
```

## 문제 현상 및 물리 구조 설명

종이 4분할 workspace의 outer dimension과 개별 cell dimension을 함께
측정한 결과, 합계가 일치하지 않습니다.

| 방향 | outer dimension | cell 합계 | 차이 |
| --- | ---: | ---: | ---: |
| 가로 | 1100 mm | `515 + 520 = 1035 mm` | 65 mm |
| 세로 | 800 mm | `365 + 365 = 730 mm` | 70 mm |

또한 화이트보드 안 종이 여백 합계는 보드 크기에 대해 가로 `+10 mm`,
세로 `+5 mm`의 차이가 있습니다.

사용자 확인 내용:

- 종이 네 장을 절연테이프로 연결해 하나의 4분할 탐색판을 만들었습니다.
- 이 큰 탐색판을 다시 절연테이프로 화이트보드에 부착했습니다.
- 따라서 outer dimension에는 중앙/외곽 테이프 영역이 포함되고, 개별
  종이 치수는 보이는 종이 면 또는 usable area일 가능성이 높습니다.
- 첨부 사진에서 외곽/중앙 테이프와 네 cell 구성이 확인되며, 테이프
  폭은 약 `20 mm`로 측정되었습니다.

방향별로 외곽 두 band와 중앙 band가 종이 usable 영역에서 제외된다고
보면 약 `20 x 3 = 60 mm`입니다. 측정된 차이인 가로 `65 mm`, 세로
`70 mm`와의 잔차는 각각 `5 mm`, `10 mm`로, 현재 작업영역 모델에는
충분히 일관된 수준입니다.

## 영향

- outer workspace와 테이프 교차점은 scan 영역 정의에 사용할 수 있습니다.
- 개별 종이 면의 usable area를 outer quadtree cell 경계로 그대로 쓰면
  중앙/외곽 테이프 band를 잃어버려 cell center 또는 motion margin
  해석이 어긋날 수 있습니다.

## 현재 처리

- `config/workspace.yaml`에는 outer dimension과 테이프 교차점 기준 bounds를 반영합니다.
- root 분할은 `(0, 0)` 테이프 교차점을 사용합니다.
- 개별 cell 치수는 visible/usable paper area로 기록합니다.
- 약 `20 mm`의 테이프 영역은 target을 두지 않는 경계/dead zone으로
  취급합니다.
- 해당 폭은 근사값이므로 safety margin 또는 정밀 scan target 계산에
  자동 반영하지 않습니다.

## 확인할 내용

1. 실제 motion 안전 margin을 도입하기 전, 필요한 경계별 tape overlap
   폭을 정밀 측정합니다.
2. overview camera와 RViz marker에서 outer/cell boundary 대응을 확인합니다.

## 검증 근거

- 발견 run: `RUN-20260526-002`
- 관련 config: `config/workspace.yaml`
- 부분 검증: outer bounds 및 tape split 반영 후 unit test 10개와
  `workspace_marker_node` 실행/초기 `root/nw` publish 확인
- 추가 근거: 약 `20 mm` tape 폭과 physical workspace 사진 확보
- 현재 판단: workspace geometry를 정의하기 위한 불일치는 설명되었고,
  motion margin 반영 여부만 후속 검증 대상으로 남음
- 해결 검증 run: overview/RViz 대응 및 실제 scan motion 전 갱신 예정

## 시각자료 계획

![외곽/중앙 tape band가 확인되는 물리 workspace](../assets/exploration/RUN-20260526-002_workspace_board.jpg)
