# 시각자료 촬영, 보관, 문서 배치 규칙

## 1. 목적

사진과 영상은 장식이 아니라, 실제로 무엇을 구현했고 어떤 문제가 있었으며
어떻게 개선했는지를 증명하는 자료로 관리합니다.

모든 시각자료는 다음 중 하나에 해당해야 합니다.

- 시스템 구성 설명: 로봇, camera, 딸기 모형, tray, marker 위치
- 구현 증거: RViz, `rqt_graph`, TF tree, node/topic 동작
- 실험 결과: 성공/실패 장면, scene별 비교, tray 이동 전후
- 문제 해결: 잘못된 경로, 충돌 위험, 수정 전후 비교
- 최종 발표 자료: end-to-end demo와 결과표

## 2. 저장 위치

### 원본 자료: Git에 올리지 않음

raw 이미지, 전체 영상, rosbag, 다량 캡처는 프로젝트 root의
`artifacts/` 아래에 보관합니다. 이 폴더는 `.gitignore`로 제외합니다.

```text
artifacts/
  RUN-20260526-001/
    raw/
      rqt_graph_full.png
      rviz_full.png
      terminal_output.txt
      demo_recording.mp4
```

원본 자료의 장점:

- 해상도와 전체 맥락을 보존함
- 공개하기 어려운 장비 정보나 배경이 있어도 로컬 근거로 유지 가능함
- 필요할 때 공개용 자료를 잘라내거나 블러 처리할 수 있음

### 공개 대표 자료: Git에 포함 가능

포트폴리오와 GitHub README에서 보여줄 가치가 있고 개인정보/민감 설정이
없는 자료만 `docs/assets/` 아래에 추가합니다.

```text
docs/assets/
  architecture/
  exploration/
  motion/
  tray/
  integration/
  results/
```

권장 파일명:

```text
RUN-20260526-001_rqt_graph_workspace_node.png
RUN-20260526-001_rviz_quadtree_cells.png
RUN-20260530-002_scan_pose_markers.png
RUN-20260610-001_tray_relocation_place_success.mp4
```

규칙:

- 파일명 앞에 관련 `RUN ID`를 붙입니다.
- 공개 자료는 가능한 한 `.png`, 짧은 `.gif`, 압축된 짧은 `.mp4`로 관리합니다.
- 장비 IP, 토큰, 개인 얼굴, 불필요한 터미널 정보가 보이면 공개하지 않거나 편집합니다.
- 긴 원본 영상 대신 핵심 구간만 공개합니다.

## 3. 어떤 시점에 무엇을 찍을지

| 개발 단계 | 반드시 확보할 자료 | 문서에 넣을 위치 |
| --- | --- | --- |
| Quadtree core/visualization | `rqt_graph`, RViz cell marker, 상태 전환 전후 | run record, evidence bank, README 진행상태 |
| Scan pose 생성 | cell center와 camera pose marker가 함께 보이는 RViz 화면 | run record, architecture 설명 |
| Robot scan motion | 로봇이 cell 순서대로 관찰하는 영상, graph | run record, portfolio motion section |
| Detector 연결 | 검출된 target과 해당 cell state 변화 화면 | run record, perception-to-motion 설명 |
| Tray 자동 place | tray marker/slot RViz, tray 위치 A/B/C place 영상 | result report, portfolio 핵심 성과 |
| Collision/retry | 충돌 위험 수정 전후 비교 캡처/영상 | issue record, 문제 해결 카드 |
| VLA 통합 | VLA proposal, motion validation, 실행 결과 흐름 graph | integration section |
| 최종 demo | 전체 수확 시연, 지표표, 최종 architecture | README, portfolio, 발표자료 |

## 4. 문서 안에 placeholder를 넣는 방식

아직 자료가 없을 때는 빈 설명으로 두지 않고, 들어갈 위치에 HTML comment를
남깁니다. GitHub Markdown 화면에서는 보이지 않지만 편집할 때 해야 할
촬영 항목을 놓치지 않을 수 있습니다.

형식:

```markdown
<!-- VISUAL TODO
asset_id: RUN-20260526-001_rviz_quadtree_cells
capture: RViz에서 quadtree 4개 cell과 next cell marker가 표시된 화면
source_path: artifacts/RUN-20260526-001/raw/rviz_full.png
public_path: docs/assets/exploration/RUN-20260526-001_rviz_quadtree_cells.png
use_in: GitHub README, 포트폴리오 탐색 모듈 설명, Notion Run page
status: NOT_CAPTURED
-->
```

자료 확보 후에는 다음과 같이 이미지 또는 링크로 교체합니다.

```markdown
![Quadtree workspace cells in RViz](../assets/exploration/RUN-20260526-001_rviz_quadtree_cells.png)
```

영상은 GitHub에 넣을지 여부를 용량과 공개 범위에 따라 판단하고, 공개하지
않으면 run record에는 원본 위치만 남깁니다.

```markdown
video_asset: local-only / artifacts/RUN-20260610-001/raw/tray_place_success.mp4
```

## 5. Run Record에 들어갈 시각자료 블록

각 `RUN` 문서에는 아래 네 항목을 둡니다.

| 항목 | 내용 |
| --- | --- |
| 필수 캡처 | 이번 run에서 반드시 남겨야 했던 화면/영상 |
| 확보 상태 | `NOT_CAPTURED`, `LOCAL_ONLY`, `PUBLIC`, `NOT_REQUIRED` |
| 원본 위치 | `artifacts/RUN-.../raw/...` |
| 공개 위치 | `docs/assets/...` 또는 `N/A` |

실패 run도 자료 가치가 있습니다. 실패 원인이 화면으로 설명 가능하다면
성공 영상만큼 중요하게 보관합니다.

## 6. Issue Record에 들어갈 자료

문제가 시각적으로 드러나는 경우 다음을 우선 확보합니다.

- 수정 전 증상 화면 또는 영상
- 수정 후 같은 조건의 비교 화면 또는 영상
- `rqt_graph`, RViz planning scene, terminal error 중 원인을 뒷받침하는 것

포트폴리오용 문제 해결 카드에서는 다음 구성이 가장 좋습니다.

```text
문제 현상 캡처
  -> 원인 분석 그림/graph
  -> 수정한 코드 또는 설계 요약
  -> 해결 후 검증 영상/결과표
```

## 7. Portfolio Evidence와 Notion 배치

`docs/portfolio_evidence.md`에는 대표 자료만 연결합니다.

- 한 기능당 핵심 캡처 1~2장
- 핵심 demo 영상 1개
- 수치 결과표 또는 실패 분류표 1개

Notion에서는 다음 방식으로 배치합니다.

| Notion 페이지 | 넣을 자료 |
| --- | --- |
| 프로젝트 개요 | 전체 hardware/testbed 사진, architecture diagram |
| Milestone | 단계별 대표 RViz/robot 실행 캡처 |
| Experiment Run | 해당 run의 영상/이미지와 결과 |
| Issue & Solution | 수정 전후 비교 자료 |
| Portfolio Evidence | 공개 가능한 대표 자료와 설명 문장 |

Git 문서에는 자료의 사실 관계와 경로를 유지하고, Notion에는 보는 사람이
흐름을 빠르게 이해할 수 있도록 대표 이미지를 배치합니다.
