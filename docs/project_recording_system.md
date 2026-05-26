# 프로젝트 진행 기록 체계

## 1. 왜 이렇게 기록하는가

이 프로젝트는 마지막에 결과를 회상해서 포트폴리오를 쓰는 방식이 아니라,
개발과 실험을 진행하는 동안 근거를 계속 축적하는 방식으로 관리합니다.

기록 목적:

- 실제로 구현한 것과 계획 중인 것을 구분합니다.
- 동작하지 않았던 시도와 해결 과정을 남깁니다.
- 코드 변경, ROS graph, 실험 결과, 영상 근거를 연결합니다.
- 포트폴리오와 자기소개서에 넣을 문장을 사실 기반으로 축적합니다.
- 새 세션, 팀원, 면접 준비 시 빠르게 맥락을 복원합니다.

## 2. 기록 단위

### Worklog: 하루의 진행 흐름

위치:

```text
docs/worklogs/YYYY-MM-DD.md
```

내용:

- 오늘 목표와 결정
- 작업한 기능
- 오늘 생성한 `RUN`/`ISSUE`
- 관련 commit
- 다음 작업

### Run Record: 실행 또는 검증 1회

위치:

```text
docs/runs/RUN-YYYYMMDD-NNN_<short_name>.md
```

예:

```text
RUN-20260526-001_workspace_marker_node.md
```

기록 대상:

- unit test 실행
- `colcon build`
- ROS node/topic/TF/RViz 점검
- 실제 로봇 motion 실행
- scene별 pick/place 반복 실험

하나의 run은 반드시 입력 조건, 실행 명령, 관찰 결과, 성공 판정, 다음
행동을 포함합니다.

### Issue Record: 문제 하나와 해결 과정

위치:

```text
docs/issues/ISSUE-YYYYMMDD-NNN_<short_name>.md
```

상태:

```text
OPEN
INVESTIGATING
RESOLVED
DEFERRED
```

기록 대상:

- build/runtime 오류
- 잘못된 scan/motion 동작
- collision, TF, coordinate, IK, planner 실패
- graph에서 발견한 중복 node 또는 interface 문제
- 해결했거나 아직 남아 있는 기술 문제

### Portfolio Evidence: 대외 설명용 근거 누적

위치:

```text
docs/portfolio_evidence.md
```

이 문서에는 다음만 넣습니다.

- 검증 완료된 구현 내용
- 수치 또는 재현 가능한 증거가 있는 결과
- 본인이 해결한 문제와 기술적 선택
- 자소서/면접에서 사용할 수 있는 문장 초안

계획만 있는 기능은 `향후 계획`으로 명확하게 구분합니다.

## 3. ID와 연결 규칙

식별자:

```text
RUN-YYYYMMDD-NNN
ISSUE-YYYYMMDD-NNN
```

연결 원칙:

- Worklog에는 그날 생성/갱신한 run과 issue 링크를 남깁니다.
- Run record에는 관련 commit과 발견한 issue를 연결합니다.
- Issue record에는 문제를 발견한 run과 해결을 검증한 run을 연결합니다.
- Portfolio evidence에는 증거가 되는 run, issue, commit만 인용합니다.

예시:

```text
Worklog
  -> RUN-20260526-001: workspace visualization node 실행 확인
       -> ISSUE-20260526-001: scan order/shutdown 문제 해결
       -> commit a95ca8e
  -> Portfolio Evidence: quadtree exploration 기반 구현
```

## 4. 실행 단계마다 작성하는 순서

### 구현 전

1. 이번 기능의 목적과 완료 기준을 worklog에 적습니다.
2. topic, frame, config, 안전 조건이 바뀌는지 적습니다.
3. 실제 로봇 실행이면 scene과 속도 제한을 먼저 기록합니다.

### 구현 중

1. 문제를 발견하면 숨기지 않고 issue를 생성합니다.
2. 임시 workaround인지 근본 해결인지 구분합니다.
3. ROS 연결이 바뀌면 `rqt_graph`, topic endpoint, TF/RViz 상태를 확인합니다.

### 구현 후

1. 실행/검증 결과를 run record로 남깁니다.
2. 해결한 issue는 원인, 수정, 검증 근거를 기록하고 `RESOLVED`로 바꿉니다.
3. 의미 있는 단위가 끝나면 commit/push합니다.
4. 대외적으로 설명할 만한 성과면 `portfolio_evidence.md`에 한 줄 추가합니다.

## 5. Git과 공개 자료 관리

GitHub에 올리는 것:

- source code와 config schema
- 공개 가능한 run/issue 요약
- 대표 architecture, graph, RViz 캡처
- 성능 결과표와 기술적 회고

GitHub에 올리지 않는 것:

- calibration 원본
- model weight
- raw image/video/rosbag 전체
- credential, token, 장비별 민감 설정

원본 artifact는 로컬 또는 별도 저장소에 보관하고, Git 문서에는 다음처럼
참조 정보만 남깁니다.

```text
evidence_asset: local-only / RUN-20260526-001 / rqt_graph_capture
```

## 6. Notion 정리 방식

Notion은 GitHub를 대신하는 source code 저장소가 아니라, 진행상황을
한눈에 보고 발표/자소서 재료를 모으는 dashboard로 사용합니다.

권장 페이지 구조:

```text
Strawberry Final Project
  - 프로젝트 개요 및 역할 분담
  - Roadmap / Milestone Board
  - Daily Worklogs
  - Experiment Runs
  - Issue & Solution Log
  - Portfolio / 자기소개서 Evidence Bank
```

권장 database 항목:

| Database | 주요 속성 |
| --- | --- |
| Experiment Runs | Run ID, Date, Stage, Scene, Result, Commit, Related Issue |
| Issue & Solution Log | Issue ID, Status, Cause, Resolution, Verified Run |
| Portfolio Evidence | Category, Claim, Evidence Link, Interview Keyword, Status |

동기화 원칙:

- 코드와 정확한 실행 기록의 원본은 Git 문서로 둡니다.
- Notion에는 진행 상태, 요약, 대표 근거 링크를 옮깁니다.
- Notion에서 문장을 다듬어도 성과 claim은 Git run/issue 근거와 일치시킵니다.

Notion 페이지 생성 도구가 연결되지 않은 동안에는 Git에 Notion-ready
형식으로 기록합니다. Notion을 연결한 세션에서는 이 구조를 기준으로
dashboard와 database를 생성하거나 갱신합니다.
