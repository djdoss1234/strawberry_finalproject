# 딸기 수확 로봇 최종 프로젝트 (Strawberry Final Project)

실제에 가까운 딸기 모형 환경에서 반복 가능한 수확 동작을 구현하기 위한
딸기 수확 로봇 최종 프로젝트 저장소입니다.

이 프로젝트는 미니프로젝트에서 검증한 `RealSense + YOLO + cuRobo +
Doosan E0509` 기반 pick & place pipeline을 출발점으로 삼아, 변화하는
환경에서도 측정 가능하고 확장 가능한 수확 시스템으로 고도화합니다.

## 프로젝트 역할 분담

| 영역 | 담당 | 범위 |
| --- | --- | --- |
| 수확 모션 시스템 | djdoss1234 | approach, grasp, retreat, transfer, tray placement, planning/execution 통합, 실패 복구, 성능 평가 |
| 복잡한 환경의 VLA 수확 판단 | 팀원 | 가림/복잡 장면 해석, 수확 대상 판단, high-level action 제안 |
| 통합 실험 | 공동 | target/action interface 합의, 실험 protocol, end-to-end demo |

핵심 원칙은 **VLA가 로봇 trajectory를 직접 실행하지 않는 것**입니다.
VLA는 수확할 대상이나 다음 행동을 제안할 수 있지만, 실제 하드웨어 명령
전에는 모션 시스템이 target pose, workspace, collision, 실행 가능성을
검증합니다.

## 이전 미니프로젝트 기준 구현

기존 구현과 실험 기록은 다음 저장소에 정리되어 있습니다.

- [strawberry_miniproject](https://github.com/djdoss1234/strawberry_miniproject)

미니프로젝트에서 구현하거나 검증한 기반 기능:

- RGB-D 기반 딸기 후보 검출과 3D target 생성
- eye-in-hand coordinate transform
- cuRobo 기반 approach, grasp, retreat planning
- Doosan 로봇 실행과 gripper 제어
- 티칭된 tray slot으로의 place 동작
- 실험 결과, 문제점, 개선 방향 문서화

최종 프로젝트에서는 실험용 코드 전체를 바로 섞지 않고, 안정화된 기능을
필요한 순서대로 옮기며 모듈 구조와 평가 체계를 새로 잡습니다.

## 내 담당 범위: Motion System

이 저장소에서 우선 구현하고 검증할 내용:

- 실제형 딸기 모형에 대한 안정적인 수확 motion sequence
- planning, execution, task sequence 모듈화
- tray 위치 인식과 자동 place target 생성
- 딸기, tray, 장애물, 이미 배치된 과실을 고려한 collision world 관리
- 실패 유형별 retry/recovery policy
- planner 비교 및 trajectory 품질 분석
- 팀원의 VLA 판단 결과를 수신하는 integration interface

## 첫 기능 목표

첫 번째 기능 milestone은 **움직인 계란판에 대한 자동 place**입니다.

> 사람이 계란판 또는 수확 tray의 위치를 바꾼 뒤에도, 로봇이
> AprilTag/ArUco 또는 RGB-D 기반으로 tray pose를 다시 인식하고,
> 자동 생성한 빈 slot에 딸기 모형을 배치한다.

이 기능을 먼저 구현하는 이유:

- 미니프로젝트의 고정 티칭 의존성을 제거할 수 있음
- 환경 변화 대응 능력을 수치로 평가할 수 있음
- VLA 구현 전에 모션 시스템 자체의 신뢰성을 검증할 수 있음
- 포트폴리오에서 미니프로젝트 대비 발전점을 분명히 보여줄 수 있음

## 초기 진행 순서

1. 하드웨어 구성, 실제형 모형 환경, 작업 정의, 평가 지표를 확정합니다.
2. 이동 가능한 tray와 marker 기반 localization 테스트베드를 만듭니다.
3. 미니프로젝트에서 안정적으로 동작한 motion 기능을 선별해 이식합니다.
4. `tray_frame` 기준 slot 자동 생성과 place 평가 기능을 구현합니다.
5. planner 비교와 실패 원인 기록 체계를 만듭니다.
6. 검증된 interface를 통해 VLA의 high-level 제안을 연결합니다.

세부 역할, interface 방향, 평가 기준, 첫 sprint는
[docs/project_scope.md](docs/project_scope.md)에 기록합니다.

## 현재 상태

현재는 최종 프로젝트의 시작 단계로, 역할 분담과 motion 중심 개발 범위,
첫 milestone, 공개 저장소 관리 기준을 정의한 상태입니다. 실제 runtime
코드는 baseline 경계와 실험 환경이 확정된 뒤 선별적으로 추가합니다.

## 데이터 및 안전 관리

다음 항목은 별도 승인 없이 공개 저장소에 commit하지 않습니다.

- 로봇/카메라 calibration 파일
- 배포 여부가 정해지지 않은 model weight
- raw camera log, rosbag, 실험 영상
- credential, token, 장비별 네트워크 설정

실제 로봇에서 motion 변경을 시험할 때는 속도를 제한하고, collision scene과
실험 조건을 기록한 뒤 pick & place 전체 동작을 실행합니다.

## 다음 세션에서 이어가는 방법

새 대화나 새 개발 환경에서는 이전 채팅 내용을 자동으로 기억하는 방식이
아닙니다. 대신 다음 순서로 저장소 기록을 읽으면 같은 맥락에서 작업을
이어갈 수 있습니다.

1. 이 `README.md`에서 목표와 현재 상태를 확인합니다.
2. `AGENTS.md`에서 담당 범위, 설계 원칙, 작업 순서를 확인합니다.
3. `docs/project_scope.md`에서 milestone과 interface/평가 계획을 확인합니다.
4. `git log`와 issue/commit 기록으로 마지막 실제 변경 내용을 확인합니다.
5. 구현을 옮길 때는 `strawberry_miniproject`를 baseline reference로 확인합니다.

즉, 중요한 결정과 진행 결과를 문서와 commit으로 계속 남기는 것이 이
프로젝트의 기억 장치입니다.
