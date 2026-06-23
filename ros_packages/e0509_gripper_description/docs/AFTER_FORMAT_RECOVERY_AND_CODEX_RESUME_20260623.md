# 포맷 후 복구 및 Codex 재개 인계서 (2026-06-23)

이 문서는 현재 컴퓨터를 포맷한 뒤, 딸기 수확 로봇 프로젝트를 다시 복구하고 Codex/Claude Code와
이어서 작업하기 위한 최소 인계서다.

## 1. 공식 GitHub 저장소

공식 저장소:

```text
https://github.com/djdoss1234/strawberry_finalproject.git
```

현재 작업 브랜치:

```text
debug/nw-return-to-depth-good
```

2026-06-23 기준 마지막 로컬 스냅샷 커밋:

```text
a899587 chore: snapshot motion demo and presentation artifacts
```

주의:

- 앞으로 커밋은 `strawberry_finalproject` 저장소에만 한다.
- `~/doosan_ws/src/e0509_gripper_description`은 보통
  `~/doosan_ws/src/strawberry_finalproject/ros_packages/e0509_gripper_description`로 연결된 작업 경로다.
- `scripts/측정.py`는 절대 수정하지 않는다.

## 2. 포맷 전 로컬 압축 백업 위치

포맷 전에 만들어 둔 백업은 아래 폴더에 있었다.

```text
/home/user/project_backups
```

압축 파일 3개:

```text
doosan_ws_strawberry_full_with_git_20260623_1406.tar.gz
strawberry_portfolio_artifacts_20260623_1409.tar.gz
presentation_assets_20260623.tar.gz
```

각 역할:

| 파일 | 역할 |
| --- | --- |
| `doosan_ws_strawberry_full_with_git_20260623_1406.tar.gz` | `doosan_ws` 전체 복구용. git 이력 포함 |
| `strawberry_portfolio_artifacts_20260623_1409.tar.gz` | 코드/문서/reports 중심 포트폴리오 자료 |
| `presentation_assets_20260623.tar.gz` | PPT, 영상, GIF, PNG, WEBM 발표자료 |

포맷 전에 외장 SSD/USB/클라우드로 위 3개를 복사해야 한다.

## 3. 새 컴퓨터/포맷 후 Git 기준 복구

ROS workspace를 다시 만들고 저장소를 clone한다.

```bash
mkdir -p ~/doosan_ws/src
cd ~/doosan_ws/src
git clone https://github.com/djdoss1234/strawberry_finalproject.git
cd strawberry_finalproject
git checkout debug/nw-return-to-depth-good
```

패키지 작업 경로가 필요하면 symlink 또는 직접 경로를 사용한다.

```bash
cd ~/doosan_ws/src/strawberry_finalproject/ros_packages/e0509_gripper_description
```

또는 기존처럼 쓰려면:

```bash
ln -s ~/doosan_ws/src/strawberry_finalproject/ros_packages/e0509_gripper_description \
      ~/doosan_ws/src/e0509_gripper_description
```

## 4. 압축 백업에서 자료 확인

발표자료만 풀고 싶을 때:

```bash
mkdir -p ~/restored_project_assets
tar -xzf /path/to/presentation_assets_20260623.tar.gz -C ~/restored_project_assets
```

포트폴리오 문서/reports만 풀고 싶을 때:

```bash
mkdir -p ~/restored_project_assets
tar -xzf /path/to/strawberry_portfolio_artifacts_20260623_1409.tar.gz -C ~/restored_project_assets
```

`doosan_ws` 전체를 복구할 때는 기존 workspace와 충돌하지 않게 별도 폴더에서 먼저 푸는 것을 권장한다.

```bash
mkdir -p ~/restore_test
tar -xzf /path/to/doosan_ws_strawberry_full_with_git_20260623_1406.tar.gz -C ~/restore_test
```

## 5. 다음에 Codex에게 처음 보낼 프롬프트

새 세션에서 아래 내용을 그대로 보내면 된다.

```text
딸기 수확 로봇 프로젝트 이어서 작업하자.

공식 repo:
~/doosan_ws/src/strawberry_finalproject

작업 브랜치:
debug/nw-return-to-depth-good

패키지 경로:
~/doosan_ws/src/strawberry_finalproject/ros_packages/e0509_gripper_description

먼저 읽어야 할 파일:
docs/AFTER_FORMAT_RECOVERY_AND_CODEX_RESUME_20260623.md
ros_packages/e0509_gripper_description/reports/presentation/motion_part_presentation_prep_kimminseok_actual_project.md
ros_packages/e0509_gripper_description/reports/presentation/motion_part_presentation_prep_kimminseok_detailed_backup_20260623.md
ros_packages/e0509_gripper_description/docs/RUNTIME_MODULE_INTERFACE_SPEC_20260620.md
ros_packages/e0509_gripper_description/docs/NW_TROUBLESHOOTING_CASE_LOG_20260621.md

절대 건드리지 말 것:
scripts/측정.py

현재 상태:
- 정상 노출 딸기 조건에서 10회 반복, 총 30개 중 27개 성공(90.0%)을 발표용 KPI로 정리함.
- 발표자료/PPT/영상/그래프는 reports와 project_backups 압축본에 백업함.
- 모션 파트 발표 준비 문서와 상세 백업본을 작성함.
- 아직 가림/군집/높은 위치 target은 실패 분석 단계이며, 전체 성공률로 과장하면 안 됨.

먼저 git status와 최신 커밋 상태를 확인하고, 이어서 내가 요청하는 작업을 진행해줘.
```

## 6. 프로젝트 현재 요약

성공적으로 정리한 내용:

- 정상 노출 딸기 반복 검증: 10회 x 3개 = 30개 중 27개 성공
- 성공률: 90.0%
- 평균 수확: 2.7개/회
- pick sequence 평균: 약 34.2초/개
- 3개 run 평균: 약 118초/run
- 접근-파지 구간 개선: 36.4초 -> 15.1초
- 발표용 그래프 생성:
  - `reports/harvest_kpi/presentation_10trial_success_rate_20260623.svg`
- 발표 준비 문서:
  - `reports/presentation/motion_part_presentation_prep_kimminseok_actual_project.md`
  - `reports/presentation/motion_part_presentation_prep_kimminseok_detailed_backup_20260623.md`
  - `reports/presentation/motion_slides_23_24_update_20260623.md`

미해결/향후 개선:

- fusion node가 계산한 실제 stem direction을 planner grasp orientation에 더 직접 반영해야 함.
- 주변 줄기/딸기를 cuRobo obstacle proxy로 더 잘 넣어야 함.
- SafeGrasp/current/position 기반 자동 파지 성공 판정은 아직 완전 신뢰 단계가 아님.
- NW/가림/군집/높은 위치 target은 정량 반복 성공률로 주장하지 말고 실패 분석으로 다룰 것.
- 시뮬레이션은 코드만 넘기면 되는 문제가 아니라 ROS topic/service/action, camera frame,
  gripper service, collision world bridge가 함께 필요함.

## 7. 발표용 핵심 문장

```text
정상 노출 조건에서는 현재 rule-based motion으로 실제 pick-place까지 연결할 수 있었지만,
가림/군집/높은 위치 조건에서는 줄기 방향, 접근 깊이, 주변 간섭, 관절 branch 문제가 함께 발생했다.
따라서 다음 단계는 단순 offset 추가가 아니라 stem direction 기반 grasp orientation,
multi-view 재관찰, obstacle-aware planning, VLA supervisor로 확장하는 것이다.
```

