# 개발 운영 방식: 모듈화, ROS Graph 점검, Git 기록

## 1. 기본 원칙

이 프로젝트는 큰 기능을 한 번에 붙이는 방식보다, 작은 모듈을 만들고
ROS 연결과 실제 motion 영향을 확인하며 진행합니다.

작업 loop:

```text
작은 기능 정의
  -> 모듈 구현
  -> 단위/정적 검사
  -> ROS node/topic/TF 연결 확인
  -> RViz 또는 저속 실기 검증
  -> 결과 기록
  -> Git commit/push
```

## 2. 코드 모듈화 기준

첫 구현 대상인 quadtree 탐색 환경은 다음 책임으로 분리합니다.

```text
exploration/
  quadtree_map.py          cell 분할, depth, 상태 저장
  region_state.py          상태 enum 및 transition 규칙
  scan_pose_generator.py   cell 중심에서 camera observation pose 생성
  exploration_manager.py   다음 관찰 cell 선택과 결과 반영

visualization/
  workspace_marker_node.py RViz에 workspace/cell/상태/scan pose 표시
  alignment_overlay.py     camera 중앙 십자선 overlay 렌더링
  camera_alignment_node.py RGB image를 받아 정렬용 overlay image publish
  realsense_alignment_viewer.py RealSense 직접 표시용 저지연 정렬 도구

config/
  workspace.yaml           작업영역 frame, 크기, 최대 분할 depth
  scan_policy.yaml         탐색 순서, 재방문, 분할 규칙
```

모듈 분리 순서:

1. 로봇과 연결되지 않는 `quadtree_map`과 상태 모델을 먼저 만듭니다.
2. RViz marker visualization을 붙여 분할 결과를 눈으로 검증합니다.
3. `scan_pose_generator`를 붙여 실제 motion target을 만듭니다.
4. planner/executor와 연결하기 전에 pose/TF/작업영역 경계를 검증합니다.

## 3. `rqt_graph` 확인 시점

`rqt_graph`는 마지막 발표 자료용 캡처가 아니라, node 연결을 바꿀 때마다
확인하는 디버깅 도구로 사용합니다.

| 점검 시점 | 확인할 내용 | 저장할 증거 |
| --- | --- | --- |
| exploration node 최초 실행 | node가 올라오는지, 의도한 topic만 publish하는지 | graph 캡처 |
| RViz marker 연결 후 | marker topic 연결과 중복 publisher 여부 | graph + RViz 캡처 |
| camera overview 정렬 시 | camera 입력과 alignment overlay 출력 연결 | overlay 화면 + joint/TCP pose |
| scan pose publish 후 | exploration에서 motion 입력까지 흐름 | graph + topic echo 요약 |
| planner/executor 연결 후 | command/result feedback loop | graph + 실행 log |
| perception/VLA 연결 후 | target proposal과 motion result 경계 | 최종 graph 캡처 |

현재 최초 exploration node interface:

```text
/strawberry/exploration/set_cell_state
  -> /workspace_marker_node
  -> /strawberry/exploration/workspace_cells
  -> /strawberry/exploration/next_cell

/camera/camera/color/image_raw
  -> /camera_alignment_node
  -> /strawberry/alignment/overlay_image
```

`camera_alignment_node`는 현재 환경에서 확인된 `cv_bridge`와 `NumPy 2.x`
binary compatibility 문제를 피하기 위해 `sensor_msgs/Image` buffer를
직접 읽어 overlay를 publish합니다. 입력 topic은 launch argument로
변경할 수 있습니다.

실제 robot jog 중에는 ROS image relay/viewer 지연을 피하기 위해
`realsense_alignment_viewer`를 우선 사용합니다. ROS overlay node는
topic interface와 graph/evidence 확인에 사용합니다.

함께 확인할 것:

- `ros2 node list`
- `ros2 topic list`
- `ros2 topic info <topic>`
- `ros2 run tf2_tools view_frames` 또는 TF tree 확인
- RViz에서 workspace marker와 target pose 확인

캡처 파일은 raw artifact 저장 위치에 보관하고, Git에는 파일 자체보다
어느 run에서 무엇을 확인했는지 기록합니다. 포트폴리오에 사용할 대표
이미지만 크기와 공개 여부를 확인한 뒤 별도로 추가합니다.

구체적인 폴더 구조, 파일명, 문서 내 `VISUAL TODO` 주석 형식은
`docs/visual_asset_guide.md`를 따릅니다.

## 4. Git에 저장하는 방식

진행상황은 Git에 자주 남기는 편이 좋습니다. 다만 **파일을 한 줄 수정할
때마다 commit하는 것이 아니라, 되돌아갈 수 있는 의미 있는 단위가 끝날
때마다 commit**합니다.

권장 commit 단위:

- 문서로 설계 결정이 확정됨
- 하나의 모듈 skeleton 또는 순수 로직이 동작함
- ROS node/topic 연결이 확인됨
- RViz 시각화 또는 실기 실험 한 단계가 검증됨
- bug 원인과 수정이 재현 가능한 형태로 해결됨

권장 commit message 예시:

```text
docs: define quadtree exploration states and scan policy
feat: add workspace quadtree map and RViz cell markers
feat: publish scan poses for unobserved workspace cells
test: record quadtree scan motion validation on scene S0
fix: prevent revisiting harvested cells during exploration
```

Git에 넣지 않을 것:

- calibration 원본
- weight 파일
- raw camera images, 전체 영상, rosbag
- token, IP 등 장비별 민감 설정
- 동작하지 않는 임시 scratch 파일

## 5. 진행 기록 규칙

매 작업일마다 `docs/worklogs/YYYY-MM-DD.md`를 작성하거나 갱신합니다.

기록 항목:

- 오늘 목표
- 변경한 module/topic/config
- 확인한 `rqt_graph`/TF/RViz 상태
- 실행한 실험 scene과 결과
- 막힌 문제와 다음 행동
- 관련 commit hash

이 기록과 Git commit이 다음 세션에서 프로젝트를 이어가는 기준이 됩니다.

단계별 실행, 문제 해결, 포트폴리오 근거를 함께 관리하는 자세한 규칙은
`docs/project_recording_system.md`를 따릅니다.

기본 연결 구조:

```text
worklog
  -> run record
       -> issue record
       -> commit / evidence asset
  -> portfolio_evidence
```
