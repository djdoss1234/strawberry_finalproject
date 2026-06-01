# RUN-20260527-003 — Scan Pose TCP Preview (Direction Arrow Corrected)

날짜: 2026-05-27
관련 노드: `scan_pose_tcp_preview_node`

## 내용

`scan_pose_tcp_preview_node`에서 발행하는 RViz MarkerArray를 통해
각 셀 스캔 포즈의 TCP 위치와 카메라 방향 화살표를 시각적으로 확인.

- 화살표 방향: TCP Z축 기준 (카메라 광축 방향)
- 좌표 기준: `base_link`
- 마커 색상: 셀별 구분 (NW/NE/SE/SW)

## 결과

TCP 방향 화살표가 각 셀 중심을 향해 올바르게 표시됨을 RViz에서 확인.
이전 버전에서 panel_normal vs cam_dir 혼용으로 방향이 틀렸던 문제 수정 후 재확인.

## 사진

`docs/runs/photos/RUN-20260527-003/scan_pose_preview_corrected.png`
