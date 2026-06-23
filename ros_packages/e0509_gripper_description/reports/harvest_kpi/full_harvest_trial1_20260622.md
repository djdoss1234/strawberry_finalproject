# Full Harvest Trial 1 - 2026-06-22

## Run Evidence

- Planner log: `logs/runtime/2026-06-22/curobo_planner_node_20260622T203920-7efa5ed6.jsonl`
- Fusion log: `logs/runtime/2026-06-22/strawberry_fusion_node_20260622T203927-76484715.jsonl`
- Git commit in runtime log: `3fb345f`
- 목적: 전체 셀 순회 중 실제 수확 가능/불가능 케이스와 KPI 수집 가능성 확인

## Manual Result Label

| 구분 | 결과 |
| --- | --- |
| 정상 딸기 수확 | 2개 성공 |
| 성공 위치 | SW 셀에서만 성공 |
| 나머지 정상 딸기 | 실패 |
| NW | 접근은 시도했으나 직선 진입 실패 |
| NE | 사용 가능한 pick target 인식 실패 또는 후보 rejection |
| SE | 빈 셀/유효 후보 없음으로 판단 |

> 이번 회차의 최종 성공 라벨은 사람 관찰 기준이다. gripper position 기반 자동 판정은 실제 파지와 불일치가 있어 보정 필요.

## Observed Problems

| 문제 | 관찰 내용 | 로그 근거 |
| --- | --- | --- |
| SW J6 branch 불안정 | 같은 SW에서도 J6가 불필요하게 회전하거나 접근이 부자연스러움 | pick/return spline에서 J6 equivalent branch가 계속 바뀌는 증상 관찰 |
| SW 파지 깊이 부족 | 딸기는 따지만 꽉 물고 당기는 느낌이 약함 | 여러 pick에서 `GRASP_EMPTY pos=700`으로 기록되어 자동 판정과 실제 관찰이 불일치 |
| NW 직선 진입 실패 | 높은/가려진 타겟에서 final approach가 0.15초 만에 실패 | `FINAL_APPROACH_STRAIGHT_BASE success=false elapsed=0.15s`, fallback IK_FAIL |
| NE 인식 실패 | 딸기가 있어도 usable pick target으로 안정화되지 않음 | `stem_keypoint_depth_invalid`, `stem_geometry_implausible` 다수 |
| 자동 grasp 판정 불일치 | 실제로 잡은 경우에도 gripper position이 699~700으로 나오는 케이스 발생 | `verify_grasp: GRASP_EMPTY present_position=700` 반복 |

## Perception Evidence

초기 SW 관측에서는 ripe 후보 자체는 충분히 나왔다.

- `seg_ripe_count=10`
- `scene_ripe_3d_count=8`
- `pose_detection_count=11`
- 이후 안정화된 pick 후보가 생기며 target publish 진행

NE/NW 쪽에서는 후보가 보여도 줄기 keypoint depth가 깨지거나 줄기 길이가 비정상적으로 계산되어 버려지는 케이스가 많았다.

- `stem_keypoint_depth_invalid`
- `stem_geometry_implausible`
- 일부 keypoint 3D가 벽 뒤쪽으로 크게 튀는 현상 확인

## Motion Evidence

SW 쪽 final approach는 여러 번 성공했다.

- `FINAL_APPROACH_STRAIGHT_BASE success=true`
- 약 `3.65s` 내 직선 진입 완료

NW 높은 타겟에서는 같은 final approach 단계가 실패했다.

- `FINAL_APPROACH_STRAIGHT_BASE success=false`
- elapsed 약 `0.15s`
- cuRobo fallback도 `IK_FAIL`

따라서 현재 실패는 단순히 "로봇이 전체적으로 못 감"이 아니라, 셀/타겟 높이/줄기 keypoint 품질/관절 branch에 따라 실패 양상이 갈린다.

## Immediate Conclusion

1. 발표용 전체 수확 KPI는 아직 공식 성공률로 쓰면 안 된다.
2. 이번 1트는 `SW normal case 2개 성공`, `NW/NE 복잡 케이스 실패`로 정직하게 기록한다.
3. 바로 다음 목표는 전체 셀 완성보다 `NW에서 1~2개라도 안정적으로 따는 프로파일`을 만드는 것이다.
4. SW는 성공했지만 J6 branch와 얕은 파지 문제가 남아 있으므로 baseline freeze 전 regression 확인이 필요하다.

## Next Debug Plan

| 우선순위 | 작업 |
| --- | --- |
| 1 | NW는 `flat_grasp_only` 기반 수평 접근 프로파일로 다시 제한 테스트 |
| 2 | 높은/가려진 후보를 바로 집지 말고, lower-z / valid stem target을 우선 선택 |
| 3 | final approach 실패 시 무리한 fallback 반복 대신 해당 target skip 후 다음 후보로 전환 |
| 4 | gripper position-only 판정값 699~700 불일치 원인 확인 |
| 5 | J6 branch 변화가 큰 경로를 pre-approach 단계에서 reject 또는 같은 branch seed로 고정 |

## Excluded Attempt Notes

- `2026-06-22T21:00` 전후 NW-flat 재시도는 공식 3트로 집계하지 않음.
- 관찰: 의도한 중앙/나머지 후보가 아니라 Y자 형태의 왼쪽 딸기 후보로 보이는 target을 먼저 선택했고, 직선 진입 단계에서 실패.
- 코드 조치: 실패한 target 좌표를 다음 스캔에서 제외하고 남은 후보를 이어서 시도하도록 scan executor에 attempted-target blacklist를 추가함.
- `2026-06-22T21:09` 전후 NW-flat 재시도도 공식 3트로 집계하지 않음.
- 관찰: 왼쪽/Y자 후보는 직선 진입 실패 후 다음 후보로 넘어갔고, 다음 후보는 직선 진입까지 성공했으나 gripper close `set_position(700)` timeout으로 파지 실패.
- 코드 조치: flat grasp에서 Y clamp가 있어도 180mm blind push를 쓰지 않고 target-plane + margin으로 final approach 거리를 제한하도록 수정. 또한 gripper close timeout 직후 바로 abort하지 않고 position-only 상태를 다시 읽어 파지/빈파지를 판정하도록 수정.
