# SW Demo KPI - 2026-06-22

Source log: `logs/runtime/2026-06-22/curobo_planner_node_20260622T123552-b867b0c6.jsonl`

- Baseline SW single-pick time: **36.4s**
- Current approach-to-grasp average: **15.1s** (n=3)
- Reduction: **58.6%**
- Pick-to-place average: **49.3s**
- Full sequence average: **61.7s** (includes place + return to scan pose)
- Sequence completion: **3/3**
- Place completion: **3/3** (slots 0, 1, 3)
- Automatic gripper-confirmed grasp: **0/3** (`GRASP_EMPTY`, `GRASP_EMPTY`, `GRASP_UNVERIFIED`)
- Note: use video/manual label for real harvest success; gripper sensor threshold is not yet reliable for thin stem.
