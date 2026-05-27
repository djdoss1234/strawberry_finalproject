"""Checks for the offline registered-whiteboard collision world."""

from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parent.parent


def test_scan_world_remains_non_executable():
    with (ROOT / "config" / "scan_collision_world.yaml").open() as stream:
        cfg = yaml.safe_load(stream)["scan_collision_world"]
    assert cfg["robot_collision_spheres_enabled"] is True
    assert cfg["self_collision_check_enabled"] is False
    assert cfg["use_for_automated_motion"] is False


def test_registered_whiteboard_is_only_enabled_scan_obstacle():
    with (ROOT / "config" / "scan_collision_world.yaml").open() as stream:
        cfg = yaml.safe_load(stream)["scan_collision_world"]
    active = [item for item in cfg["objects"] if item["enabled"]]
    assert [item["name"] for item in active] == ["registered_whiteboard"]
    assert active[0]["dims_m"] == [1.5, 0.9, 0.02]
