"""Loading and validating workspace exploration configuration."""

from dataclasses import dataclass
from pathlib import Path

import yaml

from .quadtree_map import WorkspaceBounds


@dataclass(frozen=True)
class WorkspaceConfig:
    frame_id: str
    bounds: WorkspaceBounds
    root_split: tuple[float, float]
    max_depth: int
    initial_subdivision_depth: int
    preview_standoff_m: float | None


def load_workspace_config(path: Path) -> WorkspaceConfig:
    with path.open("r", encoding="utf-8") as config_file:
        raw = yaml.safe_load(config_file)

    workspace = raw["workspace"]
    bounds = workspace["bounds_m"]
    split = workspace["root_split_m"]
    config = WorkspaceConfig(
        frame_id=str(workspace["frame_id"]),
        bounds=WorkspaceBounds(
            float(bounds["min_x"]),
            float(bounds["max_x"]),
            float(bounds["min_y"]),
            float(bounds["max_y"]),
        ),
        root_split=(float(split["x"]), float(split["y"])),
        max_depth=int(workspace["max_depth"]),
        initial_subdivision_depth=int(raw["scan_policy"]["initial_subdivision_depth"]),
        preview_standoff_m=(
            float(raw["scan_pose"]["preview_standoff_m"])
            if raw.get("scan_pose", {}).get("preview_standoff_m") is not None
            else None
        ),
    )
    if config.initial_subdivision_depth > config.max_depth:
        raise ValueError("initial_subdivision_depth cannot exceed max_depth.")
    config.bounds.quadrants(config.root_split)
    if config.preview_standoff_m is not None and config.preview_standoff_m <= 0:
        raise ValueError("preview_standoff_m must be positive.")
    return config
