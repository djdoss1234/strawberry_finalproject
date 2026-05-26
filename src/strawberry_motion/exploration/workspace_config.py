"""Loading and validating workspace exploration configuration."""

from dataclasses import dataclass
from pathlib import Path

import yaml

from .quadtree_map import WorkspaceBounds


@dataclass(frozen=True)
class WorkspaceConfig:
    frame_id: str
    bounds: WorkspaceBounds
    max_depth: int
    initial_subdivision_depth: int


def load_workspace_config(path: Path) -> WorkspaceConfig:
    with path.open("r", encoding="utf-8") as config_file:
        raw = yaml.safe_load(config_file)

    workspace = raw["workspace"]
    bounds = workspace["bounds_m"]
    config = WorkspaceConfig(
        frame_id=str(workspace["frame_id"]),
        bounds=WorkspaceBounds(
            float(bounds["min_x"]),
            float(bounds["max_x"]),
            float(bounds["min_y"]),
            float(bounds["max_y"]),
        ),
        max_depth=int(workspace["max_depth"]),
        initial_subdivision_depth=int(raw["scan_policy"]["initial_subdivision_depth"]),
    )
    if config.initial_subdivision_depth > config.max_depth:
        raise ValueError("initial_subdivision_depth cannot exceed max_depth.")
    return config
