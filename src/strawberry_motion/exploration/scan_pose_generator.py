"""Generate visualization-only cell observation pose previews."""

from dataclasses import dataclass
from typing import Iterable, Tuple

import numpy as np

from .quadtree_map import QuadtreeCell


@dataclass(frozen=True)
class ObservationPosePreview:
    cell_id: str
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class ObservationPoseTarget:
    cell_id: str
    camera_transform_base: np.ndarray
    tcp_transform_base: np.ndarray


def generate_observation_pose_previews(
    cells: Iterable[QuadtreeCell], standoff_m: float
) -> Tuple[ObservationPosePreview, ...]:
    """Place preview camera origins on the panel normal above each cell center."""
    if standoff_m <= 0:
        raise ValueError("standoff_m must be positive.")
    return tuple(
        ObservationPosePreview(cell.cell_id, cell.center[0], cell.center[1], standoff_m)
        for cell in cells
    )


def generate_observation_pose_targets(
    cells: Iterable[QuadtreeCell],
    standoff_m: float,
    panel_transform_base: np.ndarray,
    tcp_to_camera_transform: np.ndarray,
) -> Tuple[ObservationPoseTarget, ...]:
    """Create camera-facing-board targets and corresponding TCP transforms."""
    if standoff_m <= 0:
        raise ValueError("standoff_m must be positive.")
    if panel_transform_base.shape != (4, 4) or tcp_to_camera_transform.shape != (4, 4):
        raise ValueError("Transforms must be 4x4 matrices.")

    # Camera optical +Z looks into the panel; image +X follows panel +X.
    camera_in_panel_rotation = np.diag([1.0, -1.0, -1.0])
    camera_to_tcp = np.linalg.inv(tcp_to_camera_transform)
    targets = []
    for cell in cells:
        camera_in_panel = np.eye(4)
        camera_in_panel[:3, :3] = camera_in_panel_rotation
        camera_in_panel[:3, 3] = [cell.center[0], cell.center[1], standoff_m]
        camera_transform_base = panel_transform_base @ camera_in_panel
        targets.append(
            ObservationPoseTarget(
                cell_id=cell.cell_id,
                camera_transform_base=camera_transform_base,
                tcp_transform_base=camera_transform_base @ camera_to_tcp,
            )
        )
    return tuple(targets)
