"""Generate visualization-only cell observation pose previews."""

from dataclasses import dataclass
from typing import Iterable, Tuple

from .quadtree_map import QuadtreeCell


@dataclass(frozen=True)
class ObservationPosePreview:
    cell_id: str
    x: float
    y: float
    z: float


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
