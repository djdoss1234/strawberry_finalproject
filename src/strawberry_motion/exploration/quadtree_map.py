"""Quadtree workspace model for eye-in-hand harvesting exploration."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Iterable, Optional, Tuple

from .region_state import RegionState


@dataclass(frozen=True)
class WorkspaceBounds:
    """Rectangular workspace bounds expressed in the workspace frame."""

    min_x: float
    max_x: float
    min_y: float
    max_y: float

    def __post_init__(self) -> None:
        if self.min_x >= self.max_x or self.min_y >= self.max_y:
            raise ValueError("Workspace bounds must have positive width and height.")

    @property
    def center(self) -> Tuple[float, float]:
        return ((self.min_x + self.max_x) / 2.0, (self.min_y + self.max_y) / 2.0)

    def quadrants(self) -> Tuple["WorkspaceBounds", ...]:
        """Return NW, NE, SW, SE quadrants in deterministic scan order."""
        center_x, center_y = self.center
        return (
            WorkspaceBounds(self.min_x, center_x, center_y, self.max_y),
            WorkspaceBounds(center_x, self.max_x, center_y, self.max_y),
            WorkspaceBounds(self.min_x, center_x, self.min_y, center_y),
            WorkspaceBounds(center_x, self.max_x, self.min_y, center_y),
        )


@dataclass
class QuadtreeCell:
    """A rectangular observation cell and its current task state."""

    cell_id: str
    bounds: WorkspaceBounds
    depth: int
    state: RegionState = RegionState.UNSCANNED
    children: Tuple[str, ...] = field(default_factory=tuple)

    @property
    def is_leaf(self) -> bool:
        return not self.children

    @property
    def center(self) -> Tuple[float, float]:
        return self.bounds.center


class QuadtreeMap:
    """Stateful quadtree used to schedule observation cells.

    This class deliberately contains no ROS dependency. A visualization node
    and motion node can consume the same deterministic cell/state model.
    """

    def __init__(self, bounds: WorkspaceBounds, max_depth: int = 3) -> None:
        if max_depth < 0:
            raise ValueError("max_depth must be non-negative.")
        self.max_depth = max_depth
        self.cells: Dict[str, QuadtreeCell] = {
            "root": QuadtreeCell(cell_id="root", bounds=bounds, depth=0)
        }

    def get(self, cell_id: str) -> QuadtreeCell:
        try:
            return self.cells[cell_id]
        except KeyError as exc:
            raise KeyError(f"Unknown cell_id: {cell_id}") from exc

    def leaf_cells(self) -> Iterable[QuadtreeCell]:
        return (cell for cell in self.cells.values() if cell.is_leaf)

    def subdivide(self, cell_id: str) -> Tuple[QuadtreeCell, ...]:
        """Split a leaf cell and return its four children."""
        cell = self.get(cell_id)
        if not cell.is_leaf:
            return tuple(self.get(child_id) for child_id in cell.children)
        if cell.depth >= self.max_depth:
            raise ValueError(f"Cell {cell_id} already reached max_depth.")

        suffixes = ("nw", "ne", "sw", "se")
        child_ids = tuple(f"{cell.cell_id}/{suffix}" for suffix in suffixes)
        children = tuple(
            QuadtreeCell(child_id, bounds, cell.depth + 1)
            for child_id, bounds in zip(child_ids, cell.bounds.quadrants())
        )
        cell.children = child_ids
        self.cells.update({child.cell_id: child for child in children})
        return children

    def update_state(self, cell_id: str, state: RegionState) -> QuadtreeCell:
        cell = self.get(cell_id)
        if not cell.is_leaf:
            raise ValueError("Only leaf cell state can be updated.")
        cell.state = state
        return cell

    def next_scan_cell(self) -> Optional[QuadtreeCell]:
        """Select the next leaf cell to observe.

        New cells are observed before explicit revisit requests. Within one
        state, shallower cells and then deterministic IDs are preferred.
        """
        priority = {
            RegionState.UNSCANNED: 0,
            RegionState.REVISIT: 1,
            RegionState.OCCLUDED: 2,
        }
        candidates = [
            cell for cell in self.leaf_cells() if cell.state in priority
        ]
        if not candidates:
            return None
        return min(candidates, key=lambda cell: (priority[cell.state], cell.depth, cell.cell_id))
