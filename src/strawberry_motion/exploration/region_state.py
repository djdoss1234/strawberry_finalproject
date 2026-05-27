"""State labels for a workspace cell during harvesting exploration."""

from enum import Enum


class RegionState(str, Enum):
    """Observable state of a quadtree cell."""

    UNSCANNED = "UNSCANNED"
    SCANNING = "SCANNING"
    SCANNED_EMPTY = "SCANNED_EMPTY"
    TARGET_FOUND = "TARGET_FOUND"
    PICKABLE = "PICKABLE"
    OCCLUDED = "OCCLUDED"
    PLANNING_FAIL = "PLANNING_FAIL"
    HARVESTED = "HARVESTED"
    REVISIT = "REVISIT"
