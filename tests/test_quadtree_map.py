import unittest

from strawberry_motion.exploration import QuadtreeMap, RegionState, WorkspaceBounds


class WorkspaceBoundsTest(unittest.TestCase):
    def test_quadrants_preserve_deterministic_centers(self) -> None:
        bounds = WorkspaceBounds(-0.4, 0.4, -0.2, 0.2)
        centers = [quadrant.center for quadrant in bounds.quadrants()]

        self.assertEqual(
            centers,
            [(-0.2, 0.1), (0.2, 0.1), (-0.2, -0.1), (0.2, -0.1)],
        )


class QuadtreeMapTest(unittest.TestCase):
    def setUp(self) -> None:
        self.tree = QuadtreeMap(WorkspaceBounds(-0.4, 0.4, -0.2, 0.2), max_depth=2)

    def test_subdivide_creates_four_unscanned_leaf_cells(self) -> None:
        children = self.tree.subdivide("root")

        self.assertEqual(len(children), 4)
        self.assertTrue(all(child.is_leaf for child in children))
        self.assertTrue(all(child.state is RegionState.UNSCANNED for child in children))

    def test_next_scan_cell_prefers_unscanned_before_revisit(self) -> None:
        self.tree.subdivide("root")
        self.tree.update_state("root/ne", RegionState.REVISIT)

        self.assertEqual(self.tree.next_scan_cell().cell_id, "root/nw")

    def test_next_scan_cell_returns_none_when_no_observation_is_pending(self) -> None:
        self.tree.subdivide("root")
        for cell_id in ("root/nw", "root/ne", "root/sw", "root/se"):
            self.tree.update_state(cell_id, RegionState.SCANNED_EMPTY)

        self.assertIsNone(self.tree.next_scan_cell())

    def test_cannot_update_non_leaf_cell(self) -> None:
        self.tree.subdivide("root")

        with self.assertRaises(ValueError):
            self.tree.update_state("root", RegionState.HARVESTED)


if __name__ == "__main__":
    unittest.main()
