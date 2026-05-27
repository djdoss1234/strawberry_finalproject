import unittest

import numpy as np

from strawberry_motion.exploration import QuadtreeMap, WorkspaceBounds
from strawberry_motion.exploration.scan_pose_generator import (
    generate_observation_pose_previews,
    generate_observation_pose_targets,
)


class ScanPoseGeneratorTest(unittest.TestCase):
    def test_generates_cell_center_previews_at_requested_standoff(self) -> None:
        tree = QuadtreeMap(
            WorkspaceBounds(-0.545, 0.555, -0.405, 0.395),
            root_split=(0.0, 0.0),
        )
        cells = tree.subdivide("root")

        previews = generate_observation_pose_previews(cells, 0.9)

        self.assertEqual(previews[0].cell_id, "root/nw")
        self.assertEqual((previews[0].x, previews[0].y), (-0.2725, 0.1975))
        self.assertEqual(previews[0].z, 0.9)

    def test_rejects_nonpositive_standoff(self) -> None:
        with self.assertRaises(ValueError):
            generate_observation_pose_previews([], 0.0)

    def test_generates_camera_and_tcp_targets_facing_panel(self) -> None:
        tree = QuadtreeMap(WorkspaceBounds(-1.0, 1.0, -1.0, 1.0))
        cell = tree.subdivide("root")[0]

        targets = generate_observation_pose_targets(
            [cell], 0.5, np.eye(4), np.eye(4)
        )

        np.testing.assert_allclose(targets[0].camera_transform_base[:3, 3], [-0.5, 0.5, 0.5])
        np.testing.assert_allclose(targets[0].camera_transform_base[:3, :3], np.diag([1.0, -1.0, -1.0]))
        np.testing.assert_allclose(targets[0].tcp_transform_base, targets[0].camera_transform_base)


if __name__ == "__main__":
    unittest.main()
