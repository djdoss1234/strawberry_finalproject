import unittest

import numpy as np

from strawberry_motion.visualization.alignment_overlay import (
    CrosshairStyle,
    draw_alignment_overlay,
)


class AlignmentOverlayTest(unittest.TestCase):
    def test_draws_crosshair_at_image_center_without_mutating_input(self):
        image = np.zeros((100, 200, 3), dtype=np.uint8)
        overlay = draw_alignment_overlay(image, CrosshairStyle(crosshair_length_px=10))

        self.assertTrue(np.array_equal(image, np.zeros_like(image)))
        self.assertGreater(int(overlay[50, 100].sum()), 0)
        self.assertGreater(int(overlay[50, 90].sum()), 0)
        self.assertGreater(int(overlay[40, 100].sum()), 0)

    def test_rejects_non_bgr_image(self):
        with self.assertRaises(ValueError):
            draw_alignment_overlay(np.zeros((20, 20), dtype=np.uint8))


if __name__ == "__main__":
    unittest.main()
