import unittest

import numpy as np

from strawberry_motion.registration.panel_frame_capture import estimate_panel_transform


class PanelFrameCaptureTest(unittest.TestCase):
    def test_estimates_origin_and_image_aligned_axes(self):
        origin = np.array([0.1, -0.2, 0.3])
        result = estimate_panel_transform(
            origin,
            origin + np.array([0.4, 0.0, 0.0]),
            origin + np.array([0.0, 0.5, 0.0]),
        )

        np.testing.assert_allclose(result[:3, 3], origin)
        np.testing.assert_allclose(result[:3, :3], np.eye(3), atol=1e-9)


if __name__ == "__main__":
    unittest.main()
