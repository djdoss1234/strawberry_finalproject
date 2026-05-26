import unittest

from strawberry_motion.visualization.realsense_alignment_viewer import delta_for_key, parse_args


class RealsenseAlignmentViewerTest(unittest.TestCase):
    def test_uses_low_latency_color_stream_defaults(self):
        options = parse_args([])

        self.assertEqual(options.width, 640)
        self.assertEqual(options.height, 480)
        self.assertEqual(options.fps, 30)
        self.assertEqual(options.crosshair_length_px, 60)
        self.assertFalse(options.enable_robot_control)

    def test_accepts_camera_stream_override(self):
        options = parse_args(["--width", "848", "--height", "480", "--fps", "60"])

        self.assertEqual(options.width, 848)
        self.assertEqual(options.height, 480)
        self.assertEqual(options.fps, 60)

    def test_maps_camera_alignment_keys_to_small_base_frame_steps(self):
        self.assertEqual(delta_for_key(ord("a"), 2.0), ("LEFT  -X", [-2.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
        self.assertEqual(delta_for_key(ord("w"), 2.0), ("UP    +Z", [0.0, 0.0, 2.0, 0.0, 0.0, 0.0]))
        self.assertEqual(delta_for_key(ord("f"), 2.0), ("DEPTH -Y", [0.0, -2.0, 0.0, 0.0, 0.0, 0.0]))
        self.assertIsNone(delta_for_key(ord("q"), 2.0))


if __name__ == "__main__":
    unittest.main()
