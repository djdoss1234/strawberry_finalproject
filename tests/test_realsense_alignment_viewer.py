import unittest

from strawberry_motion.visualization.realsense_alignment_viewer import (
    parse_args,
    reject_unsafe_motion_option,
)


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

    def test_rejects_withdrawn_robot_control_option(self):
        options = parse_args(["--enable-robot-control"])

        with self.assertRaises(RuntimeError):
            reject_unsafe_motion_option(options)


if __name__ == "__main__":
    unittest.main()
