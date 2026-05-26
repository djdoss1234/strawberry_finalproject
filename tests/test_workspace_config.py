import tempfile
import unittest
from pathlib import Path

from strawberry_motion.exploration.workspace_config import load_workspace_config


class WorkspaceConfigTest(unittest.TestCase):
    def test_loads_valid_workspace_config(self) -> None:
        config = load_workspace_config(Path("config/workspace.yaml"))

        self.assertEqual(config.frame_id, "cultivation_panel")
        self.assertEqual(config.initial_subdivision_depth, 1)
        self.assertEqual(config.bounds.center, (0.0, 0.0))

    def test_rejects_initial_depth_greater_than_max_depth(self) -> None:
        invalid_yaml = """
workspace:
  frame_id: panel
  bounds_m: {min_x: -1, max_x: 1, min_y: -1, max_y: 1}
  max_depth: 1
scan_policy:
  initial_subdivision_depth: 2
"""
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "workspace.yaml"
            path.write_text(invalid_yaml, encoding="utf-8")
            with self.assertRaises(ValueError):
                load_workspace_config(path)


if __name__ == "__main__":
    unittest.main()
