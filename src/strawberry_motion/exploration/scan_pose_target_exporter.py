"""Export camera/TCP target candidates for offline planning validation."""

import argparse
from pathlib import Path

import numpy as np
import yaml

from .quadtree_map import QuadtreeMap
from .scan_pose_generator import generate_observation_pose_targets
from .workspace_config import load_workspace_config


def parse_args(args=None):
    parser = argparse.ArgumentParser(
        description="Print visualization-only scan pose candidates as YAML; no robot motion."
    )
    parser.add_argument("--workspace-config", required=True)
    parser.add_argument("--panel-registration", required=True)
    parser.add_argument("--calibration-file", required=True)
    return parser.parse_args(args)


def export_targets(workspace_path: Path, registration_path: Path, calibration_path: Path):
    config = load_workspace_config(workspace_path)
    if config.preview_standoff_m is None:
        raise ValueError("workspace scan_pose.preview_standoff_m is required.")

    with registration_path.open("r", encoding="utf-8") as stream:
        registration = yaml.safe_load(stream)["panel_registration"]
    panel_transform_base = np.asarray(registration["transform"]["matrix"], dtype=float)
    calibration = np.load(calibration_path)
    tcp_to_camera = calibration["T_cam_to_gripper"]

    tree = QuadtreeMap(config.bounds, config.max_depth, root_split=config.root_split)
    cells = tree.subdivide("root")
    targets = generate_observation_pose_targets(
        cells, config.preview_standoff_m, panel_transform_base, tcp_to_camera
    )
    return {
        "scan_pose_candidates": {
            "status": "geometry_ready_pending_curobo_dry_run",
            "source_registration": str(registration_path),
            "standoff_m": config.preview_standoff_m,
            "use_for_automated_motion": False,
            "targets": [
                {
                    "cell_id": target.cell_id,
                    "camera_transform_base": target.camera_transform_base.round(9).tolist(),
                    "tcp_transform_base": target.tcp_transform_base.round(9).tolist(),
                }
                for target in targets
            ],
        }
    }


def main(args=None) -> None:
    options = parse_args(args)
    data = export_targets(
        Path(options.workspace_config),
        Path(options.panel_registration),
        Path(options.calibration_file).expanduser(),
    )
    print(yaml.safe_dump(data, sort_keys=False))
    print("# Read-only output: run IK/collision validation before any robot motion.")


if __name__ == "__main__":
    main()
