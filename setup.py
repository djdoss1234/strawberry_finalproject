from glob import glob
from setuptools import find_packages, setup


package_name = "strawberry_motion"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="djdoss1234",
    maintainer_email="djdoss1234@users.noreply.github.com",
    description="Motion and workspace exploration modules for strawberry harvesting.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "workspace_marker_node = strawberry_motion.visualization.workspace_marker_node:main",
            "camera_alignment_node = strawberry_motion.visualization.camera_alignment_node:main",
            "realsense_alignment_viewer = strawberry_motion.visualization.realsense_alignment_viewer:main",
            "panel_frame_capture = strawberry_motion.registration.panel_frame_capture:main",
            "scan_pose_target_exporter = strawberry_motion.exploration.scan_pose_target_exporter:main",
            "scan_pose_tcp_preview_node = strawberry_motion.visualization.scan_pose_tcp_preview_node:main",
            "scan_executor_node = strawberry_motion.execution.scan_executor_node:main",
        ],
    },
)
