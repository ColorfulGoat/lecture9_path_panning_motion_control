from setuptools import setup
from glob import glob
import os

package_name = "uav_global_planner"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="student",
    maintainer_email="student@example.com",
    description="ROS 2 3D global planner for PX4 SITL using A* on a voxel grid.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "global_planner = uav_global_planner.global_planner_node:main",
            "mission_uploader = uav_global_planner.mavros_mission_node:main",
        ],
    },
)
