import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("uav_global_planner")
    config_file = os.path.join(pkg_share, "config", "planner.yaml")

    return LaunchDescription([
        Node(
            package="uav_global_planner",
            executable="global_planner",
            name="uav_global_planner",
            output="screen",
            parameters=[
                {
                    "config_file": config_file,
                }
            ],
        )
    ])
