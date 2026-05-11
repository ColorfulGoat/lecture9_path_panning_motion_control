from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="uav_global_planner",
            executable="mission_uploader",
            name="mavros_mission_uploader",
            output="screen",
            parameters=[
                {
                    "path_topic": "/planned_path",
                    "gps_topic": "/mavros/global_position/global",
                    "arm_and_start": False,
                    "acceptance_radius": 2.0,
                    "takeoff_altitude": 3.0,
                    "input_timeout_sec": 60.0,
                }
            ],
        )
    ])
