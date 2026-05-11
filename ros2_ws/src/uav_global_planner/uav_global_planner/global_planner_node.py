import os
import yaml

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray

from ament_index_python.packages import get_package_share_directory

from uav_global_planner.voxel_grid import VoxelGrid
from uav_global_planner.astar_3d import AStar3D


class GlobalPlannerNode(Node):
    def __init__(self):
        super().__init__("uav_global_planner")

        self.declare_parameter("config_file", "")

        self.path_pub = self.create_publisher(Path, "/planned_path", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/planning_markers", 10)

        self.config = self.load_config()
        self.frame_id = self.config.get("frame_id", "map")

        self.path = []
        self.obstacles = self.config.get("obstacles", [])
        self.no_fly_zones = self.config.get("no_fly_zones", [])

        self.run_planner()

        self.timer = self.create_timer(1.0, self.publish_outputs)

    def load_config(self):
        config_file = self.get_parameter("config_file").get_parameter_value().string_value

        if not config_file:
            pkg_share = get_package_share_directory("uav_global_planner")
            config_file = os.path.join(pkg_share, "config", "planner.yaml")

        self.get_logger().info(f"Loading planner config from: {config_file}")

        with open(config_file, "r") as f:
            data = yaml.safe_load(f)

        if "uav_global_planner" in data:
            maybe_params = data["uav_global_planner"].get("ros__parameters", None)
            if maybe_params is not None:
                data = maybe_params

        return data

    def run_planner(self):
        bounds = self.config["bounds"]
        resolution = self.config["resolution"]
        safety_radius = self.config.get("safety_radius", 0.0)
        start = self.config["start"]
        goal = self.config["goal"]

        self.get_logger().info("Creating voxel grid...")

        grid = VoxelGrid(
            bounds=bounds,
            resolution=resolution,
            obstacles=self.obstacles,
            no_fly_zones=self.no_fly_zones,
            safety_radius=safety_radius,
        )

        grid_info = grid.describe()

        self.get_logger().info(
            f"Grid size: {grid_info['size_x']} x "
            f"{grid_info['size_y']} x "
            f"{grid_info['size_z']}"
        )
        self.get_logger().info(f"Resolution: {grid_info['resolution']} m")
        self.get_logger().info(f"Occupied cells: {grid_info['occupied_cells']}")
        self.get_logger().info(f"Start: {start}")
        self.get_logger().info(f"Goal: {goal}")
        self.get_logger().info(f"Obstacles: {len(self.obstacles)}")
        self.get_logger().info(f"No-fly zones: {len(self.no_fly_zones)}")

        self.get_logger().info("Running 3D A* planner...")

        planner = AStar3D(grid)
        result = planner.plan(start, goal)

        self.path = result["path"]

        self.get_logger().info("A* planning successful.")
        self.get_logger().info(f"Expanded nodes: {result['expanded_nodes']}")
        self.get_logger().info(f"Path length: {result['path_length']:.2f} m")
        self.get_logger().info(f"Number of waypoints: {len(self.path)}")

    def publish_outputs(self):
        if not self.path:
            return

        stamp = self.get_clock().now().to_msg()

        self.path_pub.publish(self.create_path_msg(stamp))
        self.marker_pub.publish(self.create_marker_array(stamp))

    def create_path_msg(self, stamp):
        path_msg = Path()
        path_msg.header.stamp = stamp
        path_msg.header.frame_id = self.frame_id

        for waypoint in self.path:
            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = self.frame_id
            pose.pose.position.x = float(waypoint[0])
            pose.pose.position.y = float(waypoint[1])
            pose.pose.position.z = float(waypoint[2])
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        return path_msg

    def create_marker_array(self, stamp):
        markers = MarkerArray()
        marker_id = 0

        clear_marker = Marker()
        clear_marker.header.stamp = stamp
        clear_marker.header.frame_id = self.frame_id
        clear_marker.action = Marker.DELETEALL
        markers.markers.append(clear_marker)

        for obstacle in self.obstacles:
            markers.markers.append(
                self.create_box_marker(
                    stamp,
                    marker_id,
                    obstacle,
                    "obstacles",
                    1.0,
                    0.1,
                    0.1,
                    0.55,
                )
            )
            marker_id += 1

        for zone in self.no_fly_zones:
            markers.markers.append(
                self.create_box_marker(
                    stamp,
                    marker_id,
                    zone,
                    "no_fly_zones",
                    0.6,
                    0.0,
                    1.0,
                    0.35,
                )
            )
            marker_id += 1

        path_marker = Marker()
        path_marker.header.stamp = stamp
        path_marker.header.frame_id = self.frame_id
        path_marker.ns = "planned_path"
        path_marker.id = marker_id
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = 0.25
        path_marker.color.r = 0.0
        path_marker.color.g = 1.0
        path_marker.color.b = 0.0
        path_marker.color.a = 1.0

        for waypoint in self.path:
            p = Point()
            p.x = float(waypoint[0])
            p.y = float(waypoint[1])
            p.z = float(waypoint[2])
            path_marker.points.append(p)

        markers.markers.append(path_marker)
        marker_id += 1

        markers.markers.append(
            self.create_sphere_marker(
                stamp,
                marker_id,
                self.path[0],
                "start",
                0.0,
                0.3,
                1.0,
                1.0,
                0.9,
            )
        )
        marker_id += 1

        markers.markers.append(
            self.create_sphere_marker(
                stamp,
                marker_id,
                self.path[-1],
                "goal",
                1.0,
                0.9,
                0.0,
                1.0,
                0.9,
            )
        )

        return markers

    def create_box_marker(self, stamp, marker_id, box, namespace, red, green, blue, alpha):
        min_corner = box["min"]
        max_corner = box["max"]

        center_x = (min_corner[0] + max_corner[0]) / 2.0
        center_y = (min_corner[1] + max_corner[1]) / 2.0
        center_z = (min_corner[2] + max_corner[2]) / 2.0

        size_x = max_corner[0] - min_corner[0]
        size_y = max_corner[1] - min_corner[1]
        size_z = max_corner[2] - min_corner[2]

        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.frame_id
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = float(center_x)
        marker.pose.position.y = float(center_y)
        marker.pose.position.z = float(center_z)
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(size_x)
        marker.scale.y = float(size_y)
        marker.scale.z = float(size_z)
        marker.color.r = float(red)
        marker.color.g = float(green)
        marker.color.b = float(blue)
        marker.color.a = float(alpha)

        return marker

    def create_sphere_marker(self, stamp, marker_id, point, namespace, red, green, blue, alpha, scale):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.frame_id
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(point[0])
        marker.pose.position.y = float(point[1])
        marker.pose.position.z = float(point[2])
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(scale)
        marker.scale.y = float(scale)
        marker.scale.z = float(scale)
        marker.color.r = float(red)
        marker.color.g = float(green)
        marker.color.b = float(blue)
        marker.color.a = float(alpha)

        return marker


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
