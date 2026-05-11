import math
import time
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix

from mavros_msgs.msg import Waypoint
from mavros_msgs.srv import WaypointClear, WaypointPush, SetMode, CommandBool


class MavrosMissionUploader(Node):
    MAV_CMD_NAV_WAYPOINT = 16
    MAV_CMD_NAV_TAKEOFF = 22
    FRAME_GLOBAL_REL_ALT = 3

    def __init__(self):
        super().__init__("mavros_mission_uploader")

        self.declare_parameter("path_topic", "/planned_path")
        self.declare_parameter("gps_topic", "/mavros/global_position/global")
        self.declare_parameter("arm_and_start", False)
        self.declare_parameter("acceptance_radius", 2.0)
        self.declare_parameter("takeoff_altitude", 3.0)
        self.declare_parameter("input_timeout_sec", 60.0)

        self.path_topic = self.get_parameter("path_topic").value
        self.gps_topic = self.get_parameter("gps_topic").value
        self.arm_and_start = bool(self.get_parameter("arm_and_start").value)
        self.acceptance_radius = float(self.get_parameter("acceptance_radius").value)
        self.takeoff_altitude = float(self.get_parameter("takeoff_altitude").value)
        self.input_timeout_sec = float(self.get_parameter("input_timeout_sec").value)

        self.home_gps: Optional[NavSatFix] = None
        self.latest_path: Optional[Path] = None

        self.path_sub = self.create_subscription(
            Path,
            self.path_topic,
            self.path_callback,
            10,
        )

        self.gps_sub = self.create_subscription(
            NavSatFix,
            self.gps_topic,
            self.gps_callback,
            qos_profile_sensor_data,
        )

        self.clear_client = self.create_client(WaypointClear, "/mavros/mission/clear")
        self.push_client = self.create_client(WaypointPush, "/mavros/mission/push")
        self.arm_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.mode_client = self.create_client(SetMode, "/mavros/set_mode")

        self.get_logger().info("MAVROS mission uploader started.")
        self.get_logger().info(f"Listening for path on: {self.path_topic}")
        self.get_logger().info(f"Listening for GPS on: {self.gps_topic}")
        self.get_logger().info(f"arm_and_start: {self.arm_and_start}")

    def gps_callback(self, msg: NavSatFix):
        if self.home_gps is None:
            self.home_gps = msg
            self.get_logger().info(
                "Home GPS fixed at "
                f"lat={msg.latitude:.8f}, "
                f"lon={msg.longitude:.8f}, "
                f"alt={msg.altitude:.2f}"
            )

    def path_callback(self, msg: Path):
        if self.latest_path is None and len(msg.poses) >= 2:
            self.latest_path = msg
            self.get_logger().info(f"Received planned path with {len(msg.poses)} poses.")

    def wait_for_inputs(self) -> bool:
        self.get_logger().info("Waiting for GPS and planned path...")

        start_time = time.time()
        last_log_time = 0.0

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)

            gps_ready = self.home_gps is not None
            path_ready = self.latest_path is not None

            if gps_ready and path_ready:
                self.get_logger().info("GPS and path are ready.")
                return True

            now = time.time()

            if now - last_log_time > 5.0:
                self.get_logger().info(
                    f"Still waiting... GPS ready: {gps_ready}, path ready: {path_ready}"
                )
                last_log_time = now

            if now - start_time > self.input_timeout_sec:
                self.get_logger().error("Timed out waiting for GPS/path.")
                return False

        return False

    def path_to_waypoints(self, path_msg: Path) -> List[Waypoint]:
        waypoints: List[Waypoint] = []

        first_pose = path_msg.poses[0].pose.position
        first_alt = max(float(first_pose.z), self.takeoff_altitude)

        waypoints.append(
            self.make_waypoint(
                local_x=float(first_pose.x),
                local_y=float(first_pose.y),
                rel_alt=first_alt,
                command=self.MAV_CMD_NAV_TAKEOFF,
                is_current=True,
            )
        )

        for pose_stamped in path_msg.poses[1:]:
            p = pose_stamped.pose.position

            waypoints.append(
                self.make_waypoint(
                    local_x=float(p.x),
                    local_y=float(p.y),
                    rel_alt=float(p.z),
                    command=self.MAV_CMD_NAV_WAYPOINT,
                    is_current=False,
                )
            )

        return waypoints

    def make_waypoint(self, local_x, local_y, rel_alt, command, is_current):
        lat, lon = self.local_enu_to_gps(local_x, local_y)

        wp = Waypoint()
        wp.frame = self.FRAME_GLOBAL_REL_ALT
        wp.command = command
        wp.is_current = is_current
        wp.autocontinue = True
        wp.param1 = 0.0
        wp.param2 = self.acceptance_radius
        wp.param3 = 0.0
        wp.param4 = 0.0
        wp.x_lat = float(lat)
        wp.y_long = float(lon)
        wp.z_alt = float(rel_alt)

        return wp

    def local_enu_to_gps(self, east_m, north_m):
        if self.home_gps is None:
            raise RuntimeError("No home GPS fix available")

        lat0_rad = math.radians(self.home_gps.latitude)

        delta_lat = north_m / 111111.0
        delta_lon = east_m / (111111.0 * math.cos(lat0_rad))

        lat = self.home_gps.latitude + delta_lat
        lon = self.home_gps.longitude + delta_lon

        return lat, lon

    def wait_for_services(self) -> bool:
        required_services = [
            (self.clear_client, "/mavros/mission/clear"),
            (self.push_client, "/mavros/mission/push"),
        ]

        if self.arm_and_start:
            required_services.extend([
                (self.arm_client, "/mavros/cmd/arming"),
                (self.mode_client, "/mavros/set_mode"),
            ])

        for client, name in required_services:
            self.get_logger().info(f"Waiting for service {name}...")
            if not client.wait_for_service(timeout_sec=10.0):
                self.get_logger().error(f"Service not available: {name}")
                return False

        return True

    def call_service_and_wait(self, client, request, timeout_sec=10.0):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

        if future.result() is None:
            return None

        return future.result()

    def clear_and_upload(self, waypoints: List[Waypoint]) -> bool:
        if not self.wait_for_services():
            return False

        self.get_logger().info("Clearing previous PX4 mission...")

        clear_result = self.call_service_and_wait(
            self.clear_client,
            WaypointClear.Request(),
            timeout_sec=10.0,
        )

        if clear_result is None:
            self.get_logger().error("Mission clear service call failed or timed out.")
            return False

        if not clear_result.success:
            self.get_logger().error("PX4 reported mission clear failed.")
            return False

        self.get_logger().info("Previous mission cleared.")
        self.get_logger().info("Uploading new mission...")

        push_req = WaypointPush.Request()
        push_req.start_index = 0
        push_req.waypoints = waypoints

        push_result = self.call_service_and_wait(
            self.push_client,
            push_req,
            timeout_sec=20.0,
        )

        if push_result is None:
            self.get_logger().error("Mission push service call failed or timed out.")
            return False

        self.get_logger().info(
            f"Mission push result: success={push_result.success}, "
            f"transferred={push_result.wp_transfered}"
        )

        return bool(push_result.success)

    def arm_vehicle(self) -> bool:
        req = CommandBool.Request()
        req.value = True

        result = self.call_service_and_wait(self.arm_client, req, timeout_sec=10.0)

        if result is None:
            self.get_logger().error("Arming service call failed.")
            return False

        self.get_logger().info(f"Arming result: success={result.success}")
        return bool(result.success)

    def set_auto_mission_mode(self) -> bool:
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = "AUTO.MISSION"

        result = self.call_service_and_wait(self.mode_client, req, timeout_sec=10.0)

        if result is None:
            self.get_logger().error("Set mode service call failed.")
            return False

        self.get_logger().info(f"Set AUTO.MISSION result: mode_sent={result.mode_sent}")
        return bool(result.mode_sent)


def main(args=None):
    rclpy.init(args=args)
    node = MavrosMissionUploader()

    try:
        if not node.wait_for_inputs():
            return

        waypoints = node.path_to_waypoints(node.latest_path)
        node.get_logger().info(f"Converted path to {len(waypoints)} MAVROS mission items.")

        success = node.clear_and_upload(waypoints)

        if not success:
            node.get_logger().error("Mission upload failed.")
            return

        node.get_logger().info("Mission upload complete.")

        if node.arm_and_start:
            node.get_logger().warn("arm_and_start is TRUE. Arming and starting AUTO.MISSION.")
            node.arm_vehicle()
            node.set_auto_mission_mode()
        else:
            node.get_logger().info("arm_and_start is FALSE. Mission was uploaded only.")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
