#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from aura_msg.msg import Waypoint
from bada_msg.msg import Mode, MissionCurrent
from std_msgs.msg import Float64MultiArray
import utm
import math


class MissionPublisher(Node):
    def __init__(self):
        super().__init__('mission_publisher')

        # --- Publishers ---
        self.waypoint_pub = self.create_publisher(Waypoint, '/bada/waypoints', 10)
        self.mode_pub     = self.create_publisher(Mode, '/bada/mode', 10)
        self.mission_pub  = self.create_publisher(MissionCurrent, '/bada/mission_current', 10)

        # --- Subscriber (UTM state) ---
        self.state_sub = self.create_subscription(
            Float64MultiArray,
            '/bada/estimated_state',
            self.state_callback,
            10
        )

        # --- Timers ---
        self.mode_timer    = self.create_timer(1.0, self.publish_mode)
        self.mission_timer = self.create_timer(1.0, self.publish_mission)
        self.create_timer(2.0, self.publish_waypoints)

        # --- Waypoints (lat/lon) ---
        self.waypoints_latlon = [
            (36.9851667, 126.7955000),
            (36.9823333, 126.7985000),
            (36.9853333, 126.8028333),
            (36.9818333, 126.8065000),
            (36.9841667, 126.8098333),
            (36.9806667, 126.8135000),
            (36.9800000, 126.8186667)
        ]

        # --- Convert all to UTM (meters) ---
        self.waypoints_utm = [utm.from_latlon(lat, lon)[:2] for lat, lon in self.waypoints_latlon]

        self.num_waypoints = len(self.waypoints_utm)
        self.current_seq = 0
        self.reach_threshold = 50.0  # [m]

        self.waypoint_published = False
        self.current_pos = None

        self.get_logger().info("MissionPublisher node started (UTM-based, looping mode).")

    # ------------------------------------------------------------------
    def publish_waypoints(self):
        """Publish fixed waypoints (lat/lon form) only once."""
        if self.waypoint_published:
            return

        msg = Waypoint()
        msg.x_lat = [wp[0] for wp in self.waypoints_latlon]
        msg.y_long = [wp[1] for wp in self.waypoints_latlon]
        msg.num_waypoints = len(msg.x_lat)
        msg.speed_ms = 4.0
        self.waypoint_pub.publish(msg)
        self.get_logger().info("✅ Published /bada/waypoints (lat/lon)")
        self.waypoint_published = True

    # ------------------------------------------------------------------
    def publish_mode(self):
        msg = Mode()
        msg.mode = 5      # FLIGHT_MODE_AUTO_MISSION
        msg.is_armed = 1
        self.mode_pub.publish(msg)
        self.get_logger().info("→ Published /bada/mode (AUTO_MISSION, armed)")

    # ------------------------------------------------------------------
    def publish_mission(self):
        msg = MissionCurrent()
        msg.seq = self.current_seq
        msg.total = self.num_waypoints
        self.mission_pub.publish(msg)
        self.get_logger().info(f"→ Published /bada/mission_current (seq={msg.seq}, total={msg.total})")

    # ------------------------------------------------------------------
    def state_callback(self, msg: Float64MultiArray):
        """
        Expected ship state in UTM coordinates: [x, y, ψ, u, v, r].
        Only x, y are used for waypoint tracking.
        """
        if len(msg.data) < 2:
            return

        x_utm, y_utm = msg.data[0], msg.data[1]
        self.current_pos = (x_utm, y_utm)

        # --- Check distance to current waypoint ---
        target_x, target_y = self.waypoints_utm[self.current_seq]
        dist = math.hypot(target_x - x_utm, target_y - y_utm)
        print(dist)

        if dist < self.reach_threshold:
            self.get_logger().info(f"✅ Waypoint {self.current_seq} reached (dist={dist:.2f} m)")

            # Move to next or loop back
            if self.current_seq < self.num_waypoints - 1:
                self.current_seq += 1
                self.get_logger().info(f"→ Moving to next waypoint {self.current_seq}")
            else:
                self.get_logger().info("🏁 All waypoints completed. Resetting to 0.")
                self.current_seq = 0  # reset sequence

            self.publish_mission()


# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = MissionPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
