#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from aura_msg.msg import Waypoint


class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')
        
        # Create publisher for waypoints topic
        self.publisher_ = self.create_publisher(Waypoint, '/bada/waypoints', 10)
        
        # Timer to publish waypoints once after a short delay
        self.timer = self.create_timer(2.0, self.publish_waypoints)
        
        self.get_logger().info('Waypoint Publisher Node has been started')

    def publish_waypoints(self):
        # Create waypoint message
        waypoint_msg = Waypoint()
        
        # Define 5 arbitrary waypoints (lat, lon coordinates)
        # These are example coordinates around a coastal area
        latitudes = [
            35.123456,   # Waypoint 1
            35.125678,   # Waypoint 2  
            35.128901,   # Waypoint 3
            35.131234,   # Waypoint 4
            35.134567    # Waypoint 5
        ]
        
        longitudes = [
            129.123456,  # Waypoint 1
            129.126789,  # Waypoint 2
            129.130123,  # Waypoint 3
            129.133456,  # Waypoint 4
            129.136789   # Waypoint 5
        ]
        
        # Set waypoint data
        waypoint_msg.x_lat = latitudes
        waypoint_msg.y_long = longitudes
        waypoint_msg.num_waypoints = len(latitudes)
        
        # Publish the message
        self.publisher_.publish(waypoint_msg)
        
        self.get_logger().info(f'Published {waypoint_msg.num_waypoints} waypoints:')
        for i in range(waypoint_msg.num_waypoints):
            self.get_logger().info(f'  Waypoint {i+1}: lat={waypoint_msg.x_lat[i]:.6f}, lon={waypoint_msg.y_long[i]:.6f}')
        
        # Stop the timer after publishing once
        self.timer.cancel()
        
        # Shutdown after 1 second
        self.create_timer(1.0, self.shutdown_node)

    def shutdown_node(self):
        self.get_logger().info('Waypoints published successfully. Shutting down...')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    
    waypoint_publisher = WaypointPublisher()
    
    try:
        rclpy.spin(waypoint_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        waypoint_publisher.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()