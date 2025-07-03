#!/usr/bin/env python3
"""
Reactive obstacle-avoidance for a differential drive robot with a 180-degree LiDAR.
Publishes to /cmd_vel using a simple “go-forward-unless-obstacle” rule.
"""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class AvoidNode(Node):

    def __init__(self):
        super().__init__('avoid_obstacles')

        # Using a safer distance now that the robot won't see itself
        self.declare_parameter('safe_dist', 0.35) 
        self.declare_parameter('fwd_speed', 0.20)
        self.declare_parameter('turn_speed', 0.8)

        self.safe_dist  = self.get_parameter('safe_dist').value
        self.fwd_speed  = self.get_parameter('fwd_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub     = self.create_subscription(LaserScan, '/scan',
                                                self.scan_cb, 10)
        
        self.get_logger().info("Obstacle Avoider node started.")

    def scan_cb(self, msg: LaserScan):
        cmd = Twist()
        num_ranges = len(msg.ranges)
        
        # --- Get distance in a narrow forward-facing cone (e.g., central 10% of readings) ---
        center_idx = num_ranges // 2
        cone_width = num_ranges // 10 # 10% of readings
        fwd_min_i = center_idx - cone_width // 2
        fwd_max_i = center_idx + cone_width // 2
        
        fwd_sector = msg.ranges[fwd_min_i : fwd_max_i]
        valid_fwd_sector = [r for r in fwd_sector if not math.isinf(r) and not math.isnan(r)]
        min_dist = min(valid_fwd_sector) if valid_fwd_sector else float('inf')

        if min_dist > self.safe_dist:
            cmd.linear.x = self.fwd_speed
            cmd.angular.z = 0.0
        else:
            # Obstacle detected, stop and turn.
            cmd.linear.x = 0.0
            self.get_logger().info(f"Obstacle detected at {min_dist:.2f}m. Deciding which way to turn.")

            # --- SIMPLIFIED LOGIC for 180° LiDAR ---
            # Right side is the first half of the array
            # Left side is the second half of the array
            
            right_sector = msg.ranges[0 : center_idx]
            left_sector = msg.ranges[center_idx : num_ranges]

            valid_right = [r for r in right_sector if not math.isinf(r) and not math.isnan(r)]
            min_right = min(valid_right) if valid_right else float('inf')

            valid_left = [r for r in left_sector if not math.isinf(r) and not math.isnan(r)]
            min_left = min(valid_left) if valid_left else float('inf')
            
            # Turn towards the direction with more space
            if min_right > min_left:
                self.get_logger().info(f"Turning RIGHT (R_space: {min_right:.2f} > L_space: {min_left:.2f})")
                cmd.angular.z = -self.turn_speed # Turn right
            else:
                self.get_logger().info(f"Turning LEFT (L_space: {min_left:.2f} >= R_space: {min_right:.2f})")
                cmd.angular.z = self.turn_speed  # Turn left
        
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = AvoidNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot on shutdown
        shutdown_cmd = Twist()
        node.cmd_pub.publish(shutdown_cmd)
        node.get_logger().info("Shutting down and stopping robot.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()