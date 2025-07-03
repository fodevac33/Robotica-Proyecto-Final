#!/usr/bin/env python3
"""
Reactive obstacle-avoidance that considers the robot's width.
Publishes to /cmd_vel using a "safety bubble" approach.
"""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class AvoidNode(Node):

    def __init__(self):
        super().__init__('avoid_obstacles')

        # Robot physical properties
        self.declare_parameter('robot_width', 0.30)  # The actual width of the robot chassis
        self.declare_parameter('side_buffer', 0.10)  # Extra buffer space for the sides

        # Control parameters
        self.declare_parameter('fwd_safe_dist', 0.35) 
        self.declare_parameter('fwd_speed', 0.20)
        self.declare_parameter('turn_speed', 0.8)

        # Get parameters
        robot_width = self.get_parameter('robot_width').value
        side_buffer = self.get_parameter('side_buffer').value
        self.fwd_safe_dist = self.get_parameter('fwd_safe_dist').value
        self.fwd_speed = self.get_parameter('fwd_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value

        # Calculate the distance to keep the sides clear
        # This is the minimum distance the side-checker rays must report
        self.side_safe_dist = (robot_width / 2) + side_buffer

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        
        self.get_logger().info("Obstacle Avoider with 'Safety Bubble' started.")
        self.get_logger().info(f"Side clearance distance set to: {self.side_safe_dist:.2f}m")

    def get_index_for_angle(self, angle: float, msg: LaserScan) -> int:
        """Helper function to get the index in the ranges array for a given angle."""
        angle = max(msg.angle_min, min(msg.angle_max, angle))
        return int((angle - msg.angle_min) / msg.angle_increment)

    def scan_cb(self, msg: LaserScan):
        cmd = Twist()
        
        # === 1. FORWARD COLLISION CHECK ===
        # Check a narrow cone directly in front of the robot.
        fwd_cone_angle = 10 * math.pi / 180  # 10 degree cone
        fwd_min_i = self.get_index_for_angle(-fwd_cone_angle / 2, msg)
        fwd_max_i = self.get_index_for_angle(fwd_cone_angle / 2, msg)
        
        fwd_sector = msg.ranges[fwd_min_i : fwd_max_i]
        valid_fwd = [r for r in fwd_sector if not math.isinf(r) and not math.isnan(r)]
        min_fwd_dist = min(valid_fwd) if valid_fwd else float('inf')

        # === 2. SIDE CLEARANCE CHECK ===
        # Check rays corresponding to the robot's corners. We use ±90 degrees.
        right_corner_idx = self.get_index_for_angle(-math.pi / 2, msg)
        left_corner_idx = self.get_index_for_angle(math.pi / 2, msg)

        right_corner_dist = msg.ranges[right_corner_idx]
        left_corner_dist = msg.ranges[left_corner_idx]

        # === 3. DECISION LOGIC ===
        
        # Condition 1: Imminent collision detected straight ahead.
        # Action: Stop and turn away from the nearest overall obstacle.
        if min_fwd_dist < self.fwd_safe_dist:
            cmd.linear.x = 0.0
            
            # Find minimums on the entire left and right arcs to decide turn direction
            center_idx = len(msg.ranges) // 2
            right_half = msg.ranges[:center_idx]
            left_half = msg.ranges[center_idx:]
            min_right = min([r for r in right_half if not math.isinf(r) and not math.isnan(r)] or [float('inf')])
            min_left = min([r for r in left_half if not math.isinf(r) and not math.isnan(r)] or [float('inf')])

            self.get_logger().info(f"FORWARD OBSTACLE at {min_fwd_dist:.2f}m. Turning.")
            cmd.angular.z = self.turn_speed if min_right < min_left else -self.turn_speed

        # Condition 2: Sides are not clear enough to move forward.
        # Action: Stop and perform a small turn away from the tight side.
        elif left_corner_dist < self.side_safe_dist or right_corner_dist < self.side_safe_dist:
            cmd.linear.x = 0.0
            
            self.get_logger().info(f"SIDE CLEARANCE ISSUE (L:{left_corner_dist:.2f} R:{right_corner_dist:.2f}). Turning.")
            # Turn away from the closer side
            cmd.angular.z = self.turn_speed if right_corner_dist < left_corner_dist else -self.turn_speed
            
        # Condition 3: Path is clear ahead and on the sides.
        # Action: Move forward.
        else:
            cmd.linear.x = self.fwd_speed
            cmd.angular.z = 0.0

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