#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoiderNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')

        # Parameters for obstacle avoidance
        self.declare_parameter('robot_width', 0.30)
        self.declare_parameter('side_buffer', 0.10)
        self.declare_parameter('fwd_safe_dist', 0.35)
        self.declare_parameter('turn_speed', 0.8)

        robot_width = self.get_parameter('robot_width').value
        side_buffer = self.get_parameter('side_buffer').value
        self.fwd_safe_dist = self.get_parameter('fwd_safe_dist').value
        self.base_turn_speed = self.get_parameter('turn_speed').value
        self.side_safe_dist = (robot_width / 2) + side_buffer

        # State
        self.goal_cmd = Twist()          # Command from the navigator
        self.prev_final_cmd = Twist()    # For smoothing

        # Publishers and Subscribers
        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.goal_sub = self.create_subscription(Twist, '/goal_vel', self.goal_cb, 10)

        self.get_logger().info("Obstacle Avoider ready and waiting for goal commands.")

    def goal_cb(self, msg: Twist):
        self.goal_cmd = msg

    def get_index_for_angle(self, angle: float, msg: LaserScan) -> int:
        angle = max(msg.angle_min, min(msg.angle_max, angle))
        return int((angle - msg.angle_min) / msg.angle_increment)

    def min_turn(self, val: float, minimum: float = 0.2) -> float:
        """Guarantee at least a small nonzero turn magnitude so we don't stall."""
        if val >= 0.0:
            return max(val, minimum)
        else:
            return min(val, -minimum)

    def scan_cb(self, msg: LaserScan):
        # The goal command from the navigator
        goal_cmd = self.goal_cmd

        # --- Find the clearest path (the direction with the furthest laser reading) ---
        # We replace NaN/inf with 0 so we can find the max range safely.
        # A 0 range won't be chosen unless all ranges are 0.
        ranges = np.array(msg.ranges)
        ranges[np.isinf(ranges)] = msg.range_max + 1 # Treat inf as wide open space
        ranges[np.isnan(ranges)] = 0.0 # Treat nan as an immediate obstacle

        # The index of the laser scan with the longest range
        best_index = np.argmax(ranges)
        # The angle of that best opening
        best_angle = msg.angle_min + best_index * msg.angle_increment

        # --- Obstacle Detection in Front ---
        fwd_cone_angle = 20 * math.pi / 180.0  # 20-degree cone
        fwd_min_i = self.get_index_for_angle(-fwd_cone_angle / 2, msg)
        fwd_max_i = self.get_index_for_angle(fwd_cone_angle / 2, msg)
        min_fwd_dist = np.min(ranges[fwd_min_i:fwd_max_i])

        # --- Decision Logic ---
        final_cmd = Twist()
        obstacle_detected = False

        if min_fwd_dist < self.fwd_safe_dist:
            # OBSTACLE DETECTED: Don't use the navigator's command.
            # Instead, turn towards the best angle (the clearest path).
            self.get_logger().info(f"Obstacle ahead! Turning towards opening at {best_angle:.2f} rad.", throttle_duration_sec=1.0)
            obstacle_detected = True

            # Slow down significantly when turning away from an obstacle
            final_cmd.linear.x = goal_cmd.linear.x * 0.2
            
            # Steer towards the opening
            turn_error = self.normalize_angle(best_angle) # The "goal" is now the opening
            final_cmd.angular.z = self.base_turn_speed * turn_error * 2.0 # Increase turn gain for escape
            
            # Ensure the robot always tries to turn out of trouble
            if abs(final_cmd.angular.z) < 0.1:
                final_cmd.angular.z = np.sign(turn_error) * 0.3

        else:
            # NO OBSTACLE: Pass the navigator's command through.
            final_cmd = goal_cmd

        # --- Smoothing and Publishing ---
        # Smoothing helps prevent jerky movements
        alpha = 0.6
        smooth_cmd = Twist()
        smooth_cmd.linear.x  = alpha * final_cmd.linear.x  + (1 - alpha) * self.prev_final_cmd.linear.x
        smooth_cmd.angular.z = alpha * final_cmd.angular.z + (1 - alpha) * self.prev_final_cmd.angular.z

        self.cmd_pub.publish(smooth_cmd)
        self.prev_final_cmd = smooth_cmd

    def normalize_angle(self, angle): # <-- ADD THIS HELPER FUNCTION TO THE CLASS
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoiderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        shutdown_cmd = Twist()
        node.cmd_pub.publish(shutdown_cmd)
        node.get_logger().info("Shutting down and stopping robot.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
