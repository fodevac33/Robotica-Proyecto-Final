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
        final_cmd = Twist()
        obstacle_detected = False

        # --- Calculate obstacle metrics ---
        fwd_cone_angle = 15 * math.pi / 180.0
        fwd_min_i = self.get_index_for_angle(-fwd_cone_angle / 2, msg)
        fwd_max_i = self.get_index_for_angle(fwd_cone_angle / 2, msg)

        fwd_sector = msg.ranges[fwd_min_i:fwd_max_i]
        valid_fwd = [r for r in fwd_sector if not math.isinf(r) and not math.isnan(r)]
        min_fwd_dist = min(valid_fwd) if valid_fwd else float('inf')

        right_corner_idx = self.get_index_for_angle(-math.pi / 2, msg)
        left_corner_idx  = self.get_index_for_angle(math.pi / 2, msg)
        right_corner_dist = msg.ranges[right_corner_idx] if not (math.isinf(msg.ranges[right_corner_idx]) or math.isnan(msg.ranges[right_corner_idx])) else float('inf')
        left_corner_dist  = msg.ranges[left_corner_idx]  if not (math.isinf(msg.ranges[left_corner_idx])  or math.isnan(msg.ranges[left_corner_idx]))  else float('inf')

        center_idx = len(msg.ranges) // 2
        min_right = min([r for r in msg.ranges[:center_idx] if not (math.isinf(r) or math.isnan(r))] or [float('inf')])
        min_left  = min([r for r in msg.ranges[center_idx:] if not (math.isinf(r) or math.isnan(r))] or [float('inf')])

        # DEBUG distances (1 Hz)
        self.get_logger().info(
            f"dist fwd={min_fwd_dist:.2f}  right={right_corner_dist:.2f}  left={left_corner_dist:.2f}  "
            f"minR={min_right:.2f} minL={min_left:.2f}",
            throttle_duration_sec=1.0
        )

        # --- Relax avoidance when navigator is creeping (near goal) ---
        # If navigator linear command is tiny, trust it unless obstacle is extremely close.
        if abs(self.goal_cmd.linear.x) < 0.05:
            if min_fwd_dist < 0.10:
                obstacle_detected = True
                final_cmd.linear.x = 0.0
                final_cmd.angular.z = self.min_turn(self.base_turn_speed)
            else:
                final_cmd = self.goal_cmd
        else:
            # --- Forward obstacle branch ---
            if min_fwd_dist < self.fwd_safe_dist:
                obstacle_detected = True
                final_cmd.linear.x = 0.0
                if math.isinf(min_right) and math.isinf(min_left):
                    turn = self.base_turn_speed
                elif min_right < min_left:
                    turn = self.base_turn_speed
                else:
                    turn = -self.base_turn_speed
                final_cmd.angular.z = self.min_turn(turn)

            # --- Side clearance branch ---
            elif left_corner_dist < self.side_safe_dist or right_corner_dist < self.side_safe_dist:
                obstacle_detected = True
                final_cmd.linear.x = self.goal_cmd.linear.x * 0.3
                if math.isinf(right_corner_dist) and math.isinf(left_corner_dist):
                    turn = self.base_turn_speed
                elif right_corner_dist < left_corner_dist:
                    turn = self.base_turn_speed
                else:
                    turn = -self.base_turn_speed
                final_cmd.angular.z = self.min_turn(turn, minimum=0.15)

            # --- No obstacle detected: pass navigator command through ---
            else:
                final_cmd = self.goal_cmd

        # Log (throttled) if we overrode
        if obstacle_detected:
            self.get_logger().info("Obstacle detected! Overriding goal.", throttle_duration_sec=1.0)

        # Smooth
        alpha = 0.5
        smooth_cmd = Twist()
        smooth_cmd.linear.x  = alpha * final_cmd.linear.x  + (1 - alpha) * self.prev_final_cmd.linear.x
        smooth_cmd.angular.z = alpha * final_cmd.angular.z + (1 - alpha) * self.prev_final_cmd.angular.z

        self.cmd_pub.publish(smooth_cmd)
        self.prev_final_cmd = smooth_cmd

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
