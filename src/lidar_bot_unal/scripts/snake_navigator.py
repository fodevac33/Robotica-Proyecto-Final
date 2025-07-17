#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from tf_transformations import euler_from_quaternion
import math
import random
import os
from ament_index_python.packages import get_package_share_directory # <-- WAS MISSING

class SnakeNavigator(Node):
    def __init__(self):
        super().__init__('snake_navigator')
        
        # Parameters
        self.declare_parameter('goal_tolerance', 0.25)
        self.declare_parameter('fwd_speed', 0.2)
        self.declare_parameter('turn_speed_gain', 0.8)
        self.declare_parameter('world_bounds', [-4.5, 4.5])

        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.fwd_speed = self.get_parameter('fwd_speed').value
        self.turn_gain = self.get_parameter('turn_speed_gain').value
        bounds = self.get_parameter('world_bounds').value
        self.x_bounds = (bounds[0], bounds[1])
        self.y_bounds = (bounds[0], bounds[1])

        # State
        self.robot_pose = None
        self.target_pos = None
        self.target_name = "food_item"
        self.is_spawning = False #<-- ADDED: Flag to prevent multiple spawns

        # Publishers and Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/goal_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Service Clients for Gazebo
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Spawn service not available, waiting again...')
        while not self.delete_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Delete service not available, waiting again...')
        
        # Main logic timer
        self.timer = self.create_timer(0.1, self.navigate)
        self.get_logger().info("Snake Navigator Initialized. Spawning first target...")
        self.spawn_new_target()

    def odom_callback(self, msg: Odometry):
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.robot_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def navigate(self):
        if self.robot_pose is None or self.target_pos is None or self.is_spawning:
            return

        rx, ry, r_yaw = self.robot_pose
        tx, ty = self.target_pos

        dist_to_goal = math.sqrt((tx - rx)**2 + (ty - ry)**2)
        if dist_to_goal < self.goal_tolerance:
            self.get_logger().info("Target reached! Spawning a new one.")
            self.spawn_new_target()
            return

        angle_to_goal = math.atan2(ty - ry, tx - rx)
        angle_error = self.normalize_angle(angle_to_goal - r_yaw)

        cmd = Twist()
        cmd.angular.z = self.turn_gain * angle_error
        
        if abs(angle_error) < math.pi / 4:
            cmd.linear.x = self.fwd_speed
        else:
            cmd.linear.x = self.fwd_speed * 0.25

        self.cmd_pub.publish(cmd)

    def spawn_new_target(self):
        if self.is_spawning:
            return
        self.is_spawning = True

        if self.target_pos is not None:
            self.delete_entity(self.target_name)

        self.target_pos = (
            random.uniform(self.x_bounds[0], self.x_bounds[1]),
            random.uniform(self.y_bounds[0], self.y_bounds[1])
        )
        self.get_logger().info(f"New target at: ({self.target_pos[0]:.2f}, {self.target_pos[1]:.2f})")
        
        self.spawn_entity(self.target_name, self.target_pos[0], self.target_pos[1])
        self.is_spawning = False

    def get_food_model_xml(self):
        pkg_share = get_package_share_directory('lidar_bot_unal')
        sdf_file_path = os.path.join(pkg_share, 'models', 'food', 'model.sdf')
        try:
            with open(sdf_file_path, 'r') as f:
                return f.read()
        except FileNotFoundError:
            self.get_logger().error(f"Food model SDF not found at: {sdf_file_path}")
            return None

    def spawn_entity(self, name, x, y):
        model_xml = self.get_food_model_xml()
        if model_xml is None:
            return #<-- ADDED: Stop if file reading failed

        req = SpawnEntity.Request()
        req.name = name
        req.xml = model_xml
        req.initial_pose.position.x = x
        req.initial_pose.position.y = y
        req.initial_pose.position.z = 0.025
        
        # Use a synchronous call with logging to see the result <-- MAJOR CHANGE
        self.get_logger().info("Calling /spawn_entity service...")
        future = self.spawn_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"Spawn service response: {future.result().status_message}")
        else:
            self.get_logger().error(f"Exception while calling service: {future.exception()}")


    def delete_entity(self, name):
        req = DeleteEntity.Request()
        req.name = name
        future = self.delete_client.call_async(req)
        rclpy.spin_until_future_complete(self, future) # Also make this sync

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = SnakeNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down navigator.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

