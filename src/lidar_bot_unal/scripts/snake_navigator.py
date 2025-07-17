#!/usr/bin/env python3
import rclpy
import xml.etree.ElementTree as ET
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

        self.obstacle_margin = 0.25  # meters of padding around obstacles
        self.obstacles_xy = self.load_world_obstacles(self.obstacle_margin)
        self.get_logger().info(f"Loaded {len(self.obstacles_xy)} obstacle footprints for spawn rejection.")

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

    def load_world_obstacles(self, margin: float):
        """
        Parse the packaged world SDF and return a list of inflated obstacle
        axis-aligned bounding boxes in the XY plane.
        Each entry: (xmin, xmax, ymin, ymax).
        Rotation in the world file is ignored (conservative AABB).
        """
        boxes = []
        try:
            pkg_share = get_package_share_directory('lidar_bot_unal')
            world_path = os.path.join(pkg_share, 'worlds', 'test_obstacles.world')
            tree = ET.parse(world_path)
            root = tree.getroot()
            # SDF <world><model>...</model></world>
            for model in root.findall('.//model'):
                # skip walls? (they’re obstacles too; include)
                pose_elem = model.find('pose')
                if pose_elem is None:
                    continue
                pose_vals = [float(v) for v in pose_elem.text.split()]
                mx, my = pose_vals[0], pose_vals[1]
                # Find first box size (if any)
                size_elem = model.find('.//box/size')
                if size_elem is None:
                    continue
                sx, sy, _sz = [float(v) for v in size_elem.text.split()]
                half_x = sx / 2.0 + margin
                half_y = sy / 2.0 + margin
                boxes.append((mx - half_x, mx + half_x, my - half_y, my + half_y))
        except Exception as e:
            self.get_logger().error(f"Failed to parse world obstacles: {e}")
        return boxes

    def xy_is_free(self, x: float, y: float) -> bool:
        """
        Return True if (x,y) lies outside all inflated obstacle AABBs.
        """
        for (xmin, xmax, ymin, ymax) in self.obstacles_xy:
            if xmin <= x <= xmax and ymin <= y <= ymax:
                return False
        return True

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

        # Remove previous target if present
        if self.target_pos is not None:
            self.delete_entity(self.target_name)

        # Rejection sample in bounds until free
        max_attempts = 100
        for attempt in range(max_attempts):
            x = random.uniform(self.x_bounds[0], self.x_bounds[1])
            y = random.uniform(self.y_bounds[0], self.y_bounds[1])
            if self.xy_is_free(x, y):
                self.target_pos = (x, y)
                break
        else:
            # Fallback: spawn 1m ahead of robot (if we know pose), else origin
            if self.robot_pose is not None:
                rx, ry, r_yaw = self.robot_pose
                self.target_pos = (rx + math.cos(r_yaw), ry + math.sin(r_yaw))
            else:
                self.target_pos = (0.0, 0.0)

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

