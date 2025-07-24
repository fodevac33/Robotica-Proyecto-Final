#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from tf_transformations import euler_from_quaternion
import math
import random
import numpy as np
from collections import deque
import os
from ament_index_python.packages import get_package_share_directory

class SlamSnakeNavigator(Node):
    def __init__(self):
        super().__init__('slam_snake_navigator')
        
        # Parameters
        self.declare_parameter('goal_tolerance', 0.25)
        self.declare_parameter('fwd_speed', 0.2)
        self.declare_parameter('turn_speed_gain', 0.8)
        self.declare_parameter('world_bounds', [-4.5, 4.5])
        self.declare_parameter('exploration_radius', 2.0)
        self.declare_parameter('stuck_threshold', 300)  # iterations
        self.declare_parameter('stuck_distance', 0.75)  # meters

        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.fwd_speed = self.get_parameter('fwd_speed').value
        self.turn_gain = self.get_parameter('turn_speed_gain').value
        bounds = self.get_parameter('world_bounds').value
        self.x_bounds = (bounds[0], bounds[1])
        self.y_bounds = (bounds[0], bounds[1])
        self.exploration_radius = self.get_parameter('exploration_radius').value
        self.stuck_threshold = self.get_parameter('stuck_threshold').value
        self.stuck_distance = self.get_parameter('stuck_distance').value

        # State
        self.robot_pose = None
        self.target_pos = None
        self.target_name = "food_item"
        self.is_spawning = False
        self.spawn_future = None
        self.delete_future = None
        self.map_data = None
        self.map_info = None
        self.position_history = deque(maxlen=self.stuck_threshold)
        self.visited_targets = []
        self.exploration_mode = False
        
        # Publishers and Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/goal_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        
        # Service Clients
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        
        # Wait for services
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Spawn service not available, waiting...')
        while not self.delete_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Delete service not available, waiting...')
        
        # Main timer
        self.timer = self.create_timer(0.1, self.navigate)
        
        # Service check timer
        self.service_timer = self.create_timer(0.1, self.check_service_futures)
        
        self.get_logger().info("SLAM Snake Navigator Initialized")
        
        # Start with initial spawn
        self.spawn_new_target()

    def check_service_futures(self):
        """Check if any service calls have completed"""
        if self.spawn_future is not None and self.spawn_future.done():
            try:
                response = self.spawn_future.result()
                self.get_logger().info(f"Spawn completed: {response.status_message}")
                self.is_spawning = False
            except Exception as e:
                self.get_logger().error(f"Spawn service call failed: {e}")
                self.is_spawning = False
            self.spawn_future = None
            
        if self.delete_future is not None and self.delete_future.done():
            try:
                response = self.delete_future.result()
                self.get_logger().debug(f"Delete completed: {response.status_message}")
            except Exception as e:
                self.get_logger().error(f"Delete service call failed: {e}")
            self.delete_future = None

    def map_callback(self, msg: OccupancyGrid):
        """Store the occupancy grid for path planning"""
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info

    def odom_callback(self, msg: Odometry):
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.robot_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        
        # Track position history for stuck detection
        if self.robot_pose:
            self.position_history.append((self.robot_pose[0], self.robot_pose[1]))

    def world_to_map(self, wx, wy):
        """Convert world coordinates to map coordinates"""
        if self.map_info is None:
            return None
        mx = int((wx - self.map_info.origin.position.x) / self.map_info.resolution)
        my = int((wy - self.map_info.origin.position.y) / self.map_info.resolution)
        return (mx, my)

    def map_to_world(self, mx, my):
        """Convert map coordinates to world coordinates"""
        if self.map_info is None:
            return None
        wx = mx * self.map_info.resolution + self.map_info.origin.position.x
        wy = my * self.map_info.resolution + self.map_info.origin.position.y
        return (wx, wy)

    def is_position_free(self, x, y):
        """Check if position is free in the map"""
        if self.map_data is None:
            return True  # If no map yet, assume free
        
        map_coords = self.world_to_map(x, y)
        if map_coords is None:
            return False
        
        mx, my = map_coords
        if 0 <= mx < self.map_info.width and 0 <= my < self.map_info.height:
            # 0 = free, 100 = occupied, -1 = unknown
            cell_value = self.map_data[my, mx]
            return cell_value == 0 or cell_value == -1  # Free or unknown
        return False

    def is_stuck(self):
        """Detect if robot is stuck based on position history"""
        if len(self.position_history) < self.stuck_threshold:
            return False
        
        # Calculate total movement over history
        positions = list(self.position_history)
        total_dist = 0
        for i in range(1, len(positions)):
            dx = positions[i][0] - positions[i-1][0]
            dy = positions[i][1] - positions[i-1][1]
            total_dist += math.sqrt(dx*dx + dy*dy)
        
        return total_dist < self.stuck_distance

    def find_exploration_target(self):
        """Find an unexplored area to navigate to"""
        if self.robot_pose is None or self.map_data is None:
            return None
        
        rx, ry, _ = self.robot_pose
        
        # Sample points around robot
        best_target = None
        best_score = -float('inf')
        
        for _ in range(50):
            angle = random.uniform(0, 2 * math.pi)
            dist = random.uniform(0.5, self.exploration_radius)
            tx = rx + dist * math.cos(angle)
            ty = ry + dist * math.sin(angle)
            
            if not (self.x_bounds[0] <= tx <= self.x_bounds[1] and 
                    self.y_bounds[0] <= ty <= self.y_bounds[1]):
                continue
            
            if not self.is_position_free(tx, ty):
                continue
            
            # Score based on distance from visited targets
            score = min([math.sqrt((tx-vx)**2 + (ty-vy)**2) 
                        for vx, vy in self.visited_targets] or [float('inf')])
            
            if score > best_score:
                best_score = score
                best_target = (tx, ty)
        
        return best_target

    def navigate(self):
        if self.robot_pose is None:
            return
        
        # Check if stuck
        if self.is_stuck():
            self.get_logger().warn("Robot appears stuck! Entering exploration mode.")
            self.exploration_mode = True
            self.position_history.clear()
            
            # Find exploration target
            exploration_target = self.find_exploration_target()
            if exploration_target:
                self.target_pos = exploration_target
                self.get_logger().info(f"Exploration target: ({exploration_target[0]:.2f}, {exploration_target[1]:.2f})")
        
        # Normal navigation
        if self.target_pos is None and not self.is_spawning:
            self.spawn_new_target()
            return
        
        if self.target_pos is None:
            return
        
        rx, ry, r_yaw = self.robot_pose
        tx, ty = self.target_pos
        
        dist_to_goal = math.sqrt((tx - rx)**2 + (ty - ry)**2)
        
        if dist_to_goal < self.goal_tolerance:
            self.get_logger().info("Target reached!")
            self.visited_targets.append(self.target_pos)
            if self.exploration_mode:
                self.exploration_mode = False
            self.spawn_new_target()
            return
        
        # Calculate command
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
        
        # Find new target position
        max_attempts = 100
        for attempt in range(max_attempts):
            x = random.uniform(self.x_bounds[0], self.x_bounds[1])
            y = random.uniform(self.y_bounds[0], self.y_bounds[1])
            
            if self.is_position_free(x, y):
                # Additional check: not too close to visited targets
                min_dist_to_visited = min([math.sqrt((x-vx)**2 + (y-vy)**2) 
                                          for vx, vy in self.visited_targets[-10:]] or [float('inf')])
                if min_dist_to_visited > 1.0:  # At least 1m from recent targets
                    self.target_pos = (x, y)
                    break
        else:
            # Fallback: exploration mode
            self.get_logger().warn("Could not find good target, entering exploration")
            exploration_target = self.find_exploration_target()
            if exploration_target:
                self.target_pos = exploration_target
            else:
                # Last resort: random direction
                if self.robot_pose:
                    rx, ry, r_yaw = self.robot_pose
                    self.target_pos = (rx + math.cos(r_yaw), ry + math.sin(r_yaw))
                else:
                    self.target_pos = (0.0, 0.0)
        
        self.get_logger().info(f"New target at: ({self.target_pos[0]:.2f}, {self.target_pos[1]:.2f})")
        self.spawn_entity(self.target_name, self.target_pos[0], self.target_pos[1])

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
            self.is_spawning = False
            return
        
        req = SpawnEntity.Request()
        req.name = name
        req.xml = model_xml
        req.initial_pose.position.x = x
        req.initial_pose.position.y = y
        req.initial_pose.position.z = 0.025
        
        self.get_logger().info("Calling /spawn_entity service...")
        self.spawn_future = self.spawn_client.call_async(req)

    def delete_entity(self, name):
        req = DeleteEntity.Request()
        req.name = name
        self.delete_future = self.delete_client.call_async(req)

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = SlamSnakeNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send stop command
        stop_cmd = Twist()
        node.cmd_pub.publish(stop_cmd)
        node.get_logger().info("Shutting down SLAM navigator.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
