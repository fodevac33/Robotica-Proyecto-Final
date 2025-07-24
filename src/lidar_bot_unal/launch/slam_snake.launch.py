from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    pkg_share = FindPackageShare('lidar_bot_unal')
    
    # Include the simulation
    sim_launch_file = PathJoinSubstitution(
        [pkg_share, 'launch', 'spawn_sim.launch.py']
    )
    
    # Include SLAM
    slam_launch_file = PathJoinSubstitution(
        [pkg_share, 'launch', 'slam.launch.py']
    )
    
    return LaunchDescription([
        # Launch simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch_file)
        ),
        
        # Launch SLAM
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_file)
        ),
        
        # Launch obstacle avoider
        Node(
            package='lidar_bot_unal',
            executable='obstacle_avoider.py',
            name='obstacle_avoider',
            output='screen'
        ),
        
        # Launch SLAM-aware snake navigator
        Node(
            package='lidar_bot_unal',
            executable='slam_snake_navigator.py',
            name='slam_snake_navigator',
            output='screen'
        ),
        
        # Optional: Launch RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([pkg_share, 'rviz', 'slam_view.rviz'])],
            output='screen'
        )
    ])
