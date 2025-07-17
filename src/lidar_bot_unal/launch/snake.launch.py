from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    # Path to the simulation launch file
    sim_launch_file = PathJoinSubstitution(
        [FindPackageShare('lidar_bot_unal'), 'launch', 'spawn_sim.launch.py']
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch_file)
        ),

        Node(
            package='lidar_bot_unal',
            executable='obstacle_avoider.py',
            name='obstacle_avoider',
            output='screen'
        ),

        Node(
            package='lidar_bot_unal',
            executable='snake_navigator.py',
            name='snake_navigator',
            output='screen'
        )
    ])