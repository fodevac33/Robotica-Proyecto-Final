from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    # Reuse the full sim bring-up (Gazebo + spawn + state publishers)
    sim_launch_file = PathJoinSubstitution(
        [FindPackageShare('lidar_bot_unal'), 'launch', 'spawn_sim.launch.py']
    )

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(sim_launch_file)),

        # Run the simple obstacle avoidance controller that publishes directly to /cmd_vel
        Node(
            package='lidar_bot_unal',
            executable='avoid.py',
            name='avoid_obstacles',
            output='screen',
        ),
    ])
