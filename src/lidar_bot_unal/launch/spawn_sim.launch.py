from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share  = FindPackageShare('lidar_bot_unal')
    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'my_bot.urdf.xacro'])
    robot_desc = Command(['xacro ', xacro_file])

    world_file = PathJoinSubstitution([pkg_share, 'worlds', 'test_obstacles.world'])

    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])),
        launch_arguments={
            'pause': 'false',
            'verbose': 'false',
            'world': world_file          
        }.items()
    )

    spawn = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-entity', 'lidar_bot',
                   '-topic', 'robot_description',
                   '-x', '0', '-y', '0', '-z', '0.1'],
        output='screen')

    return LaunchDescription([
        gz_launch,
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_desc}]),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             parameters=[{'robot_description': robot_desc}]),
        spawn
    ])
