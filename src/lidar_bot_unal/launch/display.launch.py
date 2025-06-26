from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('lidar_bot_unal')
    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'my_bot.urdf.xacro'])
    robot_description = Command(['xacro ', xacro_file])

    return LaunchDescription([
        # Publica TF fijas
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),

        # Publica JointState=0 para las ruedas
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),

        # Abre RViz con la vista guardada
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=[
                '-d', PathJoinSubstitution([pkg_share, 'rviz', 'my_bot.rviz'])
            ],
            output='screen'
        )
    ])
