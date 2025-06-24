from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Path to the Xacro file
    xacro_file = PathJoinSubstitution([
        FindPackageShare('omnibot_description'),
        'urdf',
        'omnibot.urdf.xacro'
    ])

    # Generate robot_description parameter by processing Xacro
    robot_description = Command([
        FindExecutable(name='xacro'),
        ' ',
        xacro_file
    ])

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        )
    ])
