#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Tìm file Xacro đã được cài vào share/omnibot_description/urdf
    xacro_file = PathJoinSubstitution([
        FindPackageShare('omnibot_description'),
        'urdf',
        'omnibot.urdf.xacro'
    ])

    # Tạo command: "xacro <space> <xacro_file>"
    robot_description = Command([
        TextSubstitution(text='xacro'),
        TextSubstitution(text=' '),
        xacro_file,
    ])

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description
            }],
        ),
    ])
