#!/usr/bin/env python3
import launch
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Inverse Kinematic Node
        Node(
            package='kinematics',
            executable='inverse_kinematic',
            name='inverse_kinematic',
            output='screen',
        ),
        # Serial Driver Node
        Node(
            package='kinematics',
            executable='serial_driver',
            name='serial_driver',
            output='screen',
            parameters=[{'serial_port': '/dev/ttyUSB0'}]
        ),
    ])
