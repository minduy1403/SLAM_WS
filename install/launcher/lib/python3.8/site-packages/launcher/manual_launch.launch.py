#!/usr/bin/env python3
import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # Node teleop
        Node(
            package='teleop_omni',
            executable='teleop_keyboard',
            name='teleop_keyboard',
            output='None',
        ),

        # Node IK
        Node(
            package='kinematics',
            executable='inverse_kinematic',
            name='inverse_kinematic',
            output='None',
        ),

        # Node Serial Driver
        Node(
            package='kinematics',
            executable='serial_driver',
            name='serial_driver',
            output='screen',
            parameters=[{'serial_port': '/dev/ttyUSB0'}],
        ),

    ])
