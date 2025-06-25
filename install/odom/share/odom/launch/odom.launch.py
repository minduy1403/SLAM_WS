#!/usr/bin/env python3
import launch
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Inverse Kinematic Node: chỉ hiển thị WARN và cao hơn
        Node(
            package='kinematics',
            executable='inverse_kinematic',
            name='inverse_kinematic',
            output='screen',
            emulate_tty=True,
            arguments=['--ros-args', '--log-level', 'warn'],
        ),
        # Serial Driver Node: chỉ hiển thị WARN và cao hơn
        Node(
            package='kinematics',
            executable='serial_driver',
            name='serial_driver',
            output='screen',
            emulate_tty=True,
            arguments=['--ros-args', '--log-level', 'warn'],
        ),
        # Odom Node: hiển thị INFO và cao hơn
        Node(
            package='odom',
            executable='odom_node',
            name='odom_node',
            output='screen',
            emulate_tty=True,
            arguments=['--ros-args', '--log-level', 'info'],
            parameters=[{
                'odom_frame': 'odom',
                'base_frame': 'base_link',
            }],
        ),
    ])
