#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, TextSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # --- Chuẩn bị robot_description từ URDF xacro ---
    pkg = FindPackageShare('omnibot_description')
    xacro_path = PathJoinSubstitution([
        pkg, 'urdf', 'omnibot.urdf.xacro'
    ])
    robot_desc = Command([
        TextSubstitution(text='xacro '),
        xacro_path
    ])
    robot_description_param = ParameterValue(robot_desc, value_type=str)

    # --- robot_state_publisher (URDF -> /tf_static) ---
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_param}],
    )

    # --- odom_node (dynamic odom -> base_link) ---
    odom_node = Node(
        package='odom',
        executable='odom_node',
        name='odom_node',
        output='screen',
    )

    # --- inverse kinematic ---
    inverse_kinematic = Node(
        package='kinematics',
        executable='inverse_kinematic',
        name='inverse_kinematic',
        output='screen',
    )

    # --- serial driver ---
    serial_driver = Node(
        package='kinematics',
        executable='serial_driver',
        name='serial_driver',
        output='screen',
    )

    # --- RPLIDAR driver ---
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rplidar_ros'),
                'launch',
                'rplidar_c1_launch.py'
            ])
        ),
        launch_arguments={'serial_port': '/dev/rplidar'}.items()
    )

    # --- SLAM Toolbox (mapping) với base_frame = base_link ---
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'scan_topic': '/scan',
            'mode': 'mapping',
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
        }],
    )

    return LaunchDescription([
        rsp,
        odom_node,
        inverse_kinematic,
        serial_driver,
        lidar_launch,
        slam_toolbox,
    ])
