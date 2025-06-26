#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, TextSubstitution
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # 1) robot_description từ URDF xacro
    pkg_desc = FindPackageShare('omnibot_description')
    xacro_file = PathJoinSubstitution([
        pkg_desc, 'urdf', 'omnibot.urdf.xacro'
    ])
    robot_desc = Command([
        TextSubstitution(text='xacro '), xacro_file
    ])
    robot_description_param = ParameterValue(robot_desc, value_type=str)

    # 2) robot_state_publisher → /tf_static
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_param}],
    )

    # 3) odom_node (odom→base_link)
    odom_node = Node(
        package='odom',
        executable='odom_node',
        name='odom_node',
        output='screen',
    )

    # 4) inverse kinematic
    inverse_kinematic = Node(
        package='kinematics',
        executable='inverse_kinematic',
        name='inverse_kinematic',
        output='screen',
    )

    # 5) serial_driver
    serial_driver = Node(
        package='kinematics',
        executable='serial_driver',
        name='serial_driver',
        output='screen',
    )

    # 6) RPLIDAR driver include
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rplidar_ros'),
                'launch', 'rplidar_c1_launch.py'
            ])
        ),
        launch_arguments={'serial_port': '/dev/rplidar'}.items()
    )

    # 7) SLAM Toolbox với tuning + use_tf_static
    slam_toolbox = TimerAction(
        period=2.0,
        actions=[Node(
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
                'use_tf_static': True,
                'resolution': 0.10,
                'max_laser_range': 7.0,
                'min_laser_range': 0.15,
                'minimum_travel_distance': 0.2,
                'minimum_travel_heading': 0.1,
            }],
        )]
    )

    return LaunchDescription([
        rsp,
        odom_node,
        inverse_kinematic,
        serial_driver,
        lidar_launch,
        slam_toolbox,
    ])
