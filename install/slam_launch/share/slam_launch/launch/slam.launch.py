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
    # 1) Chuẩn bị robot_description từ URDF xacro
    pkg_desc = FindPackageShare('omnibot_description')
    xacro_file = PathJoinSubstitution([
        pkg_desc,
        'urdf',
        'omnibot.urdf.xacro'
    ])
    robot_desc = Command([
        TextSubstitution(text='xacro '),
        xacro_file
    ])
    robot_description_param = ParameterValue(robot_desc, value_type=str)

    # 2) Node robot_state_publisher (publish /tf_static)
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_param}],
    )

    # 3) odom_node (publish odom -> base_link)
    odom_node = Node(
        package='odom',
        executable='odom_node',
        name='odom_node',
        output='screen',
    )

    # 4) inverse_kinematic
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
                'launch',
                'rplidar_c1_launch.py'
            ])
        ),
        launch_arguments={'serial_port': '/dev/rplidar'}.items()
    )

    # 7) Static TF base_link -> laser
    static_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_broadcaster',
        output='screen',
        arguments=[
            '0.1',  # lidar_x
            '0.0',  # lidar_y
            '0.2',  # lidar_z
            '0',    # roll
            '0',    # pitch
            '0',    # yaw
            'base_link',
            'laser',
        ],
    )

    # 8) SLAM Toolbox (mapping) - tune parameters
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
                # map resolution (m)
                'resolution': 0.10,
                # laser matching range
                'max_laser_range': 7.0,
                'min_laser_range': 0.15,
                # update thresholds
                'map_update_distance': 0.2,
                'map_update_angle': 0.1,
                # optional tuning
                'scan_rate': 10,
            }],
        )]
    )

    return LaunchDescription([
        rsp,
        odom_node,
        inverse_kinematic,
        serial_driver,
        lidar_launch,
        static_laser_tf,
        slam_toolbox,
    ])
