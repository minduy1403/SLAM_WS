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
    # Chuẩn bị robot_description từ URDF xacro
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

    # robot_state_publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_param}],
    )

    # odom_node
    odom_node = Node(
        package='odom',
        executable='odom_node',
        name='odom_node',
        output='screen',
    )

    # RPLIDAR driver
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

    # static TF base_link -> laser
    static_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_broadcaster',
        output='screen',
        arguments=[
            '0.1', '0.0', '0.2',  # xyz
            '0', '0', '0',        # rpy
            'base_link', 'laser'
        ],
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
    
    # map_server (publish saved map)
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': os.path.expanduser('~/my_map.yaml')
        }]
    )

    # SLAM Toolbox localization
    slam_loc = TimerAction(
        period=2.0,
        actions=[Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='localization_slam_toolbox_node',
            output='screen',
            parameters=[{
                'mode': 'localization',
                'map_file': os.path.expanduser('~/my_map.yaml'),
                'use_sim_time': False,
                'scan_topic': '/scan',
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'use_tf_static': True,
            }],
        )]
    )

    return LaunchDescription([
        rsp,
        odom_node,
        lidar_launch,
        static_laser_tf,
        map_server,
        slam_loc,
    ])
