#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Flush logs immediately
    SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    # Launch arguments
    map_yaml_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.expanduser('~/my_map.yaml'),
        description='Full path to map YAML file'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation time'
    )
    autostart_arg = DeclareLaunchArgument(
        'autostart', default_value='true', description='Autostart Nav2 lifecycle'
    )

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('slam_launch'),
            'params', 'nav2_params.yaml'
        ]),
        description='Path to custom Nav2 parameters file'
    )

    # LaunchConfiguration substitutions
    map_yaml = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    # Robot description (URDF xacro)
    pkg_desc = FindPackageShare('omnibot_description')
    xacro_file = PathJoinSubstitution([pkg_desc, 'urdf', 'omnibot.urdf.xacro'])
    robot_desc_cmd = Command([TextSubstitution(text='xacro '), xacro_file])
    robot_description = ParameterValue(robot_desc_cmd, value_type=str)
    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher',
        output='screen', parameters=[{'robot_description': robot_description}]
    )

    # Odometry and motor driver nodes
    odom_node = Node(
        package='odom', executable='odom_node', name='odom_node', output='screen'
    )
    inv_kin_node = Node(
        package='kinematics', executable='inverse_kinematic', name='inverse_kinematic', output='screen'
    )
    serial_drv_node = Node(
        package='kinematics', executable='serial_driver', name='serial_driver', output='screen'
    )

    # RPLIDAR driver launch include
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rplidar_ros'), 'launch', 'rplidar_c1_launch.py'
            ])
        ),
        launch_arguments={'serial_port': '/dev/rplidar'}.items()
    )

    # Include Nav2 bringup (map_server, amcl, planner, controller)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py'
            ])
        ),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': LaunchConfiguration('params_file'),
        }.items()
    )


    return LaunchDescription([
        # launch args
        map_yaml_arg,
        use_sim_time_arg,
        autostart_arg,
        params_arg,
        # robot bringup
        rsp,
        odom_node,
        inv_kin_node,
        serial_drv_node,
        lidar_launch,
        # navigation stack
        nav2_bringup,
    ])
