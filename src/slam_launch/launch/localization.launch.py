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
        'map_yaml',
        default_value=os.path.expanduser('~/my_map.yaml'),
        description='Path to map YAML file'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation time'
    )
    autostart_arg = DeclareLaunchArgument(
        'autostart', default_value='true', description='Autostart lifecycle'
    )

    # Substitutions
    map_yaml = LaunchConfiguration('map_yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    # Robot description
    pkg_desc = FindPackageShare('omnibot_description')
    xacro_file = PathJoinSubstitution([pkg_desc, 'urdf', 'omnibot.urdf.xacro'])
    robot_desc = Command([TextSubstitution(text='xacro '), xacro_file])
    robot_description = ParameterValue(robot_desc, value_type=str)
    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher',
        output='screen', parameters=[{'robot_description': robot_description}]
    )

    # Odometry and hardware interface
    odom_node = Node(
        package='odom', executable='odom_node', name='odom_node', output='screen'
    )
    inv_kin = Node(
        package='kinematics', executable='inverse_kinematic', name='inverse_kinematic', output='screen'
    )
    serial_drv = Node(
        package='kinematics', executable='serial_driver', name='serial_driver', output='screen'
    )

    # RPLIDAR driver include
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rplidar_ros'), 'launch', 'rplidar_c1_launch.py'
            ])
        ),
        launch_arguments={'serial_port': '/dev/rplidar'}.items()
    )

    # Static map server
    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server', output='screen',
        parameters=[{'yaml_filename': map_yaml}]
    )

    # AMCL for localization (override default base_frame_id)
    amcl = Node(
        package='nav2_amcl', executable='amcl', name='amcl', output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'yaml_filename': map_yaml},
            {'odom_frame_id': 'odom'},
            {'base_frame_id': 'base_link'}
        ],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )

    # Lifecycle manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_localization', output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': ['map_server', 'amcl']}
        ]
    )

    return LaunchDescription([
        map_yaml_arg,
        use_sim_time_arg,
        autostart_arg,
        rsp,
        odom_node,
        inv_kin,
        serial_drv,
        lidar_launch,
        map_server,
        amcl,
        lifecycle_manager,
    ])
