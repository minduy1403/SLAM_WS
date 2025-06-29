#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Directories
    bringup_dir = get_package_share_directory('slam_launch')
    desc_pkg_dir = get_package_share_directory('omnibot_description')

    # Launch configuration
    namespace = LaunchConfiguration('namespace')
    map_yaml = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml = LaunchConfiguration('default_bt_xml_filename')
    map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local')

    # Lifecycle nodes to autostart
    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'recoveries_server',
        'bt_navigator'
    ]

    # Remappings for TF
    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static')
    ]

    # Rewritten YAML for parameter substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'default_bt_xml_filename': default_bt_xml,
        'autostart': autostart,
        'map_subscribe_transient_local': map_subscribe_transient_local
    }
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True
    )

    # ===== Declare launch arguments =====
    declare_args = [
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Top-level namespace for all nodes'
        ),
        DeclareLaunchArgument(
            'map', default_value=os.path.expanduser('~/my_map.yaml'),
            description='Full path to map YAML file'
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock'
        ),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup Nav2 lifecycle nodes'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([bringup_dir, 'params', 'nav2_params.yaml']),
            description='Path to Nav2 parameters file'
        ),
        DeclareLaunchArgument(
            'default_bt_xml_filename',
            default_value=PathJoinSubstitution([
                get_package_share_directory('nav2_bt_navigator'),
                'behavior_trees',
                'navigate_w_replanning_and_recovery.xml'
            ]),
            description='Full path to behavior tree XML file'
        ),
        DeclareLaunchArgument(
            'map_subscribe_transient_local', default_value='false',
            description='Whether to subscribe to map with transient local QoS'
        )
    ]

    # ===== Robot state publisher (URDF) =====
    xacro_file = os.path.join(desc_pkg_dir, 'urdf', 'omnibot.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )
    rsp_node = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # ===== Odometry & Kinematics =====
    odom_node = Node(
        package='odom', executable='odom_node', name='odom_node', output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    inv_kin_node = Node(
        package='kinematics', executable='inverse_kinematic',
        name='inverse_kinematic', output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    serial_drv_node = Node(
        package='kinematics', executable='serial_driver',
        name='serial_driver', output='screen'
    )

    # ===== RPLIDAR Driver =====
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('rplidar_ros'),
                'launch',
                'rplidar_c1_launch.py'
            ])
        ),
        launch_arguments={
            'serial_port': '/dev/rplidar',
            'frame_id': 'laser'
        }.items()
    )

    # ===== Nav2 Bringup =====
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            ])
        ),
        launch_arguments={
            'namespace': namespace,
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
            'default_bt_xml_filename': default_bt_xml,
            'map_subscribe_transient_local': map_subscribe_transient_local
        }.items()
    )

    # ===== Assemble =====
    return LaunchDescription([
        # Immediate log flush
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        # Launch arguments
        *declare_args,
        # Core nodes
        rsp_node,
        odom_node,
        inv_kin_node,
        serial_drv_node,
        lidar_launch,
        # Nav2 stack
        nav2_launch
    ])
