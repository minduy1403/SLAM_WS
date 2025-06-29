#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.substitutions import TextSubstitution, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Package directories
    bringup_dir = get_package_share_directory('slam_launch')
    desc_pkg_dir = get_package_share_directory('omnibot_description')
    bt_dir       = get_package_share_directory('nav2_bt_navigator')

    # Hard-coded defaults
    namespace                     = TextSubstitution(text='')
    map_yaml                      = TextSubstitution(text='/home/jetson/my_map.yaml')
    use_sim_time                  = TextSubstitution(text='false')
    autostart                     = TextSubstitution(text='true')
    default_bt_xml                = PathJoinSubstitution([
                                        bt_dir,
                                        'behavior_trees',
                                        'navigate_w_replanning_and_recovery.xml'
                                      ])
    map_subscribe_transient_local = TextSubstitution(text='false')
    nav2_params_path              = '/home/jetson/SLAM_WS/src/slam_launch/params/nav2_params.yaml'

    # Prepare rewritten params for Nav2
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'default_bt_xml_filename': default_bt_xml,
        'autostart': autostart,
        'map_subscribe_transient_local': map_subscribe_transient_local
    }
    configured_params = RewrittenYaml(
        source_file=nav2_params_path,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True
    )

    # Robot state publisher (URDF via xacro)
    xacro_file = os.path.join(desc_pkg_dir, 'urdf', 'omnibot.urdf.xacro')
    # Note: include the space after 'xacro' so the command becomes "xacro /path/to/omnibot.urdf.xacro"
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Odometry & kinematics
    odom_node = Node(
        package='odom',
        executable='odom_node',
        name='odom_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    inv_kin_node = Node(
        package='kinematics',
        executable='inverse_kinematic',
        name='inverse_kinematic',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    serial_drv_node = Node(
        package='kinematics',
        executable='serial_driver',
        name='serial_driver',
        output='screen'
    )

    # RPLIDAR driver
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

    # Nav2 bringup (all-in-one localization + navigation)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            ])
        ),
        launch_arguments={
            'namespace':                     namespace,
            'map':                           map_yaml,
            'use_sim_time':                  use_sim_time,
            'autostart':                     autostart,
            'params_file':                   nav2_params_path,
            'default_bt_xml_filename':       default_bt_xml,
            'map_subscribe_transient_local': map_subscribe_transient_local
        }.items()
    )

    return LaunchDescription([
        # immediate logging
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        # core nodes
        rsp_node,
        odom_node,
        inv_kin_node,
        serial_drv_node,

        # lidar
        lidar_launch,

        # nav2 stack
        nav2_launch,
    ])
