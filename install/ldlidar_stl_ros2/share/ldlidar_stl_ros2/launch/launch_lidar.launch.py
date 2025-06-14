from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('product_name', default_value='LDLiDAR_L1'),
        DeclareLaunchArgument('port_name', default_value='/dev/ttyUSB0'),       # <<<< QUAN TRỌNG
        DeclareLaunchArgument('baud_rate', default_value='460800'),             # <<<< QUAN TRỌNG
        DeclareLaunchArgument('frame_id', default_value='laser_frame'),
        DeclareLaunchArgument('topic_name', default_value='scan'),

        Node(
            package='ldlidar_stl_ros2',
            executable='ldlidar_stl_ros2_node',
            name='ldlidar_node',
            parameters=[{
                'product_name': LaunchConfiguration('product_name'),
                'serial_port': LaunchConfiguration('port_name'),
                'serial_baudrate': LaunchConfiguration('baud_rate'),
                'frame_id': LaunchConfiguration('frame_id'),
                'topic_name': LaunchConfiguration('topic_name'),
                'laser_scan_dir': True,
                'enable_angle_crop_func': False
            }],
            output='screen'
        )
    ])
