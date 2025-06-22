import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Inverse Kinematic Node
        Node(
            package='kinematics',
            executable='inverse_kinematic',
            name='inverse_kinematic',
            output='screen',
        ),
        # Serial Driver Node
        Node(
            package='kinematics',
            executable='serial_driver',
            name='serial_driver',
            output='screen',
        ),

        # Odom Node
        Node(
            package='odom',
            executable='odom_node',
            name='odom_node',
            output='log',
            parameters=[{
                'baud_rate': 9600,
                'wheel_radius': 0.05,
                'odom_frame': 'odom',
                'base_frame': 'base_link',
            }],
        )
    ])
