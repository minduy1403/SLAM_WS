import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

class InverseKinematicNode(Node):
    def __init__(self):
        super().__init__('inverse_kinematic')
        # Subscribe to cmd_vel topic
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        # Publisher for wheel speeds in RPM
        self.pub = self.create_publisher(Float32MultiArray, 'wheel_speeds', 10)

        # Robot geometry
        self.r = 0.04   # wheel radius (m)
        self.L = 0.15   # distance from center to wheel (m)
        # Wheel angles measured CCW from +X: front=0°, right=120°, left=240°
        self.theta_front = 0.0
        self.theta_right = 2 * math.pi / 3
        self.theta_left = 4 * math.pi / 3

        self.get_logger().info('Inverse Kinematics ready.')

    def cmd_vel_callback(self, twist: Twist):
        # Extract chassis velocities
        vx = twist.linear.x    # forward/back
        vy = -twist.linear.y    # left/right
        wz = -twist.angular.z   # rotation

        # Zero front wheel for testing
        omega_front = (-math.sin(self.theta_front) * vx + math.cos(self.theta_front) * vy + self.L * wz) / self.r
        # Compute right and left wheel omegas
        omega_right = (-math.sin(self.theta_right) * vx + math.cos(self.theta_right) * vy + self.L * wz) / self.r
        omega_left  = (-math.sin(self.theta_left)  * vx + math.cos(self.theta_left)  * vy + self.L * wz) / self.r

        # Convert to RPM
        rpm_front = omega_front  * 60.0 / (2 * math.pi)
        rpm_right = omega_right  * 60.0 / (2 * math.pi)
        rpm_left  = omega_left   * 60.0 / (2 * math.pi)

        # self.get_logger().info(
        #     f"Debug RPMs -> Front: {rpm_front:.2f}, Right: {rpm_right:.2f}, Left: {rpm_left:.2f}"
        # )

        # Publish only front zero, others
        msg = Float32MultiArray()
        msg.data = [rpm_left, rpm_right, rpm_front]
        self.pub.publish(msg)

        # Final log
        # self.get_logger().info(
        #     f"Published wheel_speeds: [{rpm_left:.1f}, {rpm_right:.1f}, {rpm_front:.1f}]"
        # )


def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematicNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
