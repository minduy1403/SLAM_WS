import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

class InverseKinematicNode(Node):
    def __init__(self):
        super().__init__('inverse_kinematic')
        # Subscribe to cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.subscription  # avoid unused warning

        # Publisher wheel speeds
        self.pub = self.create_publisher(
            Float32MultiArray,
            'wheel_speeds',
            10
        )

        # Robot parameters
        self.r = 0.04   # wheel radius (m)
        self.L = 0.15   # robot radius (m)
        # Wheel orientation angles (rad)
        self.theta = [0, 2*math.pi/3, 4*math.pi/3]

        self.get_logger().info('Inverse Kinematic node started, waiting for /cmd_vel messages...')

    def cmd_vel_callback(self, msg: Twist):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        self.get_logger().debug(
            f'Received cmd_vel: vx={vx:.2f}, vy={vy:.2f}, wz={wz:.2f}'
        )

        # Compute wheel angular velocities (rad/s)
        omegas = [
            (-math.sin(th) * vx + math.cos(th) * vy + self.L * wz) / self.r
            for th in self.theta
        ]

        # Convert to RPM
        rpms = [omega * 60.0 / (2 * math.pi) for omega in omegas]

        # Publish wheel speeds
        msg_out = Float32MultiArray()
        msg_out.data = rpms
        self.pub.publish(msg_out)

        # Log RPM values
        self.get_logger().info(
            f"RPMs -> w1: {rpms[0]:.2f}, w2: {rpms[1]:.2f}, w3: {rpms[2]:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
