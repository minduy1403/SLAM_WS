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
        self.subscription  # tránh warning biến không sử dụng

        # Publisher wheel speeds
        self.pub = self.create_publisher(
            Float32MultiArray,
            'wheel_speeds',
            10
        )

        # Thông số robot
        self.r = 0.04   # bán kính bánh xe (m)
        self.L = 0.15   # bán kính robot (m)
        # Góc đặt bánh (rad)
        self.theta = [0, 2*math.pi/3, 4*math.pi/3]

        self.get_logger().info('Inverse Kinematic node started, waiting for /cmd_vel messages...')

    def cmd_vel_callback(self, msg: Twist):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        self.get_logger().debug(f'Received cmd_vel: vx={vx:.2f}, vy={vy:.2f}, wz={wz:.2f}')

        # Tính omega (rad/s) cho mỗi bánh
        omegas = [
            (-math.sin(th) * vx + math.cos(th) * vy + self.L * wz) / self.r
            for th in self.theta
        ]

        # Chuyển sang RPM: RPM = omega (rad/s) * 60 / (2*pi)
        rpms = [omega * 60.0 / (2 * math.pi) for omega in omegas]

        # Chuẩn bị message
        msg_out = Float32MultiArray()
        msg_out.data = rpms
        # Publish wheel speeds (RPM)
        self.pub.publish(msg_out)

        # In lên terminal trên một dòng
        w1, w2, w3 = rpms
        print(f"\rRPM1: {w1:7.2f} | RPM2: {w2:7.2f} | RPM3: {w3:7.2f}", end='', flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print()  # xuống dòng khi thoát
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
