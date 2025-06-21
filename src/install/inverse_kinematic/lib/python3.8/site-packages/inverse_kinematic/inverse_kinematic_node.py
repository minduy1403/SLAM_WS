import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class InverseKinematicNode(Node):
    def __init__(self):
        super().__init__('inverse_kinematic')
        self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        # hệ số quy đổi vận tốc -> PWM (tùy chỉnh)
        self.pwm_scale = 100  # ví dụ: 1 m/s -> 100 PWM

    def cmd_vel_callback(self, msg: Twist):
        # đọc giá trị vx, vy, wz
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        # TODO: áp dụng công thức inverse kinematics tính tốc độ bánh (rad/s)
        # v1, v2, v3 là tốc độ góc mỗi bánh
        # ví dụ tạm: giả sử v1 = vx, v2 = vy, v3 = wz
        v1 = vx
        v2 = vy
        v3 = wz

        # quy đổi sang PWM
        pwm1 = int(v1 * self.pwm_scale)
        pwm2 = int(v2 * self.pwm_scale)
        pwm3 = int(v3 * self.pwm_scale)

        # log một hàng duy nhất, cập nhật liên tục
        print(f"\rPWM1: {pwm1:4d} | PWM2: {pwm2:4d} | PWM3: {pwm3:4d}", end='', flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # khi thoát, xuống dòng để tránh đè lên terminal prompt
        print()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
