#!/usr/bin/env python3
import sys
import termios
import tty
import select
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

KEY_BINDINGS = {
    'w': ( 1.0,  0.0,  0.0),
    's': (-1.0,  0.0,  0.0),
    'a': ( 0.0,  1.0,  0.0),
    'd': ( 0.0, -1.0,  0.0),
    'q': ( 0.0,  0.0,  1.0),
    'e': ( 0.0,  0.0, -1.0),
}

LIN_SCALE = 0.5
ANG_SCALE = 1.0

class TeleopOmni(Node):
    def __init__(self):
        super().__init__('teleop_omni')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        fd = sys.stdin.fileno()
        # lưu cấu hình gốc
        self.orig_settings = termios.tcgetattr(fd)
        # tạo cấu hình raw (tty.setraw) + no echo
        raw = termios.tcgetattr(fd)
        tty.setraw(fd)  # sets raw and disables echo
        raw[3] = raw[3] & ~termios.ECHO  # đảm bảo echo tắt
        self.raw_settings = raw

        self.current_twist = Twist()
        self.get_logger().info(
            "Teleop Omni: giữ W/A/S/D để di chuyển, Q/E để quay, thả phím để dừng, Ctrl-C để thoát"
        )

    def restore_terminal(self):
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, self.orig_settings)

    def run(self):
        # bật chế độ raw + no echo một lần
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, self.raw_settings)
        try:
            while rclpy.ok():
                # non-blocking check
                rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
                if rlist:
                    key = sys.stdin.read(1)
                else:
                    key = ''

                if key == '\x03':  # Ctrl-C
                    break

                if key in KEY_BINDINGS:
                    vx, vy, wz = KEY_BINDINGS[key]
                    self.current_twist.linear.x  = vx * LIN_SCALE
                    self.current_twist.linear.y  = vy * LIN_SCALE
                    self.current_twist.angular.z = wz * ANG_SCALE
                else:
                    # không giữ phím → dừng
                    self.current_twist.linear.x  = 0.0
                    self.current_twist.linear.y  = 0.0
                    self.current_twist.angular.z = 0.0

                self.pub.publish(self.current_twist)

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

        finally:
            # publish dừng và restore terminal
            self.pub.publish(Twist())
            self.restore_terminal()

def main(args=None):
    rclpy.init(args=args)
    node = TeleopOmni()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
