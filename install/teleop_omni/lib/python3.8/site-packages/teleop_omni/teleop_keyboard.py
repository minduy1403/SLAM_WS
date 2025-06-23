#!/usr/bin/env python3
import sys
import termios
import tty
import select
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Map phím → (vx, vy, wz)
KEY_BINDINGS = {
    'w': ( 1.0,  0.0,  0.0),
    's': (-1.0,  0.0,  0.0),
    'a': ( 0.0,  1.0,  0.0),
    'd': ( 0.0, -1.0,  0.0),
    'q': ( 0.0,  0.0,  1.0),
    'e': ( 0.0,  0.0, -1.0),
}

LIN_SCALE    = 0.5    # m/s
ANG_SCALE    = 3.0    # rad/s
LOOP_DT      = 0.01   # 100 Hz
TIMEOUT      = 0.2    # 1 s để bao phủ initial repeat delay

class TeleopOmni(Node):
    def __init__(self):
        super().__init__('teleop_omni')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        fd = sys.stdin.fileno()
        # lưu config terminal gốc
        self.orig_settings = termios.tcgetattr(fd)
        # chuyển sang raw + tắt echo
        tty.setraw(fd)
        no_echo = termios.tcgetattr(fd)
        no_echo[3] &= ~termios.ECHO
        self.raw_settings = no_echo

        self.last_twist = Twist()
        self.last_time = 0.0

        self.get_logger().info(
            "Teleop Omni: giữ W/A/S/D để di chuyển, Q/E để quay. Thả phím → dừng sau 1 s. Ctrl-C để thoát."
        )

    def restore_terminal(self):
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, self.orig_settings)

    def run(self):
        # kích hoạt raw + no echo
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, self.raw_settings)
        try:
            while rclpy.ok():
                now = time.monotonic()
                # non-blocking read
                rlist, _, _ = select.select([sys.stdin], [], [], LOOP_DT)
                if rlist:
                    key = sys.stdin.read(1).lower()
                    if key == '\x03':  # Ctrl-C
                        break
                    if key in KEY_BINDINGS:
                        vx, vy, wz = KEY_BINDINGS[key]
                        msg = Twist()
                        msg.linear.x  = vx * LIN_SCALE
                        msg.linear.y  = vy * LIN_SCALE
                        msg.angular.z = wz * ANG_SCALE
                        self.last_twist = msg
                        self.last_time = now

                # nếu quá TIMEOUT không có event → dừng
                to_pub = self.last_twist if (now - self.last_time) < TIMEOUT else Twist()
                self.pub.publish(to_pub)

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

        finally:
            # publish stop và restore terminal
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
