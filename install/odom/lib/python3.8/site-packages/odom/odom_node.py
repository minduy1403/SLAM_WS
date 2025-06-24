#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from serial.tools import list_ports
import serial
import math


def find_ftdi_port(vid=0x0403, pid=0x6001):
    for port in list_ports.comports():
        if port.vid == vid and port.pid == pid:
            return port.device
    return None

# wheel radius and arm length must match real robot
class OdomNode(Node):
    def __init__(self):
        super().__init__('odom_node')
        # Tham số
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('wheel_radius', 0.04)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.baud_rate = self.get_parameter('baud_rate').value
        self.r = self.get_parameter('wheel_radius').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # Mở serial
        port = find_ftdi_port()
        if not port:
            self.get_logger().error('Không tìm thấy FTDI device 0403:6001!')
            rclpy.shutdown()
            return
        try:
            self.ser = serial.Serial(port, self.baud_rate, timeout=1)
        except serial.SerialException as e:
            self.get_logger().error(f'Không mở được serial {port}: {e}')
            rclpy.shutdown()
            return
        self.get_logger().info(f'Kết nối serial: {port}')

        # Publisher + TF
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Trạng thái odom
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()

        # Timer
        self.timer = self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        # 1) Đọc raw
        raw = self.ser.readline()
        line = raw.decode('ascii', errors='ignore').strip().replace('\x00','')
        if not line:
            return

        # 2) Parse RPM
        parts = line.split()
        if len(parts) != 3:
            return
        try:
            rpmL, rpmR, rpmM = map(float, parts)
        except ValueError:
            return

        # 3) RPM -> rad/s
        wL = rpmL * 2 * math.pi / 60.0
        wR = rpmR * 2 * math.pi / 60.0
        wM = rpmM * 2 * math.pi / 60.0

        vL = wL * self.r
        vR = wR * self.r
        vM = wM * self.r

        # 4) Inverse kinematics → twist
        # Chuyển 30 độ sang radian
        angle = math.radians(30)
        cos30 = math.cos(angle)
        sin30 = math.sin(angle)

        vx = (-vL * cos30 + vR * cos30)
        vy = (vR * sin30 + vL * sin30 - vM)
        omega = (wL + wR + wM) * (self.r / (3.0 * 0.1))

        # 5) Tích phân pose
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        dx = (vx * math.cos(self.th) - vy * math.sin(self.th)) * dt
        dy = (vx * math.sin(self.th) + vy * math.cos(self.th)) * dt
        dth = omega * dt
        self.x += dx
        self.y += dy
        self.th += dth

        # 6) Publish Odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.th / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.th / 2.0)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = omega
        self.odom_pub.publish(odom)

        # 7) Broadcast TF
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = math.sin(self.th / 2.0)
        t.transform.rotation.w = math.cos(self.th / 2.0)
        self.tf_broadcaster.sendTransform(t)

        # 8) DEBUG: log gọn gàng bằng ROS logger
        self.get_logger().info(
            f"x={self.x:.3f}, y={self.y:.3f}, vx={vx:.3f}, ω={omega:.3f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()
    if rclpy.ok() and hasattr(node, 'ser'):
        rclpy.spin(node)
    if hasattr(node, 'ser'):
        node.ser.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
