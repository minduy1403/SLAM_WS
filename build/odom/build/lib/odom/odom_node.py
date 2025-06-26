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

class OdomNode(Node):
    def __init__(self):
        super().__init__('odom_node')
        # Tham số
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('wheel_radius', 0.04)  # bán kính bánh (m)
        self.declare_parameter('scale_factor', 1.0)     # hệ số hiệu chỉnh chung
        self.declare_parameter('scale_x', 0.6)         # hệ số hiệu chỉnh cho x
        self.declare_parameter('scale_y', 0.7)         # hệ số hiệu chỉnh cho y
        self.declare_parameter('scale_theta', 1.0)     # hệ số hiệu chỉnh cho theta
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.baud_rate    = self.get_parameter('baud_rate').value
        base_r           = self.get_parameter('wheel_radius').value
        common_scale     = self.get_parameter('scale_factor').value
        self.scale_x     = self.get_parameter('scale_x').value * common_scale
        self.scale_y     = self.get_parameter('scale_y').value * common_scale
        self.scale_theta = self.get_parameter('scale_theta').value * common_scale
        self.r           = base_r * common_scale
        self.odom_frame  = self.get_parameter('odom_frame').value
        self.base_frame  = self.get_parameter('base_frame').value

        # Mở serial
        port = find_ftdi_port()
        if not port:
            print('ERROR: Không tìm thấy FTDI device 0403:6001!', flush=True)
            rclpy.shutdown()
            return
        try:
            self.ser = serial.Serial(port, self.baud_rate, timeout=1)
        except serial.SerialException as e:
            print(f'ERROR: Không mở được serial {port}: {e}', flush=True)
            rclpy.shutdown()
            return
        print(f'Kết nối serial: {port}', flush=True)

        # Publisher + TF
        self.odom_pub       = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Trạng thái odom
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()

        # Timer 20 Hz
        self.timer = self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        # 1) Đọc raw
        raw = self.ser.readline()
        line = raw.decode('ascii', errors='ignore').strip().replace('\x00', '')
        if not line:
            return

        # 2) Parse RPM từ serial
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
        angle = math.radians(30)
        cos30 = math.cos(angle)
        sin30 = math.sin(angle)

        vx    = vL * cos30 - vR * cos30
        vy    = vR * sin30 + vL * sin30 - vM
        omega = -(wL + wR + wM) * (self.r / (3.0 * 0.15))

        # 5) Tích phân pose
        now = self.get_clock().now()
        dt  = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        dx = (vx * math.cos(self.th) - vy * math.sin(self.th)) * dt
        dy = (vx * math.sin(self.th) + vy * math.cos(self.th)) * dt
        dth = omega * dt

        # Áp dụng scale cho từng thành phần
        dx *= self.scale_x
        dy *= self.scale_y
        dth *= self.scale_theta

        self.x  += dx
        self.y  += dy
        self.th += dth

        # Chuyển theta sang độ
        theta_deg = self.th * 180.0 / math.pi
        # In các giá trị lên một dòng, dùng flush và carriage return
        print(
            f"rpmL={rpmL:.1f}, rpmR={rpmR:.1f}, rpmM={rpmM:.1f} | "
            f"vx={vx:.3f}, vy={vy:.3f}, omega={omega:.3f} | "
            f"x={self.x:.3f}, y={self.y:.3f}, theta={theta_deg:.1f}°",
            end='\r', flush=True
        )

        # 6) Publish Odometry
        odom = Odometry()
        odom.header.stamp    = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id  = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.th / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.th / 2.0)
        odom.twist.twist.linear.x  = vx
        odom.twist.twist.linear.y  = vy
        odom.twist.twist.angular.z = omega
        self.odom_pub.publish(odom)

        # 7) Broadcast TF
        t = TransformStamped()
        t.header.stamp    = now.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id  = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z    = math.sin(self.th / 2.0)
        t.transform.rotation.w    = math.cos(self.th / 2.0)
        self.tf_broadcaster.sendTransform(t)


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
