import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import threading

class SerialDriverNode(Node):
    def __init__(self):
        super().__init__('serial_driver')
        # Serial port placeholder (set actual port before running)
        self.serial_port = '/dev/ttyUSB0'
        self.baudrate = 9600
        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=1)
            self.get_logger().info(f"Opened serial port {self.serial_port} @ {self.baudrate} bps")
        except serial.SerialException as e:
            self.get_logger().error(f"Cannot open serial port: {e}")
            self.ser = None

        # Subscribe to wheel_speeds topic
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'wheel_speeds',
            self.wheel_speeds_callback,
            10
        )
        self.subscription  # avoid unused warning
        self.lock = threading.Lock()

    def wheel_speeds_callback(self, msg: Float32MultiArray):
        # msg.data assumed [left, right, front] in RPM
        if not self.ser or not self.ser.is_open:
            self.get_logger().warn("Serial port not open, cannot send data.")
            return

        rpms = msg.data
        # Send in the same order: left, right, front
        line = f"{int(rpms[0])} {int(rpms[1])} {int(rpms[2])}\n"
        with self.lock:
            try:
                self.ser.write(line.encode('utf-8'))
                self.get_logger().debug(f"Sent over serial: {line.strip()}")
            except serial.SerialException as e:
                self.get_logger().error(f"Error writing to serial port: {e}")

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Serial port closed.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
