import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import serial.tools.list_ports
import threading

class SerialDriverNode(Node):
    def __init__(self):
        super().__init__('serial_driver')

        # Auto-detect serial port by VID:PID for FT232 (0403:6001)
        serial_port = self._find_serial_port(0x0403, 0x6001)
        self.baudrate = 9600
        if serial_port:
            self.serial_port = serial_port
            try:
                self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=1)
                self.get_logger().info(
                    f"Opened serial port {self.serial_port} @ {self.baudrate} bps"
                )
            except serial.SerialException as e:
                self.get_logger().error(f"Cannot open serial port: {e}")
                self.ser = None
        else:
            self.get_logger().error(
                "FT232 device not found (VID:PID=0403:6001)."
            )
            self.ser = None

        # Subscribe to wheel_speeds topic
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'wheel_speeds',
            self.wheel_speeds_callback,
            10
        )
        self.subscription  # avoid unused warning

        # Thread lock for serial writes
        self.lock = threading.Lock()

    def _find_serial_port(self, vid, pid):
        """Finds the serial port by vendor ID and product ID."""
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if (port.vid, port.pid) == (vid, pid):
                return port.device
        return None

    def wheel_speeds_callback(self, msg: Float32MultiArray):
        # msg.data contains [rpm0, rpm120, rpm240]
        if not getattr(self, 'ser', None) or not self.ser.is_open:
            self.get_logger().warn("Serial port not open, cannot send data.")
            return

        rpms = msg.data
        # Reorder to send: rpm at 240°, rpm at 120°, rpm at 0°
        ordered = [rpms[2], rpms[1], rpms[0]]

        # Format: "240rpm 120rpm 0rpm\n"
        line = f"{int(ordered[0])} {int(ordered[1])} {int(ordered[2])}\n"
        with self.lock:
            try:
                self.ser.write(line.encode('utf-8'))
                self.get_logger().debug(f"Sent over serial: {line.strip()}")
            except serial.SerialException as e:
                self.get_logger().error(f"Error writing to serial port: {e}")

    def destroy_node(self):
        # Close serial port cleanly
        if getattr(self, 'ser', None) and self.ser.is_open:
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
