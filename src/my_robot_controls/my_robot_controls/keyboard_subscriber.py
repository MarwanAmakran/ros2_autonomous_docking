import sys
import threading
import time
import termios
import tty
import serial
import select
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class KeyboardSubscriber(Node):
    def __init__(self):
        super().__init__('keyboard_subscriber')
        self.subscription = self.create_subscription(
            String,
            'cmd_key',
            self.listener_callback,
            10
        )

        self.declare_parameter('port', '/dev/ttyACM0')  # default Arduino port
        self.declare_parameter('baud', 57600)           # default baud rate
        self.declare_parameter('timeout', 0.1)          # serial read timeout in seconds

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        timeout = self.get_parameter('timeout').get_parameter_value().double_value

        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=timeout)
            time.sleep(2.0)
            self.get_logger().info(f"Connected to serial port {port} at {baud} baud")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port {port}: {e}")
            raise

        self.get_logger().info("Keyboard subscriber started and ready to send commands to MCU")

    def listener_callback(self, msg: String):
        """
        Called whenever a message is received on /cmd_key.
        Sends the received string over serial to the robot.
        """
        command = msg.data
        try:
            self.ser.write((command + "\n").encode())
            self.get_logger().info(f"Sent to robot: {command}")
        except Exception as e:
            self.get_logger().error(f"Failed to send command over serial: {e}")

    def destroy_node(self):
        """Close serial connection and destroy node properly."""
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
                self.get_logger().info("Serial connection closed")
        except Exception:
            pass
        super().destroy_node()


def main():
    """Main entry point for the serial subscriber node."""
    rclpy.init()
    node = KeyboardSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
