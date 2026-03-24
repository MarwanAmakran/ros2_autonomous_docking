import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial


class CmdVelSerialBridge(Node):

    def __init__(self):
        super().__init__('cmdvel_serial_bridge')

        self.ser = serial.Serial('/dev/ttyACM0', 57600, timeout=1)
        self.get_logger().info("Connected to /dev/ttyACM0")

        self.subscription = self.create_subscription(
            Twist,
            '/detector',
            self.cmd_callback,
            10)

        self.left = 0
        self.right = 0

        # Timer → stuur elke 0.3 sec
        self.timer = self.create_timer(1, self.send_command)

    def cmd_callback(self, msg):
        x = msg.x
        y = msg.y

        self.left = 10
        self.right = -10

        command = f"D {self.left} {self.right} 1\n"
        self.ser.write(command.encode())
        self.get_logger().info(f"Sent: {command.strip()}")


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSerialBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
