import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial


class CmdVelSerialBridge(Node):

    def __init__(self):
        super().__init__('cmdvel_serial_bridge')

        self.ser = serial.Serial('/dev/ttyACM0', 57600, timeout=1)
        self.get_logger().info("Connected")

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10)

        self.left = 0
        self.right = 0

        self.timer = self.create_timer(0.1, self.send_command)

    def cmd_callback(self, msg):
        self.left = int(50 * msg.linear.x - 50 * msg.angular.z)
        self.right = int(50 * msg.linear.x + 50 * msg.angular.z)

    def send_command(self):
        command = f"D {self.left} {self.right} 1\n"
        self.ser.write(command.encode())

    def destroy_node(self):
        if self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSerialBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()