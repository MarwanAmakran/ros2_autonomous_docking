import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial


class CmdVelSerialBridge(Node):

    def __init__(self):
        super().__init__('cmdvel_serial_bridge')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 57600)
        self.declare_parameter('cmd_timeout_sec', 0.5)
        self.declare_parameter('max_pwm', 80)
        self.declare_parameter('linear_gain', 50.0)
        self.declare_parameter('angular_gain', 50.0)

        port = str(self.get_parameter('port').value)
        baud = int(self.get_parameter('baud').value)
        self.cmd_timeout_sec = float(self.get_parameter('cmd_timeout_sec').value)
        self.max_pwm = int(self.get_parameter('max_pwm').value)
        self.linear_gain = float(self.get_parameter('linear_gain').value)
        self.angular_gain = float(self.get_parameter('angular_gain').value)

        self.ser = serial.Serial(port, baud, timeout=1)
        self.get_logger().info(f"Connected to {port} @ {baud}")

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10)

        self.left = 0
        self.right = 0
        self.last_cmd_time = self.get_clock().now()

        # Send continuously so the motor controller receives fresh commands
        self.timer = self.create_timer(0.1, self.send_command)

    def cmd_callback(self, msg):
        linear = float(msg.linear.x)
        angular = float(msg.angular.z)

        left = int(self.linear_gain * linear - self.angular_gain * angular)
        right = int(self.linear_gain * linear + self.angular_gain * angular)

        self.left = max(min(left, self.max_pwm), -self.max_pwm)
        self.right = max(min(right, self.max_pwm), -self.max_pwm)
        self.last_cmd_time = self.get_clock().now()

    def send_command(self):
        dt = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if dt > self.cmd_timeout_sec:
            self.left = 0
            self.right = 0

        command = f"D {self.left} {self.right} 1\n"
        self.ser.write(command.encode())
        self.get_logger().info(f"Sent: {command.strip()}")

    def destroy_node(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSerialBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()