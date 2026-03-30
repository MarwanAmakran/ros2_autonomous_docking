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
        self.declare_parameter('send_period_sec', 0.2)
        self.declare_parameter('heartbeat_sec', 0.6)
        self.declare_parameter('max_pwm', 80)
        self.declare_parameter('linear_gain', 50.0)
        self.declare_parameter('angular_gain', 50.0)

        port = str(self.get_parameter('port').value)
        baud = int(self.get_parameter('baud').value)
        self.cmd_timeout_sec = float(self.get_parameter('cmd_timeout_sec').value)
        self.send_period_sec = float(self.get_parameter('send_period_sec').value)
        self.heartbeat_sec = float(self.get_parameter('heartbeat_sec').value)
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
        self.last_sent_left = None
        self.last_sent_right = None
        self.last_send_time = self.get_clock().now()

        # Send continuously so the motor controller receives fresh commands
        self.timer = self.create_timer(self.send_period_sec, self.send_command)

    def cmd_callback(self, msg):
        linear = float(msg.linear.x)
        angular = float(msg.angular.z)

        left = int(self.linear_gain * linear - self.angular_gain * angular)
        right = int(self.linear_gain * linear + self.angular_gain * angular)

        self.left = max(min(left, self.max_pwm), -self.max_pwm)
        self.right = max(min(right, self.max_pwm), -self.max_pwm)
        self.last_cmd_time = self.get_clock().now()

    def send_command(self):
        now = self.get_clock().now()
        dt = (now - self.last_cmd_time).nanoseconds / 1e9
        if dt > self.cmd_timeout_sec:
            self.left = 0
            self.right = 0

        send_due_to_change = (
            self.left != self.last_sent_left or
            self.right != self.last_sent_right
        )
        send_due_to_heartbeat = (
            (now - self.last_send_time).nanoseconds / 1e9 >= self.heartbeat_sec
        )

        if not send_due_to_change and not send_due_to_heartbeat:
            return

        command = f"D {self.left} {self.right} 1\n"
        self.ser.write(command.encode())
        self.last_sent_left = self.left
        self.last_sent_right = self.right
        self.last_send_time = now

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