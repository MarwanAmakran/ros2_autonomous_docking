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
        self.declare_parameter('command_apply_sec', 1.0)
        self.declare_parameter('command_cooldown_sec', 30.0)
        self.declare_parameter('max_pwm', 80)
        self.declare_parameter('linear_gain', 50.0)
        self.declare_parameter('angular_gain', 35.0)

        port = str(self.get_parameter('port').value)
        baud = int(self.get_parameter('baud').value)
        self.cmd_timeout_sec = float(self.get_parameter('cmd_timeout_sec').value)
        self.send_period_sec = float(self.get_parameter('send_period_sec').value)
        self.heartbeat_sec = float(self.get_parameter('heartbeat_sec').value)
        self.command_apply_sec = float(self.get_parameter('command_apply_sec').value)
        self.command_cooldown_sec = float(self.get_parameter('command_cooldown_sec').value)
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
        self.apply_until_time = self.get_clock().now()
        self.cooldown_until_time = self.get_clock().now()
        self.last_sent_left = None
        self.last_sent_right = None
        self.last_send_time = self.get_clock().now()

        # Send commands with reduced rate to prevent queue buildup
        self.timer = self.create_timer(self.send_period_sec, self.send_command)

    def cmd_callback(self, msg):
        now = self.get_clock().now()
        linear = float(msg.linear.x)
        angular = float(msg.angular.z)

        left = int(self.linear_gain * linear - self.angular_gain * angular)
        right = int(self.linear_gain * linear + self.angular_gain * angular)

        candidate_left = max(min(left, self.max_pwm), -self.max_pwm)
        candidate_right = max(min(right, self.max_pwm), -self.max_pwm)

        # Respect rest/cooldown between accepted commands
        if now < self.cooldown_until_time:
            self.last_cmd_time = now
            return

        # Accept command: execute briefly, then rest
        self.left = candidate_left
        self.right = candidate_right
        self.apply_until_time = now + rclpy.duration.Duration(seconds=self.command_apply_sec)
        self.cooldown_until_time = now + rclpy.duration.Duration(
            seconds=(self.command_apply_sec + self.command_cooldown_sec)
        )
        self.last_cmd_time = now
        self.get_logger().info(
            f"Accepted cmd: D {self.left} {self.right} 1 | apply={self.command_apply_sec:.1f}s, rest={self.command_cooldown_sec:.1f}s"
        )

    def send_command(self):
        now = self.get_clock().now()
        dt = (now - self.last_cmd_time).nanoseconds / 1e9
        if dt > self.cmd_timeout_sec:
            # No new upstream cmd: keep safe stop
            desired_left = 0
            desired_right = 0
        elif now < self.apply_until_time:
            # During apply window: execute accepted command
            desired_left = self.left
            desired_right = self.right
        else:
            # Rest window: stop robot
            desired_left = 0
            desired_right = 0

        # Only send if command changed OR heartbeat due
        send_due_to_change = (
            desired_left != self.last_sent_left or
            desired_right != self.last_sent_right
        )
        send_due_to_heartbeat = (
            (now - self.last_send_time).nanoseconds / 1e9 >= self.heartbeat_sec
        )

        if not (send_due_to_change or send_due_to_heartbeat):
            return

        command = f"D {desired_left} {desired_right} 1\n"
        try:
            self.ser.write(command.encode())
        except Exception as e:
            self.get_logger().warn(f"Serial write failed: {e}")

        self.last_sent_left = desired_left
        self.last_sent_right = desired_right
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