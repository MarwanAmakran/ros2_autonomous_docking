import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial


class CmdVelSerialBridge(Node):

    def __init__(self):
        super().__init__('cmdvel_serial_bridge')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 57600)
        self.declare_parameter('cmd_timeout_sec', 1.0)
        self.declare_parameter('send_period_sec', 0.1)
        self.declare_parameter('heartbeat_sec', 2.0)
        self.declare_parameter('max_pwm', 80)
        self.declare_parameter('linear_gain', 50.0)
        self.declare_parameter('angular_gain', 35.0)
        self.declare_parameter('min_pwm_move', 14)

        port = str(self.get_parameter('port').value)
        baud = int(self.get_parameter('baud').value)
        self.cmd_timeout_sec = float(self.get_parameter('cmd_timeout_sec').value)
        self.send_period_sec = float(self.get_parameter('send_period_sec').value)
        self.heartbeat_sec = float(self.get_parameter('heartbeat_sec').value)
        self.max_pwm = int(self.get_parameter('max_pwm').value)
        self.linear_gain = float(self.get_parameter('linear_gain').value)
        self.angular_gain = float(self.get_parameter('angular_gain').value)
        self.min_pwm_move = int(self.get_parameter('min_pwm_move').value)

        self.ser = serial.Serial(port, baud, timeout=1)
        print(f"[BRIDGE] Connected to {port} @ {baud}", flush=True)

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10)

        self.left = 0
        self.right = 0
        self.last_rx_left = None
        self.last_rx_right = None
        self.last_cmd_time = self.get_clock().now()
        self.last_sent_left = None
        self.last_sent_right = None
        self.last_send_time = self.get_clock().now()

        # Send commands with reduced rate to prevent queue buildup
        self.timer = self.create_timer(self.send_period_sec, self.send_command)

    def _apply_deadband(self, pwm):
        if pwm == 0:
            return 0
        sign = 1 if pwm > 0 else -1
        mag = abs(int(pwm))
        if mag < self.min_pwm_move:
            mag = self.min_pwm_move
        return sign * mag

    def cmd_callback(self, msg):
        now = self.get_clock().now()
        linear = float(msg.linear.x)
        angular = float(msg.angular.z)

        left = int(self.linear_gain * linear - self.angular_gain * angular)
        right = int(self.linear_gain * linear + self.angular_gain * angular)

        candidate_left = max(min(left, self.max_pwm), -self.max_pwm)
        candidate_right = max(min(right, self.max_pwm), -self.max_pwm)

        # Apply minimum PWM to avoid tiny ineffective commands like D -1 1 1
        new_left = self._apply_deadband(candidate_left)
        new_right = self._apply_deadband(candidate_right)

        self.left = new_left
        self.right = new_right
        self.last_cmd_time = now

        # Avoid terminal spam: log only when received command changes.
        if new_left != self.last_rx_left or new_right != self.last_rx_right:
            print(f"[BRIDGE RX] L={new_left:3d}, R={new_right:3d}", flush=True)
            self.last_rx_left = new_left
            self.last_rx_right = new_right

    def send_command(self):
        now = self.get_clock().now()
        dt = (now - self.last_cmd_time).nanoseconds / 1e9
        if dt > self.cmd_timeout_sec:
            desired_left = 0
            desired_right = 0
        else:
            desired_left = self.left
            desired_right = self.right

        # Only send if command changed OR heartbeat due (but heartbeat sends ZERO if timeout)
        send_due_to_change = (
            desired_left != self.last_sent_left or
            desired_right != self.last_sent_right
        )
        send_due_to_heartbeat = (
            (desired_left != 0 or desired_right != 0) and
            (now - self.last_send_time).nanoseconds / 1e9 >= self.heartbeat_sec
        )

        if not (send_due_to_change or send_due_to_heartbeat):
            return

        command = f"D {desired_left} {desired_right} 1\n"
        print(f"[BRIDGE SEND] {command.strip()}", flush=True)
        try:
            self.ser.write(command.encode())
        except Exception as e:
            print(f"[BRIDGE ERROR] Serial write failed: {e}", flush=True)

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