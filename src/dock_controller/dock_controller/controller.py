import math

import rclpy
from geometry_msgs.msg import Point, Twist
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Image


class DockController(Node):
    """Autonomous docking controller (step-by-step)."""

    def __init__(self):
        super().__init__('dock_controller')

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.dock_status_pub = self.create_publisher(Point, '/dock_status', 10)
        self.charging_state_pub = self.create_publisher(Point, '/charging_state', 10)

        # Subscriptions
        self.create_subscription(Point, '/dock_position', self.marker_callback, 10)
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.create_subscription(BatteryState, '/battery_state', self.battery_callback, 10)
        self.create_subscription(Point, '/dock_robot', self.dock_trigger_callback, 10)
        self.create_subscription(Point, '/undock_robot', self.undock_callback, 10)
        self.create_subscription(Point, '/stop_robot', self.stop_callback, 10)
        self.create_subscription(Point, '/dock_mode', self.mode_callback, 10)

        # Vision/Battery memory
        self.image_width = None
        self.marker_x = None
        self.marker_area = 0.0
        self.marker_id = None
        self.last_marker_time = None

        self.last_battery_voltage = None
        self.last_battery_time = None
        self.current_voltage = 0.0
        self.charging_state = 'UNKNOWN'

        # Control flags and state
        self.dock_requested = False
        self.emergency_stop = False
        self.state = 'IDLE'
        self.state_enter_time = self.get_clock().now()

        # Move/rest phase for "do command -> wait -> check"
        self.phase = 'REST'
        self.phase_until = self.get_clock().now()
        self.phase_twist = Twist()

        # Parameters
        self.declare_parameter('target_marker_id', -1)
        self.declare_parameter('search_angular_speed', 0.12)
        self.declare_parameter('approach_linear_speed', 0.08)
        self.declare_parameter('marker_timeout_sec', 0.8)
        self.declare_parameter('search_timeout_sec', 25.0)
        self.declare_parameter('approach_timeout_sec', 20.0)
        self.declare_parameter('align_error_threshold', 0.08)
        self.declare_parameter('dock_area_threshold', 18000.0)
        self.declare_parameter('undock_linear_speed', -0.08)
        self.declare_parameter('undock_duration_sec', 2.5)
        self.declare_parameter('command_move_sec', 0.8)
        self.declare_parameter('command_rest_sec', 1.2)
        self.declare_parameter('distance_scale_cm_px', 2400.0)
        self.declare_parameter('charging_voltage_rise_per_sec', 0.01)
        self.declare_parameter('low_battery_alarm_v', 6.0)

        self.target_marker_id = int(self.get_parameter('target_marker_id').value)
        self.search_angular_speed = float(self.get_parameter('search_angular_speed').value)
        self.approach_linear_speed = float(self.get_parameter('approach_linear_speed').value)
        self.marker_timeout_sec = float(self.get_parameter('marker_timeout_sec').value)
        self.search_timeout_sec = float(self.get_parameter('search_timeout_sec').value)
        self.approach_timeout_sec = float(self.get_parameter('approach_timeout_sec').value)
        self.align_error_threshold = float(self.get_parameter('align_error_threshold').value)
        self.dock_area_threshold = float(self.get_parameter('dock_area_threshold').value)
        self.undock_linear_speed = float(self.get_parameter('undock_linear_speed').value)
        self.undock_duration_sec = float(self.get_parameter('undock_duration_sec').value)
        self.command_move_sec = float(self.get_parameter('command_move_sec').value)
        self.command_rest_sec = float(self.get_parameter('command_rest_sec').value)
        self.distance_scale_cm_px = float(self.get_parameter('distance_scale_cm_px').value)
        self.charging_voltage_rise_per_sec = float(self.get_parameter('charging_voltage_rise_per_sec').value)
        self.low_battery_alarm_v = float(self.get_parameter('low_battery_alarm_v').value)

        # Timers
        self.control_timer = self.create_timer(0.1, self.control_loop)
        self.status_timer = self.create_timer(1.0, self.publish_status)

    def image_callback(self, msg):
        self.image_width = msg.width

    def marker_callback(self, msg):
        marker_id = int(msg.z)
        if self.target_marker_id >= 0 and marker_id != self.target_marker_id:
            return
        self.marker_x = float(msg.x)
        self.marker_area = float(msg.y)
        self.marker_id = marker_id
        self.last_marker_time = self.get_clock().now()

    def battery_callback(self, msg):
        now = self.get_clock().now()
        voltage = float(msg.voltage)
        self.current_voltage = voltage

        if voltage > 0.0 and voltage <= self.low_battery_alarm_v:
            self.charging_state = 'LOW_BATTERY_ALARM'
            self.last_battery_voltage = voltage
            self.last_battery_time = now
            return

        if msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING:
            self.charging_state = 'CHARGING'
        elif msg.power_supply_status in (
            BatteryState.POWER_SUPPLY_STATUS_DISCHARGING,
            BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING,
        ):
            self.charging_state = 'DISCHARGING'
        else:
            if self.last_battery_voltage is not None and self.last_battery_time is not None:
                dt = (now - self.last_battery_time).nanoseconds / 1e9
                if dt > 0.0:
                    slope = (voltage - self.last_battery_voltage) / dt
                    self.charging_state = (
                        'CHARGING' if slope >= self.charging_voltage_rise_per_sec else 'DISCHARGING'
                    )

        self.last_battery_voltage = voltage
        self.last_battery_time = now

    def dock_trigger_callback(self, msg):
        if msg.x > 0.5:
            self.dock_requested = True
            if self.state == 'IDLE' and not self.emergency_stop:
                self.set_state('SEARCH')

    def undock_callback(self, msg):
        if msg.x > 0.5 and not self.emergency_stop:
            self.dock_requested = False
            self.set_state('UNDOCK')

    def stop_callback(self, msg):
        self.emergency_stop = msg.x > 0.5
        if self.emergency_stop:
            self.set_state('STOPPED')

    def mode_callback(self, msg):
        mode = int(msg.x)
        if mode == 0:
            self.dock_requested = False
            if not self.emergency_stop:
                self.set_state('IDLE')
        elif mode == 1:
            self.dock_requested = True
            if not self.emergency_stop:
                self.set_state('SEARCH')
        elif mode == 2:
            self.dock_requested = False
            if not self.emergency_stop:
                self.set_state('UNDOCK')
        elif mode == 3:
            self.emergency_stop = True
            self.set_state('STOPPED')

    def set_state(self, new_state):
        if self.state != new_state:
            self.state = new_state
            self.state_enter_time = self.get_clock().now()

    def state_age(self):
        return (self.get_clock().now() - self.state_enter_time).nanoseconds / 1e9

    def marker_recent(self):
        if self.last_marker_time is None:
            return False
        age = (self.get_clock().now() - self.last_marker_time).nanoseconds / 1e9
        return age <= self.marker_timeout_sec

    def distance_cm(self):
        if self.marker_area is None or self.marker_area <= 1.0:
            return -1.0
        return self.distance_scale_cm_px / math.sqrt(self.marker_area)

    def heading_error(self):
        if self.image_width is None or self.marker_x is None:
            return 0.0
        center_x = 0.5 * float(self.image_width)
        return (self.marker_x - center_x) / max(center_x, 1.0)

    def start_move_phase(self, twist):
        self.phase = 'MOVE'
        self.phase_twist = twist
        self.phase_until = self.get_clock().now() + rclpy.duration.Duration(seconds=self.command_move_sec)

    def start_rest_phase(self):
        self.phase = 'REST'
        self.phase_twist = Twist()
        self.phase_until = self.get_clock().now() + rclpy.duration.Duration(seconds=self.command_rest_sec)

    def apply_phase_command(self):
        now = self.get_clock().now()
        if now < self.phase_until:
            self.cmd_pub.publish(self.phase_twist)
            return True

        if self.phase == 'MOVE':
            self.start_rest_phase()
            self.cmd_pub.publish(Twist())
            return True

        return False

    def publish_status(self):
        state_codes = {
            'IDLE': 0.0,
            'SEARCH': 1.0,
            'ALIGN': 2.0,
            'APPROACH': 3.0,
            'DOCKED': 4.0,
            'UNDOCK': 5.0,
            'STOPPED': 6.0,
        }
        charging_codes = {
            'UNKNOWN': 0.0,
            'DISCHARGING': 1.0,
            'CHARGING': 2.0,
            'LOW_BATTERY_ALARM': 3.0,
        }

        dock_msg = Point()
        dock_msg.x = state_codes.get(self.state, -1.0)
        dock_msg.y = self.distance_cm()
        dock_msg.z = self.heading_error()
        self.dock_status_pub.publish(dock_msg)

        charge_msg = Point()
        charge_msg.x = charging_codes.get(self.charging_state, 0.0)
        charge_msg.y = self.current_voltage
        charge_msg.z = 1.0 if self.charging_state == 'LOW_BATTERY_ALARM' else 0.0
        self.charging_state_pub.publish(charge_msg)

        self.get_logger().info(
            f"state={self.state} | V={self.current_voltage:.2f}V | dist={dock_msg.y:.1f}cm | err={dock_msg.z:.3f}"
        )

    def control_loop(self):
        if self.emergency_stop:
            self.cmd_pub.publish(Twist())
            return

        if self.apply_phase_command():
            return

        cmd = Twist()

        if self.state == 'IDLE':
            self.cmd_pub.publish(Twist())
            if self.dock_requested:
                self.set_state('SEARCH')
            return

        if self.state == 'SEARCH':
            if self.state_age() > self.search_timeout_sec:
                self.set_state('IDLE')
                self.cmd_pub.publish(Twist())
                return
            if self.marker_recent() and self.image_width is not None:
                self.set_state('ALIGN')
                return
            cmd.angular.z = self.search_angular_speed
            self.start_move_phase(cmd)
            self.cmd_pub.publish(cmd)
            return

        if self.state == 'ALIGN':
            if not self.marker_recent() or self.image_width is None or self.marker_x is None:
                self.set_state('SEARCH')
                return
            err = self.heading_error()
            if abs(err) < self.align_error_threshold:
                self.set_state('APPROACH')
                return
            cmd.angular.z = max(min(-0.45 * err, 0.25), -0.25)
            self.start_move_phase(cmd)
            self.cmd_pub.publish(cmd)
            return

        if self.state == 'APPROACH':
            if self.state_age() > self.approach_timeout_sec:
                self.set_state('IDLE')
                self.cmd_pub.publish(Twist())
                return
            if not self.marker_recent() or self.image_width is None or self.marker_x is None:
                self.set_state('SEARCH')
                return
            err = self.heading_error()
            if abs(err) < 0.05 and self.marker_area >= self.dock_area_threshold:
                self.set_state('DOCKED')
                self.cmd_pub.publish(Twist())
                return
            cmd.linear.x = self.approach_linear_speed
            cmd.angular.z = max(min(-0.35 * err, 0.2), -0.2)
            self.start_move_phase(cmd)
            self.cmd_pub.publish(cmd)
            return

        if self.state == 'DOCKED':
            self.cmd_pub.publish(Twist())
            return

        if self.state == 'UNDOCK':
            if self.state_age() <= self.undock_duration_sec:
                cmd.linear.x = self.undock_linear_speed
                self.start_move_phase(cmd)
                self.cmd_pub.publish(cmd)
                return
            self.set_state('IDLE')
            self.dock_requested = False
            self.cmd_pub.publish(Twist())
            return

        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = DockController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
