import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import Image, BatteryState
import math


class DockController(Node):

    def __init__(self):
        super().__init__('dock_controller')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.dock_status_pub = self.create_publisher(Point, '/dock_status', 10)
        self.charging_state_pub = self.create_publisher(Point, '/charging_state', 10)

        self.create_subscription(
            Point,
            '/dock_position',
            self.marker_callback,
            10)

        self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)

        self.create_subscription(
            BatteryState,
            '/battery_state',
            self.battery_callback,
            10)

        self.create_subscription(
            Point,
            '/undock_robot',
            self.undock_callback,
            10)

        self.image_width = None
        self.marker_x = None
        self.marker_area = 0.0
        self.marker_id = None
        self.last_marker_time = None
        self.state_enter_time = self.get_clock().now()

        self.last_battery_voltage = None
        self.last_battery_time = None
        self.charging_state = "UNKNOWN"

        self.declare_parameter('target_marker_id', -1)
        self.declare_parameter('search_angular_speed', 0.12)
        self.declare_parameter('approach_linear_speed', 0.08)
        self.declare_parameter('marker_timeout_sec', 0.6)
        self.declare_parameter('dock_area_threshold', 18000.0)
        self.declare_parameter('approach_timeout_sec', 20.0)
        self.declare_parameter('undock_linear_speed', -0.08)
        self.declare_parameter('undock_duration_sec', 2.5)
        self.declare_parameter('charging_voltage_rise_per_sec', 0.01)

        self.target_marker_id = int(self.get_parameter('target_marker_id').value)
        self.search_angular_speed = float(self.get_parameter('search_angular_speed').value)
        self.approach_linear_speed = float(self.get_parameter('approach_linear_speed').value)
        self.marker_timeout_sec = float(self.get_parameter('marker_timeout_sec').value)
        self.dock_area_threshold = float(self.get_parameter('dock_area_threshold').value)
        self.approach_timeout_sec = float(self.get_parameter('approach_timeout_sec').value)
        self.undock_linear_speed = float(self.get_parameter('undock_linear_speed').value)
        self.undock_duration_sec = float(self.get_parameter('undock_duration_sec').value)
        self.charging_voltage_rise_per_sec = float(self.get_parameter('charging_voltage_rise_per_sec').value)

        # Basic state machine
        self.state = "SEARCH"
        self.publish_dock_status()

        self.timer = self.create_timer(0.1, self.control_loop)
        self.charging_timer = self.create_timer(1.0, self.publish_charging_status)

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
        # Prefer explicit battery status if available
        if msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING:
            self.charging_state = "CHARGING"
        elif msg.power_supply_status in (
            BatteryState.POWER_SUPPLY_STATUS_DISCHARGING,
            BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING,
        ):
            self.charging_state = "DISCHARGING"
        else:
            # Fallback: estimate charging from voltage slope
            now = self.get_clock().now()
            voltage = float(msg.voltage)
            if self.last_battery_voltage is not None and self.last_battery_time is not None:
                dt = (now - self.last_battery_time).nanoseconds / 1e9
                if dt > 0.0:
                    slope = (voltage - self.last_battery_voltage) / dt
                    self.charging_state = (
                        "CHARGING"
                        if slope >= self.charging_voltage_rise_per_sec
                        else "DISCHARGING"
                    )

            self.last_battery_voltage = voltage
            self.last_battery_time = now

    def undock_callback(self, msg):
        if msg.x > 0.5 and self.state == "DOCKED":
            self.set_state("UNDOCK")

    def marker_recent(self):
        if self.last_marker_time is None:
            return False

        age = (self.get_clock().now() - self.last_marker_time).nanoseconds / 1e9
        return age <= self.marker_timeout_sec

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def set_state(self, new_state):
        if self.state != new_state:
            self.state = new_state
            self.state_enter_time = self.get_clock().now()
            self.publish_dock_status()
            self.get_logger().info(f"STATE CHANGE → {new_state}")

    def state_age(self):
        return (self.get_clock().now() - self.state_enter_time).nanoseconds / 1e9

    def publish_dock_status(self):
        state_codes = {
            "IDLE": 0.0,
            "SEARCH": 1.0,
            "ALIGN": 2.0,
            "APPROACH": 3.0,
            "DOCKED": 4.0,
            "UNDOCK": 5.0,
        }
        msg = Point()
        msg.x = state_codes.get(self.state, -1.0)
        msg.y = 0.0
        msg.z = 0.0
        self.dock_status_pub.publish(msg)

    def publish_charging_status(self):
        charging_codes = {
            "UNKNOWN": 0.0,
            "DISCHARGING": 1.0,
            "CHARGING": 2.0,
        }
        msg = Point()
        msg.x = charging_codes.get(self.charging_state, 0.0)
        msg.y = 0.0
        msg.z = 0.0
        self.charging_state_pub.publish(msg)

    def control_loop(self):

        twist = Twist()

        # -------- SEARCH --------
        if self.state == "SEARCH":

            if self.marker_recent() and self.image_width is not None:
                self.set_state("ALIGN")
            else:
                twist.angular.z = self.search_angular_speed
                self.cmd_pub.publish(twist)
                return

        # -------- ALIGN --------
        if self.state == "ALIGN":
            if not self.marker_recent() or self.image_width is None or self.marker_x is None:
                self.set_state("SEARCH")
                self.get_logger().warn("Marker lost → back to SEARCH")
                return

            center_x = 0.5 * float(self.image_width)
            error = (self.marker_x - center_x) / max(center_x, 1.0)

            # Proportional angular control
            twist.angular.z = -0.45 * error
            twist.angular.z = max(min(twist.angular.z, 0.25), -0.25)
            self.cmd_pub.publish(twist)

            if math.fabs(error) < 0.08:
                self.set_state("APPROACH")
            return

        # -------- APPROACH --------
        if self.state == "APPROACH":
            if not self.marker_recent() or self.image_width is None or self.marker_x is None:
                self.set_state("SEARCH")
                self.get_logger().warn("Marker lost while approaching → SEARCH")
                return

            if self.state_age() > self.approach_timeout_sec:
                self.stop_robot()
                self.set_state("SEARCH")
                self.get_logger().warn("Approach timeout → SEARCH")
                return

            center_x = 0.5 * float(self.image_width)
            error = (self.marker_x - center_x) / max(center_x, 1.0)

            twist.linear.x = self.approach_linear_speed
            twist.angular.z = max(min(-0.35 * error, 0.2), -0.2)
            self.cmd_pub.publish(twist)

            # Docked heuristic: centered and marker appears large in frame
            if math.fabs(error) < 0.05 and self.marker_area >= self.dock_area_threshold:
                self.stop_robot()
                self.set_state("DOCKED")
            return

        # -------- DOCKED --------
        if self.state == "DOCKED":
            self.stop_robot()
            return

        # -------- UNDOCK --------
        if self.state == "UNDOCK":
            if self.state_age() <= self.undock_duration_sec:
                twist.linear.x = self.undock_linear_speed
                self.cmd_pub.publish(twist)
                return

            self.stop_robot()
            self.set_state("SEARCH")
            return


def main(args=None):
    rclpy.init(args=args)
    node = DockController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
