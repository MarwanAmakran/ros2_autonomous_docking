import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import Image


class DockController(Node):

    def __init__(self):
        super().__init__('dock_controller')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

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

        self.image_width = None
        self.marker_detected = False

        # TRUE state machine
        self.state = "SEARCH"

        self.timer = self.create_timer(0.1, self.control_loop)

    def image_callback(self, msg):
        self.image_width = msg.width

    def marker_callback(self, msg):
        self.marker_detected = True

    def control_loop(self):

        twist = Twist()

        # -------- SEARCH --------
        if self.state == "SEARCH":

            if self.marker_detected:
                self.state = "STOP"
                self.get_logger().info("STATE CHANGE → STOP")
            else:
                twist.angular.z = 0.2
                self.cmd_pub.publish(twist)
                return

        # -------- STOP --------
        if self.state == "STOP":
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            return


def main(args=None):
    rclpy.init(args=args)
    node = DockController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
