import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


class UndockCommand(Node):

    def __init__(self):
        super().__init__('undock_command')
        self.pub = self.create_publisher(Point, '/undock_robot', 10)

        # publish once after startup
        self.timer = self.create_timer(0.5, self.publish_once)
        self.sent = False

    def publish_once(self):
        if self.sent:
            return

        msg = Point()
        msg.x = 1.0  # x>0.5 means trigger undock
        msg.y = 0.0
        msg.z = 0.0
        self.pub.publish(msg)
        self.get_logger().info('Undock trigger published on /undock_robot')
        self.sent = True

        # give ROS middleware a short moment, then shutdown
        self.destroy_timer(self.timer)
        self.create_timer(0.5, self.shutdown_node)

    def shutdown_node(self):
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = UndockCommand()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

