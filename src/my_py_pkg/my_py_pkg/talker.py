# Importeer de ROS 2 Python client library
import rclpy

# Importeer de Node-basisclass om een ROS 2-node te definiëren
from rclpy.node import Node

# Importeer het standaard ROS 2 berichttype 'String'
from std_msgs.msg import String

# Definieer een klasse die onze node representeert (erft van Node)
class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.pub = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(0.5, self.tick)
        self.count = 0

    def tick(self):
        msg = String()
        msg.data = f'Hello from Jazzy #{self.count}'
        self.pub.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')
        self.count += 1

def main():
    rclpy.init()
    node = Talker()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
