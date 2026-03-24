import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np


class ArucoDetector(Node):

    def __init__(self):
        super().__init__('aruco_detector')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)

        self.publisher = self.create_publisher(
            Point,
            '/dock_position',
            10)

        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.parameters
        )

        if ids is not None:
            center = np.mean(corners[0][0], axis=0)

            point = Point()
            point.x = float(center[0])
            point.y = float(center[1])
            point.z = 0.0

            self.publisher.publish(point)

            self.get_logger().info(
                f"Marker detected at x={point.x:.1f}, y={point.y:.1f}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
