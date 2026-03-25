import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
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

        # -1 means: accept any marker ID
        self.declare_parameter('target_marker_id', -1)
        self.target_marker_id = int(self.get_parameter('target_marker_id').value)

        # Camera publishers are often BEST_EFFORT
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            qos)

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
            marker_index = 0
            marker_id = int(ids[0][0])

            if self.target_marker_id >= 0:
                matches = np.where(ids.flatten() == self.target_marker_id)[0]
                if len(matches) == 0:
                    return
                marker_index = int(matches[0])
                marker_id = self.target_marker_id
            else:
                marker_id = int(ids[marker_index][0])

            marker_corners = corners[marker_index][0].astype(np.float32)
            center = np.mean(marker_corners, axis=0)
            area = float(cv2.contourArea(marker_corners))

            point = Point()
            point.x = float(center[0])
            # Use y to carry marker area (px^2) for docking distance heuristic
            point.y = area
            point.z = float(marker_id)

            self.publisher.publish(point)

            self.get_logger().info(
                f"ArUco ID={marker_id} x={point.x:.1f}, area={area:.1f}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
