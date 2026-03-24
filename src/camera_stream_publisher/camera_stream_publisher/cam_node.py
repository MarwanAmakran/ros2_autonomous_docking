#!/usr/bin/env python3
import time
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

class MinimalV4L2Cam(Node):
    def __init__(self):
        super().__init__('rpi_cam_min')

        # Parameters (change defaults as needed)
        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 5.0)
        self.declare_parameter('fourcc', 'MJPG')   # try 'YUYV' if MJPG not supported
        self.declare_parameter('frame_id', 'camera_frame')
        self.declare_parameter('topic', '/camera/image_raw')

        # --- Use .value to read parameters (Jazzy-safe) ---
        dev      = str(self.get_parameter('device').value)
        width    = int(self.get_parameter('width').value)
        height   = int(self.get_parameter('height').value)
        fps      = float(self.get_parameter('fps').value)
        fourcc_s = str(self.get_parameter('fourcc').value)[:4]  # ensure length ≤ 4
        self.frame_id = str(self.get_parameter('frame_id').value)
        topic    = str(self.get_parameter('topic').value)

        # Publisher (BEST_EFFORT is typical for camera streams)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(Image, topic, qos)

        # Open V4L2 device via OpenCV (fallback to default backend if needed)
        self.cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().warn(f"CAP_V4L2 failed for {dev}, trying default backend…")
            self.cap = cv2.VideoCapture(dev)

        if not self.cap.isOpened():
            self.get_logger().error(f"Cannot open device {dev}")
            raise RuntimeError("VideoCapture open failed")

        # Apply capture properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS,         fps)
        if len(fourcc_s) == 4:
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc_s))

        self.period = max(1.0 / max(fps, 0.1), 0.001)
        self._last  = time.time()

        self.get_logger().info(
            f"📷 Publishing {topic} from {dev} ({width}x{height}@{fps} {fourcc_s})"
        )
        self.timer = self.create_timer(self.period, self._tick)

    def _tick(self):
        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.get_logger().warn("No frame from camera; will retry…")
            return

        # Build sensor_msgs/Image manually (no cv_bridge)
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = frame.shape[0]
        msg.width  = frame.shape[1]
        msg.encoding = 'bgr8'       # OpenCV frames are BGR
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = frame.tobytes()

        self.pub.publish(msg)

        # Soft throttle to target FPS
        dt = time.time() - self._last
        sleep_left = self.period - dt
        if sleep_left > 0:
            time.sleep(sleep_left)
        self._last = time.time()

    def destroy_node(self):
        try:
            if self.cap is not None:
                self.cap.release()
        except Exception:
            pass
        super().destroy_node()

def main():
    rclpy.init()
    node = None
    try:
        node = MinimalV4L2Cam()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
