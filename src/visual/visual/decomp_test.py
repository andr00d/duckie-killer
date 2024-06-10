import time

import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage

# simple setup for gesture node
class Camera(Node):
    def __init__(self):
        super().__init__("Camera")

        self.subscriber = self.create_subscription(CompressedImage, "cam/compressed", self._cam_callback, 10)
        self.bridge = CvBridge()

    def _cam_callback(self, msg):
        img = self.bridge.compressed_imgmsg_to_cv2(msg)
        cv2.imshow("img", img)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = Camera()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
