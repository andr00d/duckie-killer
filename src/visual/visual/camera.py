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

        self.timer = self.create_timer((1/24), self._camera_callback)
        self.publisher_ = self.create_publisher(CompressedImage, "cam/compressed", 10)
        self.bridge = CvBridge()
        # self.vid = cv2.VideoCapture(2) 
        self.vid = cv2.VideoCapture(0) 

    def _camera_callback(self):
        ret, frame = self.vid.read() 
        
        if(ret):
            self.publisher_.publish(self.bridge.cv2_to_compressed_imgmsg(frame, dst_format = "jpg"))
            


def main(args=None):
    rclpy.init(args=args)
    node = Camera()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
