import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # Publish at 10Hz
        self.cap = cv2.VideoCapture(2)  # Open the webcam
        self.bridge = CvBridge()
        self.get_logger().info('Camera Publisher Node has been started.')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher.publish(msg)
        else:
            self.get_logger().warn('Unable to capture image from webcam')

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    rclpy.spin(node)
    node.cap.release()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

