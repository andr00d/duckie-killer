import time

import cv2
import rclpy
from rclpy.node import Node
from interfaces.msg import Object, Objects

# simple setup for gesture node
class Gesture(Node):
    def __init__(self):
        super().__init__("gesture")

        self.timer = self.create_timer(.2, self._gesture_callback)
        self.publisher_ = self.create_publisher(Objects, "objects", 10)

    def _gesture_callback(self):
        object_msg = Object()
        object_msg.x = 1.0
        object_msg.y = 1.0
        object_msg.x_width = 1.0
        object_msg.y_width = 1.0
        object_msg.type = "test"

        objects_msg = Objects()
        objects_msg.objects = [object_msg]
        self.publisher_.publish(objects_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Gesture()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
