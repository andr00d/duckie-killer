import os

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.node import Node

import interfaces
from interfaces.msg import Object


class Navigation(Node):
    def __init__(self):
        super().__init__("Navigation")

        self.subscriber = self.create_subscription(Object, "objects", self._objects_callback, 10)
        self.publisher_ = self.create_publisher(Twist, "rbt_vel", 10)

    def _objects_callback(self, msg):
        # Do something with the objects, then send robot velocity

        twist_msg = Twist()
        twist_msg.linear.x = 1
        twist_msg.angular.y = 0
        twist_msg.angular.z = 0
        self.publisher_.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Navigation()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
