import os

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.node import Node

import interfaces
from interfaces.msg import Object, Objects


class Navigation(Node):
    def __init__(self):
        super().__init__("Navigation")

        self.subscriber = self.create_subscription(Objects, "objects", self._objects_callback, 10)
        self.pub_surveillance_ = self.create_publisher(Objects, "state_surveillance", 10)
        self.pub_guard = self.create_publisher(Objects, "state_guard", 10)
        self.pub_home = self.create_publisher(Objects, "state_home", 10)

    def _objects_callback(self, msg):
        # Do something with the objects, then send objects to correct node

        self.pub_surveillance_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Navigation()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
