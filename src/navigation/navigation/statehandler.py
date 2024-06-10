import os

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.node import Node

import interfaces
from interfaces.msg import Object, Objects


class Statehandler(Node):
    def __init__(self):
        super().__init__("Statehandler")

        self.subscriber = self.create_subscription(Objects, "objects", self._objects_callback, 10)
        self.pub_surveillance = self.create_publisher(Objects, "state_surveillance", 10)
        self.pub_guard = self.create_publisher(Objects, "state_guard", 10)
        self.pub_home = self.create_publisher(Objects, "state_home", 10)

        self.curr_state = None

    def _objects_callback(self, msgs):
        self.pub_surveillance.publish(msgs)
        return

        gestures = ["gesture_1", "gesture_2"]
        hands = [m for m in msgs.objects if m.type in gestures]
        biggest_hand = max(msgs.objects, key=lambda m: m.width * m.height)

        if(len(gestures == 0) and self.curr_state != None):
            self.curr_state.publish(msgs)

        if("stop" in [m.type for m in hands.type] and self.curr_state != None):
            object_msg = Object()
            object_msg.type = "clear"

            objects_msg = Objects()
            objects_msg.objects = [object_msg]
            
            self.curr_state.publish(objects_msg)
            self.curr_state = None

        else:
            if biggest_hand.type == "surveillance":
                self.curr_state = self.pub_surveillance
            if biggest_hand.type == "guard":
                self.curr_state = self.pub_guard
            if biggest_hand.type == "pub_home":
                self.curr_state = self.pub_home


        self.pub_surveillance.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Statehandler()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
