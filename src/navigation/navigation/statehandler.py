import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.node import Node

import interfaces
from interfaces.msg import Object, Objects

STOP = "Open_Palm"
SURV = "Thumb_Down"
GUARD = "Thumb_Up"
HOME = "Closed_Fist"

class Statehandler(Node):
    def __init__(self):
        super().__init__("Statehandler")

        self.subscriber = self.create_subscription(Objects, "objects", self._objects_callback, 10)
        self.pub_surveillance = self.create_publisher(Objects, "state_surveillance", 10)
        self.pub_guard = self.create_publisher(Objects, "state_guard", 10)
        self.pub_home = self.create_publisher(Objects, "state_home", 10)

        self.curr_state = None

    def _objects_callback(self, msgs):
        # self.pub_surveillance.publish(msgs) # this would exit the callback prematurely
        # return
        if not msgs.objects:
            if self.curr_state is not None:
                self.curr_state.publish(msgs)
            return

        gesture = [m.gesture for m in msgs.objects]

        if(len(gesture) == 0 and self.curr_state != None):

            self.curr_state.publish(msgs)

        if gesture == STOP and self.curr_state != None:
            object_msg = Object()
            object_msg.gesture = "clear" 

            objects_msg = Objects()
            objects_msg.objects = [object_msg]

            self.curr_state.publish(objects_msg)
            self.curr_state = None

        elif gesture == SURV and self.curr_state is None:
            self.curr_state = self.pub_surveillance
        elif gesture == GUARD and self.curr_state is None:
            self.curr_state = self.pub_guard
        elif gesture == HOME and self.curr_state is None:
            self.curr_state = self.pub_home

        if self.curr_state is not None:
            self.curr_state.publish(msgs)

def main(args=None):
    rclpy.init(args=args)
    node = Statehandler()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

