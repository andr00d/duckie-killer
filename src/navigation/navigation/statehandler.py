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

        self.curr_state = STOP

    def _objects_callback(self, msgs):
        # TODO fix the objects capturing
        gesture = [obj_msg.gesture for obj_msg in msgs.objects]
        objects_msg = Objects()


        # if(len(gesture) == 0 and self.curr_state != None):
        #     self.curr_state.publish(msgs)
        # self.get_logger().info(gesture[0])
        # self.get_logger().info(f'The initial state: {self.curr_state}')

        if gesture[0] == STOP and self.curr_state != None:
            object_msg = Object()
            object_msg.gesture = "clear" 

            objects_msg.objects = [object_msg]
            self.pub_surveillance.publish(objects_msg)
            self.pub_guard.publish(objects_msg)
            self.pub_home.publish(objects_msg)
            self.curr_state = None
            self.get_logger().info("I am in Nothing mode")
        elif gesture[0] == SURV and self.curr_state is None:
            self.curr_state = SURV
            self.pub_surveillance.publish(objects_msg)
            self.get_logger().info("I am in SRUV mode")
        elif gesture[0] == GUARD and self.curr_state is None:
            self.curr_state = GUARD
            self.pub_guard.publish(objects_msg)
            self.get_logger().info("I am in GUARD mode")
        elif gesture[0] == HOME and self.curr_state is None:
            self.curr_state = HOME
            self.pub_home.publish(objects_msg)
            self.get_logger().info("I am in HOME mode")


def main(args=None):
    rclpy.init(args=args)
    node = Statehandler()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

