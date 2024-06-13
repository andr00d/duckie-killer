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

        initial_msg = Objects()
        initial_object = Object()
        initial_object.gesture = "clear"
        initial_msg.objects = [initial_object]
        self.pub_surveillance.publish(initial_msg)
        self.pub_guard.publish(initial_msg)
        self.pub_home.publish(initial_msg)
        self.get_logger().info(f'Initial state set to: {self.curr_state}')

    def _objects_callback(self, msgs):

        obj_msg = msgs.objects[0]
        objects_msg = Objects()
        objects_msg.objects = [obj_msg]

        if obj_msg.gesture == STOP and self.curr_state != STOP:
            obj_msg = Object(); 
            obj_msg.gesture = "clear"
            objects_msg.objects = [obj_msg]
            self.pub_surveillance.publish(objects_msg)
            self.pub_guard.publish(objects_msg)
            self.pub_home.publish(objects_msg)
            self.curr_state = STOP
            self.get_logger().info("Transitioned to STOP mode")
        else:
            if obj_msg.gesture == SURV:
                self.curr_state = SURV
                self.get_logger().info("Transitioned to SURVEILLANCE mode")
            elif obj_msg.gesture == GUARD:
                self.curr_state = GUARD
                self.get_logger().info("Transitioned to GUARD mode")
            elif obj_msg.gesture == HOME:
                self.curr_state = HOME
                self.get_logger().info("Transitioned to HOME mode")

        if self.curr_state == SURV:
            self.pub_surveillance.publish(msgs)

        if self.curr_state == GUARD:
            self.pub_surveillance.publish(msgs)

        if self.curr_state == HOME:
            self.pub_surveillance.publish(msgs)

def main(args=None):
    rclpy.init(args=args)
    node = Statehandler()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
