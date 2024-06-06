#!/usr/bin/env python3
"""
Object.msg
float64 x
float64 y
float64 width
float64 height
string type
///////////////
Objects.msg
Object[] objects
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from interfaces.msg import Object, Objects

from typing import Tuple
import numpy as np

HOME_CLASS_ID = 'cone barrel'
FRAME_WIDTH = 160 # [TBD] fix it
FRAME_HEIGHT = 160 # [TBD] fix it

class Home(Node):
    def __init__(self):
        super().__init__("Home")
        self.subscriber = self.create_subscription(Objects, 'state_home', self._home_callback, 10)
        self.publisher_ = self.create_publisher(Twist, "/rbt_vel", 10)
        self.home_detected = False
        # self.home_lost = False => this will require an odometer data to draw a path
        self.home_bbox = None
        # PID controller variables
        self.prev_e = 0.0
        self.prev_int = 0.0
        self.delta_t = 0.0 # time delta between to callbacks [TBD]

    def normalize(self, value, min_value, max_value):
        normalized = (value - min_value) / (max_value - min_value)
        return 2 * normalized - 1
    
    def _home_callback(self, msg):
        twist_msg = Twist()

        self.home_detected, self.home_bbox = self.find_home(msg.objects) # look for home each msg

        if not self.home_detected:
            self.home_detected, self.home_bbox = self.find_home(msg.objects)
            if not self.home_detected:
                twist_msg.angular.z = self.normalize(0.5, -1, 1) # didn't find cone, spin in place look for it
        else:
            path_to_home_clear = self.check_path_to_home(msg.objects)
            if path_to_home_clear:
                twist_msg.linear.x, twist_msg.angular.z = self.centerline_allignment(self.home_bbox)
                twist_msg.linear.x = self.normalize(twist_msg.linear.x, -1, 1)
                twist_msg.angular.z = self.normalize(twist_msg.angular.z, -1, 1)
            else:
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0

        self.publisher_.publish(twist_msg)

    def find_home(self, objects):
        for obj in objects:
            if obj.type == HOME_CLASS_ID: # find cone
                return True, (obj.x, obj.y, obj.width, obj.height)
        return False, None

    def check_path_to_home(self, objects):
        """
        Check if the path is clear. In case the home_bbx is still in the frame but the path is blocked.
        """
        for obj in objects:
            obj.obstacle_bbox = (obj.x, obj.y, obj.width, obj.height)
            if obj.type != HOME_CLASS_ID and self.bbox_overlap(self.home_bbox, obj.obstacle_bbox):
                return False
        return True
    
    def bbox_overlap(self, home_bbox, obstacle_bbox):
        # IoU mertrics for the overlap 
        intersection_area = max(0, min(home_bbox.xmax, obstacle_bbox.xmax) - max(home_bbox.xmin, obstacle_bbox.xmin)) * \
                            max(0, min(home_bbox.ymax, obstacle_bbox.ymax) - max(home_bbox.ymin, obstacle_bbox.ymin))

        home_bbox_area = (home_bbox.xmax - home_bbox.xmin) * (home_bbox.ymax - home_bbox.ymin)
        obstacle_bbox_area = (obstacle_bbox.xmax - obstacle_bbox.xmin) * (obstacle_bbox.ymax - obstacle_bbox.ymin)

        iou = intersection_area / (home_bbox_area + obstacle_bbox_area - intersection_area)

        return iou > 0.4

    def centerline_allignment(self, bbox):
        # calculate the centerline allignment
        home_centerline = bbox.x + bbox.width / 2
        frame_centerline = FRAME_WIDTH / 2

        # Calculate the area of the home_bbox
        home_bbox_area = bbox.width * bbox.height
        frame_area = FRAME_WIDTH * FRAME_HEIGHT

        # Safety mechanism: if bbox is too large, stop the robot
        if home_bbox_area / frame_area > 0.8:
            return 0.0, 0.0

        # Keep the linear velocity constant
        v = 0.1

        # PID controller to align the frame_centerline with the home_centerline
        omega, self.prev_e, self.prev_int = self.PIDController(
            frame_centerline_hat=frame_centerline,
            home_centerline_ref=home_centerline,
            prev_e=self.prev_e,
            prev_int=self.prev_int,
            delta_t=self.delta_t
        )

        return v, omega

    def PIDController(
        self,
        frame_centerline_hat: float,
        home_centerline_ref: float,
        prev_e: float,
        prev_int: float,
        delta_t: float
    ) -> Tuple[float, float, float]:

        # Tracking error 
        e = home_centerline_ref - frame_centerline_hat
        # Integral error
        e_int = prev_int + e*delta_t
        # Finite Differences error
        e_der = (e - prev_e)/delta_t
    
        # Anti-windup:
        e_int = max(min(e_int, 2), -2)
    
        k_p = 5
        k_i = 0.2
        k_d = 0.1
    
        # GOAL: Control the alignment error of the robot directly with the twist_msg.angular.z
        ang_z = k_p*e + k_i*e_int + k_d*e_der
    
        return ang_z, e, e_int

def main(args=None):
    rclpy.init(args=args)
    node=Home()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
        main()