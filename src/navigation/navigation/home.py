#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from interfaces.msg import Object, Objects
from datetime import datetime

from typing import Tuple
import numpy as np

# person to make debugging easier
HOME_TYPE = 'person'
bbox_min = 0.2
bbox_max = 0.8

class Home(Node):
    def __init__(self):
        super().__init__("Home")
        self.subscriber = self.create_subscription(Objects, 'state_home', self._home_callback, 10)
        self.publisher_ = self.create_publisher(Twist, "/rbt_vel", 10)
        self.home_detected = False
        self.home_bbox = None
        # PID controller variables
        self.last_update = datetime.now()
        self.prev_e = 0.0
        self.prev_int = 0.0
        self.delta_t = 0.0

    def normalize(self, value, min_value, max_value):
        normalized = (value - min_value) / (max_value - min_value)
        return 2 * normalized - 1
    
    def _home_callback(self, msg):
        if len(msg.objects) > 0 and msg.objects[0].gesture == "clear":
            self.home_detected = False
            return

        curr_time = datetime.now()
        delta = self.last_update - curr_time
        self.delta_t = delta.seconds + delta.microseconds/1E6
        self.last_update = curr_time

        twist_msg = Twist()
        self.home_detected, self.home_bbox = self.find_home(msg.objects)
        
        if not self.home_detected:
            self.get_logger().info("no home detected")
            if not self.home_detected:
                twist_msg.angular.z = self.normalize(0.5, -1, 1) # didn't find cone, spin in place look for it
        else:
            path_to_home_clear = self.check_path_to_home(msg.objects)
            if path_to_home_clear:
                self.get_logger().info("home path clear")
                twist_msg.linear.x, twist_msg.angular.z = self.centerline_allignment(self.home_bbox)
                twist_msg.linear.x = self.normalize(twist_msg.linear.x, -1, 1)
                twist_msg.angular.z = self.normalize(twist_msg.angular.z, -1, 1)
            else:
                self.get_logger().info("home path obstructed")
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0

        self.publisher_.publish(twist_msg)

    def find_home(self, objects):
        for obj in objects:
            if obj.type == HOME_TYPE: # find home
                return True, obj
        return False, None

    def check_path_to_home(self, objects):
        for obj in objects:
            if obj.type != HOME_TYPE:  #and self.bbox_overlap(self.home_bbox, obj.bbox):
                return False
        return True

    def centerline_allignment(self, bbox):
        # calculate the centerline allignment
        home_centerline = bbox.x + bbox.width / 2
        frame_centerline = 0.5

        # Calculate the area of the home_bbox
        home_bbox_area = bbox.width * bbox.height

        # Safety mechanism: if bbox is too large, stop the robot
        if home_bbox_area > bbox_max:
            return 0.0, 0.0

        # Control the velocity based on the ratio
        v_0, omega = self.velocity_control(home_bbox_area, home_centerline, frame_centerline)

        return v_0, omega

    def velocity_control(self, home_bbox_area, home_centerline, frame_centerline):
        bbox_to_screen_ratio = home_bbox_area

        if bbox_to_screen_ratio < bbox_min: # until the bbox is less then 20% of screen the speed is const
            v_0 = 1.0  # Keep the velocity constant when the robot is far from home
        elif .2 <= bbox_to_screen_ratio < bbox_max:
            v_0 = 1.0 - (bbox_to_screen_ratio - bbox_min) / (bbox_max - bbox_min)  # decrease vel prop. to the bbox ratio
        else:
            v_0 = 0.0 

        # PID controller to align the frame_centerline with the home_centerline
        omega, self.prev_e, self.prev_int = self.PIDController(
            frame_centerline_hat=frame_centerline,
            home_centerline_ref=home_centerline,
            prev_e=self.prev_e,
            prev_int=self.prev_int,
            delta_t=self.delta_t
        )
    
        return v_0, omega

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