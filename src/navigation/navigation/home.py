#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from interfaces.msg import Object, Objects

from typing import Tuple
import numpy as np

HOME_CLASS_ID = 'cone'
FRAME_WIDTH = 160 # [TBD] fix it

class Home(Node):
    

    def __init__(self):

        super().__init__("Home")
        self.subscriber = self.create_subscription(Objects, 'state_home', self._home_callback, 10)
        self.publisher_ = self.create_publisher(Twist, "/rbt_vel", 10)
        self.home_detected = False
        self.home_bbox = None
        # PID controller variables
        self.prev_e = 0.0
        self.prev_int = 0.0
        self.delta_t = 0.0 # time delta between to callbacks [TBD]
        
    def _home_callback(self, msg):
        #do something with input, then send vel to robot.
        twist_msg = Twist()
        
        if not self.home_detected:
            for obj in msg.objects:
                if obj.class_id == HOME_CLASS_ID: # find cone
                    self.home_detected = True
                    self.home_bbox = obj.bbox # lock in on the cone
                    break
                if not self.home_detected:
                    twist_msg.angular.z = 0.5 # didn't find cone, spin in place look for it
        else:
            path_to_home_clear = True
            for obj in msg.objects:
                if obj.class_id != HOME_CLASS_ID and self.bbox_overlap(self.home_bbox, obj.bbox):
                    path_to_home_clear = False
                    break
            if path_to_home_clear:
                twist_msg.linear.x = 1.0
                twist_msg.angular.z = self.ceterline_allignment(self.home_bbox)
            else:
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0

        self.publisher_.publish(twist_msg)

    def bbox_overlap(self, home_bbox, obstacle_bbox):
        # IoU mertrics for the overlap 
        intersection_area = max(0, min(home_bbox.xmax, obstacle_bbox.xmax) - max(home_bbox.xmin, obstacle_bbox.xmin)) * \
                            max(0, min(home_bbox.ymax, obstacle_bbox.ymax) - max(home_bbox.ymin, obstacle_bbox.ymin))
        
        home_bbox_area = (home_bbox.xmax - home_bbox.xmin) * (home_bbox.ymax - home_bbox.ymin)
        obstacle_bbox_area = (obstacle_bbox.xmax - obstacle_bbox.xmin) * (obstacle_bbox.ymax - obstacle_bbox.ymin)

        iou = intersection_area / (home_bbox_area + obstacle_bbox_area - intersection_area)

        return iou > 0.8
    
    def centerline_allignment(self, bbox):
        # calculate the centerline allignment
        home_centerline = bbox.xmin + (bbox.xmax - bbox.xmin) / 2

        frame_centerline = FRAME_WIDTH / 2

        error = frame_centerline - home_centerline

        # PID controller to allign the frame_centerline with the home_cneterline
        v_0, omega, self.prev_e, self.prev_int = self.PIDController(
            v_0=1.0,  # You will need to update this with the actual linear velocity
            frame_centerline_hat=frame_centerline,
            home_centerline_ref=home_centerline,
            prev_e=self.prev_e,
            prev_int=self.prev_int,
            delta_t=self.delta_t
        )

        return omega

    def PIDController(
            v_0: float,
            frame_centerline_hat: float,
            home_centerline_ref: float,
            prev_e: float,
            prev_int: float,
            delta_t: float
    ) -> Tuple[float, float, float, float]:

        # Tracking error 
        e = home_centerline_ref - frame_centerline_hat
        # Integral error
        e_int = prev_int + e*delta_t
        # Finite Differences error
        e_der = (e - prev_e)/delta_t

        # Anti-windup for the Integral part of the controller
        # [AIM] prevent the integral term from accumualting excesively when system reaches its limits
        # Anti-windup:
        e_int = max(min(e_int, 2), -2)
        # min(e_int, 2) == ensures that the e_int is <= 2 UPPER BOUND
        # max(min(e_int, 2), -2) == ensures that the e_int is >= -2 LOWER BOUND

        k_p = 5
        k_i = 0.2
        k_d = 0.1

        # GOAL: Control the heading error of the robit by computing the angular speed
        omega = k_p*e + k_i*e_int + k_d*e_der

        # Hint: print for debugging
        # print(f"\n\nDelta time : {delta_t} \nE : {np.rad2deg(e)} \nE int : {e_int} \nPrev e : {prev_e} \nU : {[v_0, omega]} \nTheta hat: {np.rad2deg(theta_hat)} \n")
        # # ---
        return v_0, omega, e, e_int

def main(args=None):
    rclpy.init(args=args)
    node=Home()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
        main()