#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ament_index_python.packages import get_package_share_directory
from interfaces.msg import Object, Objects
from playsound import playsound
import random
import time
import os

class SurveilState:
    PATROLLING = 1
    FOLLOWING = 2
    BARKING = 3
    RETURNING = 4

pkg_dir = get_package_share_directory("navigation")
sounds = [
    [0.1, os.path.join(pkg_dir, "sounds/scream_1.mp3")],
    [1.0, os.path.join(pkg_dir, "sounds/scream_2.mp3")],
]

class Surveillance(Node):
    def __init__(self):
        super().__init__("Surveillance")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("bark_delay", rclpy.Parameter.Type.INTEGER),
            ],
        )
        import os

        self.get_logger().info('getcwd:      ' + os.getcwd())
        self.get_logger().info('__file__:    ' + __file__)

        self.subscriber = self.create_subscription(Objects, 'state_surveillance', self._surveillance_callback, 10)
        self.publisher_ = self.create_publisher(Twist, "/rbt_vel", 10)

        self.bark_delay = (self.get_parameter("bark_delay").get_parameter_value().integer_value)
        self.state = SurveilState.PATROLLING
        self.last_bark_time = time.time()
        self.MAX_BBOX_AREA = 0.1
        
    def _surveillance_callback(self, msgs):
        # if message received consists out of a hand, it means state is switched, and we can reset surveillance state
        if msgs.objects[0].type in ["hands"]:
            self.state = PATROLLING
            return

        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0

        if(self.state == SurveilState.PATROLLING):
            cones = [m for m in msgs.objects if m.type == "cone"]
            if len(cones) > 0:
                self.state = SurveilState.FOLLOWING
                self.get_logger().info("found target")
                return

            # no items found, continuing with surveillance
            # surveillance is simple rotation for now
            twist_msg.angular.z = 1.0

        ######################################

        elif (self.state == SurveilState.FOLLOWING):
            cones = [m for m in msgs.objects if m.type == "cone"]
            if len(cones) == 0:
                self.state = SurveilState.RETURNING
                self.get_logger().info("lost target, returning...")
                return

            cone = cones[0]
            bb_area = cone.width * cone.height
            if bb_area > self.MAX_BBOX_AREA:
                self.state = SurveilState.BARKING
                self.get_logger().info("arrive at target, barking.")
                self.last_bark_time = time.time()
                return

            center = x + w / 2.0
            x_norm = -2 * (center - 0.5)
            twist_msg.angular.z = x_norm
            twist_msg.linear.x = 1.0 * (1.0 - abs(x_norm))
            
        ######################################

        elif (self.state == SurveilState.BARKING):
            cones = [m for m in msgs.objects if m.type == "cone"]
            if len(cones) == 0:
                self.state = SurveilState.RETURNING
                self.get_logger().info("target removed, returning...")
                return

            if(time.time() - self.last_bark_time > self.bark_delay):
                val = random.random()
                self.get_logger().info(str(val))
                sound = [p[1] for p in sounds if p[0] >= val][-1]
                self.get_logger().info("barking!")
                playsound(sound)
                self.last_bark_time = time.time()

        ######################################

        elif (self.state == SurveilState.RETURNING):
            # for now, just assume already at origin  
            self.state = SurveilState.FOLLOWING   
            self.get_logger().info("returned to origin")       
            return

        ######################################
                
        self.publisher_.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node=Surveillance()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
        main()