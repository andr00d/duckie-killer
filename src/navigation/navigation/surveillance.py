#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from ament_index_python.packages import get_package_share_directory
from tf_transformations import euler_from_quaternion
from interfaces.msg import Object, Objects
from nav_msgs.msg import Odometry
import numpy as np
from playsound import playsound
import random
import time
import os

class SurveilState:
    STARTING = 0
    PATROLLING = 1
    FOLLOWING = 2
    BARKING = 3
    RETURNING = 4

pkg_dir = get_package_share_directory("navigation")
sounds = [
    # [%chance, pathfile], if rand < %chance, lowest chance file gets played. 
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
                ("move_radius", rclpy.Parameter.Type.DOUBLE),
            ],
        )
        import os

        self.subscriber = self.create_subscription(Objects, 'state_surveillance', self._surveillance_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odometry/wheels', self._odom_callback, 10)
        self.publisher_ = self.create_publisher(Twist, "/rbt_vel", 10)

        self.bark_delay = (self.get_parameter("bark_delay").get_parameter_value().integer_value)
        self.move_radius = (self.get_parameter("move_radius").get_parameter_value().double_value)
        self.last_bark_time = time.time()
        self.MAX_BBOX_AREA = 0.1
        
        self.state = SurveilState.STARTING
        self.round_index = 0
        self.last_odom = Odometry()
        self.center_pos = Pose()
        
    def _odom_callback(self, msg):
        self.last_odom = msg

    def _drive_to_pos(self, msg, current_pos, goal_pos):
            dist_x = goal_pos.position.x - current_pos.position.x
            dist_y = goal_pos.position.y - current_pos.position.y
            distance = np.hypot(dist_x, dist_y)

            diff_angle = np.arctan2(dist_y, dist_x)

            quaternion = [current_pos.orientation.x, current_pos.orientation.y, 
                          current_pos.orientation.z, current_pos.orientation.w]
            roll, pitch, yaw = euler_from_quaternion(quaternion)

            if(abs(yaw - diff_angle) > .2):
                # minimize rotation
                rot_dir = (diff_angle - yaw) / abs((diff_angle - yaw))
                if abs(yaw - diff_angle) < 1.9*np.pi:
                    if abs(yaw - diff_angle) > 1.5*np.pi:
                        rot_dir = -rot_dir
                    msg.angular.z = 0.5 * rot_dir
                    return msg, False

            if(distance < .15):
                return msg, True

            msg.linear.x = 0.6
            return msg, False

    def _surveillance_callback(self, msgs):
        if len(msgs.objects) > 0 and msgs.objects[0].gesture == "clear":
            self.round_index = 0
            self.state = SurveilState.STARTING
            return

        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0

        ######################################

        if(self.state == SurveilState.STARTING):
            self.center_pos = self.last_odom.pose.pose
            
            self.corners = []
            for offset in [[self.move_radius,  self.move_radius], 
                           [-self.move_radius, self.move_radius], 
                           [-self.move_radius, -self.move_radius], 
                           [self.move_radius,  -self.move_radius]]:
                corner = Pose()
                corner.position.x = self.center_pos.position.x + offset[0]
                corner.position.y = self.center_pos.position.y + offset[1]
                self.corners.append(corner)
                
            self.get_logger().info("current position:")
            self.get_logger().info("{} - {}".format(self.center_pos.position.x, self.center_pos.position.y))
            self.get_logger().info("calculated positions for corners:")
            for i in range(4):
                self.get_logger().info("{} - {}".format(self.corners[i].position.x, self.corners[i].position.y))

            self.state = SurveilState.PATROLLING
            self.get_logger().info("starting with surveillance")

        ######################################

        if(self.state == SurveilState.PATROLLING):
            people = [m for m in msgs.objects if m.type == "person"]
            if len(people) > 0:
                self.round_index = 0
                self.state = SurveilState.FOLLOWING
                self.get_logger().info("found target")
                return

            twist_msg, arrived = self._drive_to_pos(twist_msg, self.last_odom.pose.pose, self.corners[self.round_index])
            if arrived:
                self.round_index = (self.round_index + 1) % 4

        ######################################

        elif (self.state == SurveilState.FOLLOWING):
            people = [m for m in msgs.objects if m.type == "person"]
            if len(people) == 0:
                self.state = SurveilState.RETURNING
                self.get_logger().info("lost target, returning...")
                return

            person = max(people, key=lambda m: m.width * m.height)
            bb_area = person.width * person.height
            if bb_area > self.MAX_BBOX_AREA:
                self.state = SurveilState.BARKING
                self.last_bark_time = time.time() - 3
                self.get_logger().info("arrive at target, barking.")
                # self.last_bark_time = time.time()
                return

            center = x + w / 2.0
            x_norm = -2 * (center - 0.5)
            twist_msg.angular.z = x_norm
            twist_msg.linear.x = 1.0 * (1.0 - abs(x_norm))
            
        ######################################

        elif (self.state == SurveilState.BARKING):
            people = [m for m in msgs.objects if m.type == "person"]
            if len(people) == 0:
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
            twist_msg, arrived = self._drive_to_pos(twist_msg, self.last_odom.pose.pose,  self.center_pos)
            if arrived:
                self.state = SurveilState.PATROLLING   
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