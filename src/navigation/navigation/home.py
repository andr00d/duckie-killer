#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from interfaces.msg import Object, Objects


class Home(Node):
    def __init__(self):
        super().__init__("Home")
        self.subscriber = self.create_subscription(Objects, 'state_home', self._home_callback, 10)
        self.publisher_ = self.create_publisher(Twist, "/rbt_vel", 10)
        
    def _home_callback(self, msg):
        #do something with input, then send vel to robot.
        twist_msg = Twist()
        twist_msg.linear.x = 1
        twist_msg.angular.y = 0
        twist_msg.angular.z = 0

        self.publisher_.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node=Home()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
        main()