#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# for now a simple catch node for in the case extra processing is needed
class Rover(Node):
    def __init__(self):
        super().__init__("Rover")
        self.subscriber = self.create_subscription(Twist, 'rbt_vel', self._rover_callback, 10)
        self.publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)
        
    def _rover_callback(self, msg):
        if(not (msg.linear.x == 0.0 and msg.angular.y == 0.0 and msg.angular.z == 0.0)):
            
            rbt_msg = Twist()
            rbt_msg.linear.x = msg.linear.y
            rbt_msg.angular.y = msg.angular.y
            rbt_msg.angular.z = msg.angular.z
            
            self.publisher_.publish(rbt_msg)

def main(args=None):
    rclpy.init(args=args)
    node=Rover()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
        main()