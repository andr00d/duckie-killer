#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from interfaces.msg import Object, Objects
import numpy as np

class Guard(Node):
    def __init__(self):
        super().__init__("Gaurd")
        self.subscriber = self.create_subscription(Objects, 'state_guard', self._guard_callback, 10)
        self.publisher_ = self.create_publisher(Twist, "/rbt_vel", 10)
        self.last_person_pos = np.array((0.5, 0.5))
        self.MAX_BBOX_AREA = 0.4

    def _guard_callback(self, msg):
        if len(msg.objects) > 0 and msg.objects[0].gesture == "clear":
            self.last_person_pos = np.array((0.5, 0.5))
            return

        people = [m for m in msg.objects if m.type == "person"]
        if len(people) == 0:
            return

        # follow person nearest to last position. 
        person = min(people, key=lambda m: np.linalg.norm(np.array((m.x, m.y)) - self.last_person_pos))
        self.last_person_pos = np.array((person.x, person.y))
        self.get_logger().info("following person with following info:")
        self.get_logger().info("x:{:.3f}\ty:{:.3f}\tw:{:.3f}\th:{:.3f}".format(person.x, person.y, person.width, person.height))

        bb_area = person.width * person.height
        if bb_area < self.MAX_BBOX_AREA:
            twist_msg = Twist() 
            center = person.x + person.width/2
            x_norm = -2 * (center - 0.5)
            twist_msg.angular.z = x_norm
            # TODO: rotation can be a bit better
            twist_msg.linear.x = 1.0 * (1.0 - abs(x_norm)**2)
            self.publisher_.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node=Guard()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
        main()