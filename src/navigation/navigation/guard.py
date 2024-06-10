#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from interfaces.msg import Object, Objects


class Guard(Node):
    def __init__(self):
        super().__init__("Gaurd")
        self.subscriber = self.create_subscription(Objects, 'state_guard', self._guard_callback, 10)
        self.publisher_ = self.create_publisher(Twist, "/rbt_vel", 10)
        
    def _guard_callback(self, msg):
        person_detected = False
        frame_width = 640  
        frame_height = 480  
        frame_center_x = frame_width / 2
        
        for detection in msg.objects:
            x0,y0,width,height,class_id,gest = detection.x,detection.y,detection.width, detection.height, detection.type, detection.gesture
            if class_id == 'person':  
                person_detected = True
                person_center_x = x0 + (width / 2)


                # Determine movement direction based on the person's position
                if width*height > frame_width * frame_height * 0.6:
                    self.move_robot("stop")
                elif person_center_x < frame_center_x - 50:
                    self.move_robot("left")
                elif person_center_x > frame_center_x + 50:
                    self.move_robot("right")
                else:
                    self.move_robot("forward")

                # Stop processing after the first person
                break

        if not person_detected:
            self.move_robot("stop")

    def move_robot(self, direction):
        twist = Twist()
        if direction == "stop":
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        elif direction == "left":
            twist.linear.x = 0.0
            twist.angular.z = 0.5
        elif direction == "right":
            twist.linear.x = 0.0
            twist.angular.z = -0.5
        elif direction == "forward":
            twist.linear.x = 0.5
            twist.angular.z = 0.0

        self.publisher_.publish(twist)
        self.get_logger().info(f'Moving robot: {direction}')

def main(args=None):
    rclpy.init(args=args)
    node=Guard()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
        main()