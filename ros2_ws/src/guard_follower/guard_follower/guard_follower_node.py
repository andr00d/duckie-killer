import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import json

class GuardFollowerNode(Node):
    def __init__(self):
        super().__init__('guard_follower')
        
        self.subscription = self.create_subscription(
            String,
            'guard_follower',
            self.listener_callback,
            10)
        
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Guard Follower Node has been started.')

    def listener_callback(self, msg):
        person_detected = False
        frame_width = 640  # Example frame width, adjust as necessary
        frame_height = 480  # Example frame height, adjust as necessary
        frame_center_x = frame_width / 2
        
        json_data = msg.data.strip('\"')
        detections = json.loads(json_data.replace("'", "\""))

        #detections = json.loads(msg.data)
        for detection in detections:
            x1, y1, x2, y2, confidence, class_id = detection['bbox'][0], detection['bbox'][1], detection['bbox'][2], detection['bbox'][3], detection['confidence'], detection['class']
            if class_id == 'person':  # class_id 0 is 'person' in COCO
                person_detected = True
                person_center_x = (x1 + x2) / 2
                person_center_y = (y1 + y2) / 2
                person_width = x2 - x1
                person_height = y2 - y1

                # Determine movement direction based on the person's position
                if person_width > frame_width * 0.5 or person_height > frame_height * 0.5:
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
            twist.linear.x = 0.5
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

        self.publisher.publish(twist)
        self.get_logger().info(f'Moving robot: {direction}')

def main(args=None):
    rclpy.init(args=args)
    node = GuardFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

