import time

import cv2
import rclpy
from rclpy.node import Node
from interfaces.msg import Object, Objects
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from collections import deque

import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

from sensor_msgs.msg import CompressedImage


# simple setup for gesture node
class Gesture(Node):
    def __init__(self):
        super().__init__("gesture")

        # self.cap = cv2.VideoCapture(0)  # Open the webcam -- # temp fix 

        self.subscription = self.create_subscription(CompressedImage, "cam/compressed", self._cam_callback, 10)
        self.publisher_ = self.create_publisher(Objects, "objects", 10)

        # 
        # self.publisher_home = self.create_publisher(String, 'home_mode', 10)
        # self.publisher_surveillance = self.create_publisher(String, 'surveillance_mode', 10)
        # self.publisher_guard = self.create_publisher(String, 'guard_mode', 10)
        
        self.bridge = CvBridge()
        self.gesture_buffer = deque(maxlen=10)

        # self.home_mode_active = False
        # self.surveillance_mode_active = False
        # self.guard_mode_active = False

        # self.deactivate_all_modes()

        # self.timer = self.create_timer(.2, self.gesture_callback)
        self.publisher_ = self.create_publisher(Objects, "objects", 10)

    def _cam_callback(self, msg):
        object_msg = Object()
        object_msg.x = 0.0
        object_msg.y = 0.0
        object_msg.width = 0.0
        object_msg.height = 0.0
        object_msg.type = ""
        object_msg.gesture = ""  
        self.gesture_callback(object_msg)
        mp_hands = mp.solutions.hands
        base_options = python.BaseOptions(model_asset_path='gesture_recognizer.task')
        options = vision.GestureRecognizerOptions(base_options=base_options)
        recognizer = vision.GestureRecognizer.create_from_options(options)

        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            return

        rgb_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)

        recognition_result = recognizer.recognize(mp_image)

        if recognition_result.gestures:
            top_gesture = recognition_result.gestures[0][0]
            self.gesture_buffer.append(top_gesture.category_name)

            most_common_gesture = max(set(self.gesture_buffer), key=self.gesture_buffer.count)

            if most_common_gesture == "Open_Palm":
                object_msg = Object()
                object_msg.x = 0.0
                object_msg.y = 0.0
                object_msg.width = 0.0
                object_msg.height = 0.0
                object_msg.type = ""
                object_msg.gesture = most_common_gesture
                self.gesture_callback(object_msg)
                # self.deactivate_all_modes()
            # elif not self.home_mode_active and not self.surveillance_mode_active and not self.guard_mode_active:
            #     if most_common_gesture == "Thumb_Up":
            #         self.activate_home_mode()
            #     elif most_common_gesture == "Thumb_Down":
            #         self.activate_surveillance_mode()
            #     elif most_common_gesture == "Closed_Fist":
            #         self.activate_guard_mode()

        # self.get_logger().info(self.get_home_mode_status())
        # self.get_logger().info(self.get_surveillance_mode_status())
        # self.get_logger().info(self.get_guard_mode_status())

    # def deactivate_all_modes(self):
    #     self.publish_message(self.publisher_home, "Home Mode Deactivated")
    #     self.publish_message(self.publisher_surveillance, "Surveillance Mode Deactivated")
    #     self.publish_message(self.publisher_guard, "Guard Mode Deactivated")
    #     self.home_mode_active = False
    #     self.surveillance_mode_active = False
    #     self.guard_mode_active = False

    # def activate_home_mode(self):
    #     self.publish_message(self.publisher_home, "Activating Home Mode")
    #     self.home_mode_active = True

    # def activate_surveillance_mode(self):
    #     self.publish_message(self.publisher_surveillance, "Activating Surveillance Mode")
    #     self.surveillance_mode_active = True

    # def activate_guard_mode(self):
    #     self.publish_message(self.publisher_guard, "Activating Guard Mode")
    #     self.guard_mode_active = True

    # def publish_message(self, publisher, message):
    #     msg = String()
    #     msg.data = message
    #     publisher.publish(msg)
    #     self.get_logger().info(f'Published: {message}')

    # def get_home_mode_status(self):
    #     status = "activated" if self.home_mode_active else "deactivated"
    #     return f'Home Mode is {status}'

    # def get_surveillance_mode_status(self):
    #     status = "activated" if self.surveillance_mode_active else "deactivated"
    #     return f'Surveillance Mode is {status}'

    # def get_guard_mode_status(self):
    #     status = "activated" if self.guard_mode_active else "deactivated"
    #     return f'Guard Mode is {status}'


    def gesture_callback(self, object_msg):
        objects_msg = Objects()
        objects_msg.objects = [object_msg]
        self.publisher_.publish(objects_msg)
        self.get_logger().info(objects_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Gesture()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
