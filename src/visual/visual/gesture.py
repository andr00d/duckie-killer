import time
import cv2
import rclpy
from rclpy.node import Node
from interfaces.msg import Object, Objects
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from collections import deque

import mediapipe as mp
from mediapipe.framework.formats import landmark_pb2
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

import torch
import numpy as np
import logging
import os
import sys
import io


def suppress_stderr():
    sys.stderr.flush()
    old_stderr = os.dup(2)
    devnull = os.open(os.devnull, os.O_WRONLY)
    os.dup2(devnull, 2)
    os.close(devnull)
    return old_stderr

def restore_stderr(old_stderr):
    sys.stderr.flush()
    os.dup2(old_stderr, 2)
    os.close(old_stderr)



class Gesture(Node):
    def __init__(self):
        super().__init__("gesture")

        # self.cap = cv2.VideoCapture(0)  # Open the webcam -- # temp fix 

        self.subscription = self.create_subscription(CompressedImage, "cam/compressed", self._cam_callback, 10)
        self.publisher_ = self.create_publisher(Objects, "objects", 10)

        # TODO: check if this can be run on GPU as well (what if laptop running it doesnt have nvida?)
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s')
        
        self.bridge = CvBridge()
        self.gesture_buffer = deque(maxlen=10)
        self.current_gesture = "Open_Palm"

        object_msg = Object()
        object_msg.x = 0.0
        object_msg.y = 0.0
        object_msg.width = 0.0
        object_msg.height = 0.0
        object_msg.type = ""
        object_msg.gesture = "Open_Palm"
        
        objects_msg = Objects()
        objects_msg.objects = [object_msg]
        self.publisher_.publish(objects_msg)
        self.get_logger().info(f'Published: {objects_msg}')




    def _cam_callback(self, msg):
        object_msg = Object()
        object_msg.x = 0.0
        object_msg.y = 0.0
        object_msg.width = 0.0
        object_msg.height = 0.0
        object_msg.type = ""
        object_msg.gesture = ""  
        
        mp_hands = mp.solutions.hands
        base_options = python.BaseOptions(model_asset_path='gesture_recognizer.task')
        options = vision.GestureRecognizerOptions(base_options=base_options)
        
        old_stderr = suppress_stderr()
        try:
            recognizer = vision.GestureRecognizer.create_from_options(options)
        finally:
            restore_stderr(old_stderr)


        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error("error with image/cv_bridge")
            return

        rgb_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)
        height, width = rgb_frame.shape[:2]

        recognition_result = recognizer.recognize(mp_image)
        detected_objects = []

        # Process gestures
        if recognition_result.gestures:
            top_gesture = recognition_result.gestures[0][0]
            self.gesture_buffer.append(top_gesture.category_name)

            most_common_gesture = max(set(self.gesture_buffer), key=self.gesture_buffer.count)

            if most_common_gesture == "Open_Palm" or self.current_gesture == "Open_Palm":
                # Publish the gesture (no coordiates)
                if most_common_gesture in ("Open_Palm", "Thumb_Up", "Thumb_Down", "Closed_Fist"):
                    object_msg = Object()
                    object_msg.x = 0.0
                    object_msg.y = 0.0
                    object_msg.width = 0.0
                    object_msg.height = 0.0
                    object_msg.type = ""
                    object_msg.gesture = most_common_gesture
                    detected_objects.append(object_msg)

                    self.current_gesture = most_common_gesture
        
        # Process detections            
        if self.current_gesture != "Open_Palm":
            
            # Use YOLOv5 to detect objects
            results = self.model(cv_image)
            for detection in results.xyxy[0].cpu().numpy():
                x1, y1, x2, y2, confidence, class_id = detection
                class_name = self.model.names[int(class_id)]

                if class_name in ["chair", "person"]:
                    object_msg = Object()
                    object_msg.x = float(x1) / width
                    object_msg.y = float(y1) / height
                    object_msg.width = float(x2-x1) / width
                    object_msg.height = float(y2-y1) / height
                    object_msg.type = class_name
                    object_msg.gesture = ""
                    detected_objects.append(object_msg)

        objects_msg = Objects()
        objects_msg.objects = detected_objects
        self.publisher_.publish(objects_msg)
        # self.get_logger().info(f'Published object: {objects_msg}')


def main(args=None):
    rclpy.init(args=args)
    node = Gesture()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
