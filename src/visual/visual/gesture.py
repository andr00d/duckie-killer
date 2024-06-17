import time
import cv2
import rclpy
from rclpy.node import Node
from interfaces.msg import Object, Objects
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from collections import deque
import queue

import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

import torch
import numpy as np
import logging
import os
import sys
import threading


class Gesture(Node):
    def __init__(self):
        super().__init__("gesture")
        self.subscription = self.create_subscription(CompressedImage, "cam/compressed", self._cam_callback, 10)
        self.publisher_ = self.create_publisher(Objects, "objects", 10)

        if torch.cuda.is_available():
            self.device = torch.device('cuda')
        elif torch.backends.mps.is_available():
            self.device = torch.device('mps')
        else:
            self.device = torch.device('cpu')
            
        self.get_logger().info(f'Using device: {self.device}')

        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5n').to(self.device)  

        self.bridge = CvBridge()
        self.gesture_buffer = deque(maxlen=10)
        self.current_gesture = "Open_Palm"

        self.frame_skip = 2  
        self.frame_count = 0

        base_options = python.BaseOptions(model_asset_path='gesture_recognizer.task')
        options = vision.GestureRecognizerOptions(base_options=base_options)
        self.recognizer = vision.GestureRecognizer.create_from_options(options)

        self.frame_queue = queue.Queue(maxsize=10)
        self.latest_frame = None
        self.latest_frame_lock = threading.Lock()

        self.display_thread = threading.Thread(target=self.display_frames)
        self.display_thread.daemon = True
        self.display_thread.start()

        self.detection_thread = threading.Thread(target=self.run_detection)
        self.detection_thread.daemon = True
        self.detection_thread.start()

        self.latest_detections = []  
        self.latest_detections_lock = threading.Lock()

    def _cam_callback(self, msg):
        self.frame_count += 1
        if self.frame_count % self.frame_skip != 0:
            return

        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is not None:
            with self.latest_frame_lock:
                self.latest_frame = frame
            threading.Thread(target=self.process_gestures, args=(frame,)).start()

    def process_gestures(self, frame):
        try:
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)
            height, width = rgb_frame.shape[:2]

            recognition_result = self.recognizer.recognize(mp_image)
            detected_objects = []

            # Process gestures
            if recognition_result.gestures:
                top_gesture = recognition_result.gestures[0][0]
                self.gesture_buffer.append(top_gesture.category_name)

                most_common_gesture = max(set(self.gesture_buffer), key=self.gesture_buffer.count)

                if most_common_gesture == "Open_Palm" or self.current_gesture == "Open_Palm":
                    # Publish the gesture (no coordinates)
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

            objects_msg = Objects()
            objects_msg.objects = detected_objects
            self.publisher_.publish(objects_msg)

            if not self.frame_queue.full():
                self.frame_queue.put(frame)
            else:
                self.get_logger().warning("Frame queue is full, dropping frame")

        except Exception as e:
            self.get_logger().error(f"Error processing frame: {e}")

    def run_detection(self):
        while True:
            time.sleep(0.25)  
            with self.latest_frame_lock:
                if self.latest_frame is not None:
                    frame = self.latest_frame.copy()
            if frame is not None:
                self.process_detections(frame)

    def process_detections(self, frame):
        try:
            height, width = frame.shape[:2]
            width_res = 160
            height_res = 120
            small_image = cv2.resize(frame, (width_res, height_res))

            results = self.model(small_image)
            detected_objects = []
            for detection in results.xyxy[0].cpu().numpy():
                x1, y1, x2, y2, confidence, class_id = detection
                class_name = self.model.names[int(class_id)]

                if class_name in ["chair", "person"]:
                    x1 = int(x1 * width / width_res)
                    y1 = int(y1 * height / height_res)
                    x2 = int(x2 * width / width_res)
                    y2 = int(y2 * height / height_res)

                    object_msg = Object()
                    object_msg.x = float(x1) / width
                    object_msg.y = float(y1) / height
                    object_msg.width = float(x2 - x1) / width
                    object_msg.height = float(y2 - y1) / height
                    object_msg.type = class_name
                    object_msg.gesture = ""
                    detected_objects.append(object_msg)

            with self.latest_detections_lock:
                self.latest_detections = detected_objects  

            objects_msg = Objects()
            objects_msg.objects = detected_objects
            self.publisher_.publish(objects_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing detections: {e}")

    def display_frames(self):
        while True:
            if not self.frame_queue.empty():
                frame = self.frame_queue.get()
                with self.latest_detections_lock:
                    if self.current_gesture != "Open_Palm":
                        for detection in self.latest_detections:
                            x1 = int(detection.x * frame.shape[1])
                            y1 = int(detection.y * frame.shape[0])
                            x2 = int((detection.x + detection.width) * frame.shape[1])
                            y2 = int((detection.y + detection.height) * frame.shape[0])
                            frame = cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                            frame = cv2.putText(frame, detection.type, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36, 255, 12), 2)

                cv2.imshow("Camera Frame", frame)
                cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = Gesture()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
