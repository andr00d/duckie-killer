import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import torch
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        # Load YOLOv5 model
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s')
        
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10)
        
        self.publisher = self.create_publisher(String, 'detected_objects', 10)
        
        self.bridge = CvBridge()
        self.get_logger().info('YOLO Detector Node has been started.')
        
    def listener_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return
        
        # Perform object detection
        results = self.model(cv_image)
        
        # Process detections
        detected_objects = []
        for detection in results.xyxy[0].numpy():
            x1, y1, x2, y2, confidence, class_id = detection
            class_name = self.model.names[int(class_id)]
            detected_objects.append({
                'class': class_name,
                'confidence': confidence,
                'bbox': [x1, y1, x2, y2]
            })
        
        # Publish detected objects
        detected_objects_str = str(detected_objects)
        self.publisher.publish(String(data=detected_objects_str))
        self.get_logger().info(f'Detected Objects: {detected_objects_str}')

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

