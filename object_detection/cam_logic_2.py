import cv2
import torch

# Load YOLOv5 model
model = torch.hub.load('ultralytics/yolov5', 'yolov5s')


cap = cv2.VideoCapture(0)

# Function to send commands to the robotic platform (to be implemented)
def move_robot(direction):
    if direction == "left":
        print("Moving left")
        # Add code to move robot left
    elif direction == "right":
        print("Moving right")
        # Add code to move robot right
    elif direction == "forward":
        print("Moving forward")
        # Add code to move robot forward
    elif direction == "stop":
        print("Stopping")
        # Add code to stop the robot

while True:
    ret, frame = cap.read()
    if not ret:
        break

    height, width, channels = frame.shape
    frame_center_x = width // 2
    frame_center_y = height // 2

    results = model(frame)

    # Process detections
    person_detected = False
    for detection in results.xyxy[0].numpy():
        x1, y1, x2, y2, confidence, class_id = detection
        if msg =="home":
			if int(class_id) == 56:  # class_id 0 is 'person' in COCO
				person_detected = True
				person_center_x = (x1 + x2) / 2
				person_center_y = (y1 + y2) / 2
				person_width = x2 - x1
				person_height = y2 - y1

				label = f"home {confidence:.2f}"
				color = (0, 255, 0)
				cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
				cv2.putText(frame, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

				
				if person_width*person_height > width*height * 0.6: 
					move_robot("stop")
					color = ( 0, 0 , 255)
					cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
					cv2.putText(frame, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
				elif person_center_x < frame_center_x - 50:
					move_robot("left")
				elif person_center_x > frame_center_x + 50:
					move_robot("right")
				else:
					move_robot("forward")

				break
         elif msg =="person":
			if int(class_id) == 0:  # class_id 0 is 'person' in COCO
				person_detected = True
				person_center_x = (x1 + x2) / 2
				person_center_y = (y1 + y2) / 2
				person_width = x2 - x1
				person_height = y2 - y1

				label = f"Person {confidence:.2f}"
				color = (0, 255, 0)
				cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
				cv2.putText(frame, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

				
				if person_width*person_height > width*height * 0.6: 
					move_robot("stop")
					color = ( 0, 0 , 255)
					cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
					cv2.putText(frame, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
				elif person_center_x < frame_center_x - 50:
					move_robot("left")
				elif person_center_x > frame_center_x + 50:
					move_robot("right")
				else:
					move_robot("forward")

				break

    if not person_detected:
        move_robot("stop")

    
    cv2.imshow('Frame', frame)

    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


cap.release()
cv2.destroyAllWindows()

