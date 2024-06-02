import cv2
import supervision as sv
from inference_sdk import InferenceHTTPClient
import concurrent.futures

CLIENT = InferenceHTTPClient(
    api_url="https://detect.roboflow.com",
    api_key="vQkhB3BEwTsd5ouL2ifv"
)

# create supervision annotators
bounding_box_annotator = sv.BoundingBoxAnnotator()
label_annotator = sv.LabelAnnotator()

# Access the webcam
cap = cv2.VideoCapture(0)  # 0 is the default webcam

# Exit if the webcam is not opened
if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

# Create a ThreadPoolExecutor
executor = concurrent.futures.ThreadPoolExecutor(max_workers=2)

# Run the webcam feed
while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Check if frame is read correctly
    if not ret:
        print("Error: Failed to capture image.")
        break

    # run inference on the current frame with both models in separate threads
    future_inference1 = executor.submit(CLIENT.infer, frame, model_id="people-detection-general/5")
    future_inference2 = executor.submit(CLIENT.infer, frame, model_id="barrel-ctlrd/1")

    # load the results into the supervision Detections api
    results1 = future_inference1.result()
    results2 = future_inference2.result()
    detections1 = sv.Detections.from_inference(results1)
    detections2 = sv.Detections.from_inference(results2)

    # Get the bounding box coordinates for the first model
    for detection1 in detections1:
        bbox1 = detection1[0]
        x1 = bbox1[0]
        y1 = bbox1[1]
        width1 = bbox1[2] - bbox1[0]
        height1 = bbox1[3] - bbox1[1]
        class_label1 = detection1[5]['class_name'] 
    
        print(f"x: {x1}, y: {y1}, width: {width1}, height: {height1}, class: {class_label1}")

    # Get the bounding box coordinates for the second model
    for detection2 in detections2:
        bbox2 = detection2[0]
        x2 = bbox2[0]
        y2 = bbox2[1]
        width2 = bbox2[2] - bbox2[0]
        height2 = bbox2[3] - bbox2[1]
        class_label2 = detection2[5]['class_name'] 
    
        print(f"x: {x2}, y: {y2}, width: {width2}, height: {height2}, class: {class_label2}")

    # annotate the image with our inference results for the first model in a separate thread
    future_annotation1 = executor.submit(bounding_box_annotator.annotate, scene=frame, detections=detections1)
    annotated_image1 = future_annotation1.result()
    annotated_image1 = label_annotator.annotate(scene=annotated_image1, detections=detections1)

    # annotate the image with our inference results for the second model in a separate thread
    future_annotation2 = executor.submit(bounding_box_annotator.annotate, scene=frame, detections=detections2)
    annotated_image2 = future_annotation2.result()
    annotated_image2 = label_annotator.annotate(scene=annotated_image2, detections=detections2)

    # Display the resulting frame
    cv2.imshow('Webcam Feed', annotated_image1)
    cv2.imshow('Webcam Feed', annotated_image2)

    # Break the loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()