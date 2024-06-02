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

    # run inference on the current frame in a separate thread
    future_inference = executor.submit(CLIENT.infer, frame, model_id="barrel-ctlrd/1")

    # load the results into the supervision Detections api
    results = future_inference.result()
    detections = sv.Detections.from_inference(results)

    # Get the bounding box coordinates
    for detection in detections:
        bbox = detection[0]
        x = bbox[0]
        y = bbox[1]
        width = bbox[2] - bbox[0]
        height = bbox[3] - bbox[1]
        class_label = detection[5]['class_name'] 
    
        print(f"x: {x}, y: {y}, width: {width}, height: {height}, class: {class_label}")

    # annotate the image with our inference results in a separate thread
    future_annotation = executor.submit(bounding_box_annotator.annotate, scene=frame, detections=detections)
    annotated_image = future_annotation.result()
    annotated_image = label_annotator.annotate(scene=annotated_image, detections=detections)

    # Display the resulting frame
    cv2.imshow('Webcam Feed', annotated_image)

    # Break the loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
