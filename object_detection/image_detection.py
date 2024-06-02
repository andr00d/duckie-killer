# from inference import get_model
import supervision as sv
import cv2
from inference_sdk import InferenceHTTPClient

CLIENT = InferenceHTTPClient(
    api_url="https://detect.roboflow.com",
    api_key="vQkhB3BEwTsd5ouL2ifv"
)

# define the image to use for inference
image_file = "Test_5.jpg"           # image can be a url, a numpy array, a PIL image, etc.
image = cv2.imread(image_file)

# Load a pre-trained yolov5 model and Run inference on our chosen image
results = CLIENT.infer(image_file, model_id="people-detection-general/5")

# load the results into the supervision Detections api
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

# create supervision annotators
bounding_box_annotator = sv.BoundingBoxAnnotator()
label_annotator = sv.LabelAnnotator()

# annotate the image with our inference results
annotated_image = bounding_box_annotator.annotate(
    scene=image, detections=detections)
annotated_image = label_annotator.annotate(
    scene=annotated_image, detections=detections)

# display the image
sv.plot_image(annotated_image)