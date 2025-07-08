from inference import get_model
import supervision as sv
import cv2
import matplotlib.pyplot as plt

# Load image
image_file = "../myCreate_capture_image/images/cl_001.png"
image = cv2.imread(image_file)
image_height, image_width = image.shape[:2]

# Load model and run inference
model = get_model(model_id="box-obstaces/4")
results = model.infer(image)[0]
detections = sv.Detections.from_inference(results)

# Get the bounding box closest to bottom-left
bottom_left_detection = None
max_distance = float('-inf')

for i, box in enumerate(detections.xyxy):
    x_min, y_min, x_max, y_max = box
    box_center_x = (x_min + x_max) / 2
    box_center_y = (y_min + y_max) / 2

    # Euclidean distance to bottom-left corner
    if box_center_y > max_distance:
        max_distance = box_center_y
        bottom_left_detection = (x_min, y_min, x_max, y_max)
    # distance = ((box_center_x - 0)**2 + (box_center_y - image_height)**2)**0.5
    # if distance < min_distance:
    #     min_distance = distance
    #     bottom_left_detection = (x_min, y_min, x_max, y_max)

if bottom_left_detection is not None:
    x_min, y_min, x_max, y_max = bottom_left_detection
    width = x_max - x_min
    height = y_max - y_min
    print(f"x_min: {x_min}, x_max: {x_max}, y_min: {y_min}, y_max: {y_max}")
    print(f"Bottom-left object width: {width} px, height: {height} px")
else:
    print("No object detected.")

# Optional: draw annotations
bounding_box_annotator = sv.BoxAnnotator()
label_annotator = sv.LabelAnnotator()
annotated_image = bounding_box_annotator.annotate(scene=image, detections=detections)
annotated_image = label_annotator.annotate(scene=annotated_image, detections=detections)
# sv.plot_image(annotated_image)

# Use matplotlib to show annotated image
def on_click(event):
    if event.xdata is not None and event.ydata is not None:
        x = int(event.xdata)
        y = int(event.ydata)
        print(f"Clicked at x: {x}, y: {y}")
fig, ax = plt.subplots()
ax.imshow(cv2.cvtColor(annotated_image, cv2.COLOR_BGR2RGB))  # Convert to RGB for correct color display
ax.set_title("Click on image to get (x, y)")
fig.canvas.mpl_connect('button_press_event', on_click)
plt.show()