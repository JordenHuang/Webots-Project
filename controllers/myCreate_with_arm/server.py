import socket
import cv2
import matplotlib.pyplot as plt
import numpy as np
import threading
import time
# Roboflow
from inference import get_model
import supervision as sv

from model.my_model import *
from model.abstacle_avoidance_algorithm import *

ADDRESS = "localhost"
PORT = 8989

serversocket = None
connection = None
address = None
receivedImage = None
maskedImage = None
recvThread = None
annotatedImage = None

invokeShowImageWithCv2 = False
invokeShowImageWithMatplotlib = False
# Flag to control the loop and close the figure
matplotlibIsRunning = False

def showImageWithCv2():
    global invokeShowImageWithCv2
    global receivedImage, maskedImage
    invokeShowImageWithCv2 = True
    while True:
        time.sleep(0.1)
        if cv2.waitKey(1) == ord('q'):
            receivedImage = None
            maskedImage = None
            cv2.destroyAllWindows()
        else:
            print("[en]", flush=True)
            if receivedImage is not None and maskedImage is not None:
                print("[ok]", flush=True)
                # original
                receivedImage = cv2.resize(np.copy(receivedImage), (IMAGE_HEIGHT, IMAGE_WIDTH))
                maskedImage = cv2.cvtColor(maskedImage, cv2.COLOR_RGB2BGR)
                # mask
                if maskedImage.ndim == 2:
                    maskedImage = cv2.cvtColor(maskedImage, cv2.COLOR_GRAY2BGR)
                # overlay
                combined = cv2.addWeighted(receivedImage, 0.8, maskedImage, 0.6, 0)
                # concatenate image Vertically/Horizontally (0/1)
                verti = np.concatenate((receivedImage, maskedImage, combined), axis=1)
                cv2.imshow("Images", verti)
            # else:
            #     if received_image is not None:
            #         received_image = np.copy(received_image)
            #         received_image = np.resize(received_image, IMAGE_RESIZE_TO)
            #         cv2.imshow("Received Image (resized)", received_image)
            #     if masked_image is not None:
            #         cv2.imshow("Masked Image", masked_image)
            if annotatedImage is not None:
                print("[ok 2]", flush=True)
                cv2.imshow("Annotated", cv2.cvtColor(annotatedImage, cv2.COLOR_RGB2BGR))

# TODO:
def showImageWithMatplotlib():
    global invokeShowImageWithMatplotlib, matplotlibIsRunning
    invokeShowImageWithMatplotlib = True
    matplotlibIsRunning = True

    # Set up the figure for interactive plotting
    plt.ion() # Turn on interactive mode
    fig, axes = None, None # Initialize fig and axes
    im_input = None
    im_predict = None
    im_combine = None
    im_annotated = None

    # Try to create the figure and axes only once
    try:
        # Create a figure and axes based on what you want to show
        # Adjust layout for two subplots or one big one
        fig, axes = plt.subplots(2, 3, figsize=(15, 10))
        # fig.canvas.mpl_connect('close_event', on_close) # Connect close event

        # Initialize the image plots. Use empty arrays or None to start.
        # Ensure initial data is compatible with imshow's expectations
        im_input = axes[0, 0].imshow(np.zeros((IMAGE_HEIGHT, IMAGE_WIDTH, 3), dtype=np.uint8))
        axes[0, 0].set_title("Original")
        axes[0, 0].axis('off')
        im_predict = axes[0, 1].imshow(np.zeros((IMAGE_HEIGHT, IMAGE_WIDTH, 3), dtype=np.uint8))
        axes[0, 1].set_title("Mask")
        axes[0, 1].axis('off')
        im_combine = axes[0, 2].imshow(np.zeros((IMAGE_HEIGHT, IMAGE_WIDTH, 3), dtype=np.uint8))
        axes[0, 2].set_title("Combined")
        axes[0, 2].axis('off')

        im_annotated = axes[1, 0].imshow(np.zeros((IMAGE_HEIGHT, IMAGE_WIDTH, 3), dtype=np.uint8))
        axes[1, 0].set_title("Annotated")
        axes[1, 0].axis('off')

        # Hide unused subplots in the second row
        axes[1, 1].axis('off')
        axes[1, 2].axis('off')

        plt.tight_layout()
        plt.show(block=False) # Show without blocking, so the loop can run

    except Exception as e:
        print(f"Error initializing matplotlib figure: {e}")
        return # Exit if we can't even set up the display

    while matplotlibIsRunning:
        time.sleep(0.1) # Simulate the 0.1 second delay

        # Matplotlib doesn't have a direct 'q' key listener like cv2.waitKey
        # If you need keyboard control, you'd need a more complex event handler
        # For now, closing the window triggers `on_close` and sets `matplotlibIsRunning = False`

        # Update the main combined image
        if receivedImage is not None and maskedImage is not None:
            # --- Image Preprocessing (similar to OpenCV) ---
            # Make copies to avoid modifying original global arrays directly
            current_received_image = np.copy(receivedImage)
            current_masked_image = np.copy(maskedImage)

            current_received_image = cv2.resize(current_received_image, (IMAGE_WIDTH, IMAGE_HEIGHT)) # Note: cv2.resize expects (width, height)

            # Ensure maskedImage is BGR for cv2 operations, then convert to RGB for matplotlib
            if current_masked_image.ndim == 2: # Grayscale
                current_masked_image = cv2.cvtColor(current_masked_image, cv2.COLOR_GRAY2BGR)
            # elif current_masked_image.shape[2] == 3 and current_masked_image.dtype == np.uint8: # Assume RGB, convert to BGR for cv2
            #      current_masked_image = cv2.cvtColor(current_masked_image, cv2.COLOR_RGB2BGR)

            # Ensure maskedImage is resized to match receivedImage for concatenation
            # current_masked_image = cv2.resize(current_masked_image, (IMAGE_WIDTH, IMAGE_HEIGHT))


            # Overlay (these operations are fine with BGR)
            combined_bgr = cv2.addWeighted(current_received_image, 0.8, current_masked_image, 0.6, 0)

            # Concatenate images (these operations are fine with BGR)
            # Make sure all images have 3 channels for concatenation
            # if current_received_image.ndim == 2: current_received_image = cv2.cvtColor(current_received_image, cv2.COLOR_GRAY2BGR)
            # if current_masked_image.ndim == 2: current_masked_image = cv2.cvtColor(current_masked_image, cv2.COLOR_GRAY2BGR)

            # Convert all to RGB for matplotlib display
            # display_received_image_rgb = cv2.cvtColor(current_received_image, cv2.COLOR_BGR2RGB)
            # display_masked_image_rgb = cv2.cvtColor(current_masked_image, cv2.COLOR_BGR2RGB)
            # display_combined_rgb = cv2.cvtColor(combined_bgr, cv2.COLOR_BGR2RGB)

            # verti_rgb = np.concatenate((display_received_image_rgb, display_masked_image_rgb, display_combined_rgb), axis=1)

            # Update the image data in the plot
            if im_input: # Check if im_input was successfully initialized
                im_input.set_data(current_received_image)
            if im_predict:
                im_predict.set_data(current_masked_image)
            if im_combine:
                im_combine.set_data(combined_bgr)
        else:
            axes[0, 1].set_title("Waiting for images...")


        # Update the annotated image
        if annotatedImage is not None:
            # Assuming annotatedImage is already RGB from its source
            if im_annotated: # Check if im_annotated was successfully initialized
                im_annotated.set_data(annotatedImage)
        else:
            axes[1, 0].set_title("Waiting for images...")

        # Redraw the canvas
        fig.canvas.draw()
        fig.canvas.flush_events() # Process events to update the display

def recv_image():
    global serversocket, connection, address
    global receivedImage, maskedImage
    global annotatedImage
    while True:
        # Read image size first
        length_bytes = connection.recv(10)

        # Wait client to connect again if it's closed
        if len(length_bytes) < 10:
            print("Connection closed")
            connection.close()
            print("Waiting connection...")
            connection, address = serversocket.accept()
            print(f"Connection from: {address}")
            continue

        length = int.from_bytes(length_bytes, 'big')
        print(f"Get length: {length}")
        buf = b''
        while len(buf) < length:
            packet = connection.recv(length - len(buf))
            if not packet:
                break
            buf += packet

        print(f"Buffer len: {len(buf)}")
        buf = np.frombuffer(buf, dtype='uint8')
        print(f"Buffer: {buf}")
        cam_image = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        receivedImage = cam_image.copy()

        if cam_image is not None:
            print(f"img size: {cam_image.size}")
            
            # '''
            # Object detection
            image_for_object_detection = cam_image.copy()
            # Load model and run inference
            model = get_model(model_id="box-obstaces/4")
            results = model.infer(cv2.cvtColor(image_for_object_detection, cv2.COLOR_BGR2RGB))[0]
            detections = sv.Detections.from_inference(results)
            # Get the bounding box closest to bottom-left
            bottom_left_detection = None
            max_distance = float('-inf')

            for i, box in enumerate(detections.xyxy):
                x_min, y_min, x_max, y_max = box

                # Euclidean distance to bottom-left corner
                if y_max > max_distance:
                    max_distance = y_max
                    bottom_left_detection = (x_min, y_min, x_max, y_max)

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
            annotated_image = bounding_box_annotator.annotate(scene=image_for_object_detection, detections=detections)
            annotated_image = label_annotator.annotate(scene=annotated_image, detections=detections)

            # Use matplotlib to show annotated image
            # def on_click(event):
            #     if event.xdata is not None and event.ydata is not None:
            #         x = int(event.xdata)
            #         y = int(event.ydata)
            #         print(f"Clicked at x: {x}, y: {y}")
            # fig, ax = plt.subplots()
            # ax.imshow(annotated_image)  # Convert to RGB for correct color display
            # ax.set_title("Click on image to get (x, y)")
            # fig.canvas.mpl_connect('button_press_event', on_click)
            # plt.show()

            # Use opencv to show annotated image
            annotatedImage = annotated_image
            # '''

            # ----------------------------------------

            # Floor segmentation
            masked_image = predict_image(None, cam_image, False)
            maskedImage = masked_image.copy()
            row_line_ok = 180 # 128 + 32
            cpDot, middleDots = getControlPoint(masked_image, row_line_ok)
            print(f"cp dot is: {cpDot}")
            row_line_ok = 180 # 128 + 32
            row_line_back = 200 # 230
            range_accept = 10 #80
            image_middle_col = int(masked_image.shape[1]/2)

            cmd = ""
            if cpDot:
                if cpDot[1][1] > row_line_back:
                    cmd = "back"
                # should go left more
                elif cpDot[1][0] < (image_middle_col-range_accept):
                    cmd = "left"
                # should go right more
                elif cpDot[1][0] > (image_middle_col+range_accept):
                    cmd = "right"
                else:
                    cmd = "front"
            else:
                cmd = "back"
            print("cmd to send:", cmd)
            print(f"cmd length: {len(cmd)}")
            cmd_length = len(cmd).to_bytes(10, 'big')
            connection.sendall(cmd_length + cmd.encode())


if __name__ == "__main__":
    recvThread = threading.Thread(target=recv_image, daemon=True)
    try:
        serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        serversocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        serversocket.bind((ADDRESS, PORT))
        # serversocket.bind(('172.20.10.13', 8989))
        serversocket.listen(1) # become a server socket, maximum 1 connections

        print("Waiting connection...")
        connection, address = serversocket.accept()
        print(f"Connection from: {address}")

        recvThread.start()
        # showImageWithCv2()
        showImageWithMatplotlib()
    except KeyboardInterrupt:
        print("Keyboard interrupted, closing server...")
        matplotlibIsRunning = False
    else:
        if invokeShowImageWithCv2:
            cv2.destroyAllWindows()