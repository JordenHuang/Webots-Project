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
received_image = None
masked_image = None
recv_thread = None
annotated_image_to_show = None

def showImageWithCv2():
    global received_image, masked_image
    while True:
        time.sleep(0.1)
        if cv2.waitKey(1) == ord('q'):
            received_image = None
            masked_image = None
            cv2.destroyAllWindows()
        else:
            print("[en]", flush=True)
            if received_image is not None and masked_image is not None:
                print("[ok]", flush=True)
                # original
                received_image = cv2.resize(np.copy(received_image), (IMAGE_HEIGHT, IMAGE_WIDTH))
                masked_image = cv2.cvtColor(masked_image, cv2.COLOR_RGB2BGR)
                # mask
                if masked_image.ndim == 2:
                    masked_image = cv2.cvtColor(masked_image, cv2.COLOR_GRAY2BGR)
                # overlay
                combined = cv2.addWeighted(received_image, 0.8, masked_image, 0.6, 0)
                # concatenate image Vertically/Horizontally (0/1)
                verti = np.concatenate((received_image, masked_image, combined), axis=1)
                cv2.imshow("Images", verti)
            # else:
            #     if received_image is not None:
            #         received_image = np.copy(received_image)
            #         received_image = np.resize(received_image, IMAGE_RESIZE_TO)
            #         cv2.imshow("Received Image (resized)", received_image)
            #     if masked_image is not None:
            #         cv2.imshow("Masked Image", masked_image)
            if annotated_image_to_show is not None:
                print("[ok 2]", flush=True)
                cv2.imshow("Annotated", cv2.cvtColor(annotated_image_to_show, cv2.COLOR_RGB2BGR))

# TODO:
# def showImageWithMatplotlib():


def recv_image():
    global serversocket, connection, address
    global received_image, masked_image
    global annotated_image_to_show
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
        camImage = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        received_image = camImage.copy()

        if camImage is not None:
            print(f"img size: {camImage.size}")
            
            '''
            # Object detection
            imageForObjectDetection = camImage.copy()
            # Load model and run inference
            model = get_model(model_id="box-obstaces/4")
            results = model.infer(cv2.cvtColor(imageForObjectDetection, cv2.COLOR_BGR2RGB))[0]
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
            annotated_image = bounding_box_annotator.annotate(scene=imageForObjectDetection, detections=detections)
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
            annotated_image_to_show = annotated_image
            '''

            # ----------------------------------------

            # Floor segmentation
            image = predict_image(None, camImage, False)
            masked_image = image.copy()
            row_line_ok = 180 # 128 + 32
            cpDot, middleDots = getControlPoint(image, row_line_ok)
            print(f"cp dot is: {cpDot}")
            row_line_ok = 180 # 128 + 32
            row_line_back = 200 # 230
            range_accept = 10 #80
            image_middle_col = int(image.shape[1]/2)

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
    recv_thread = threading.Thread(target=recv_image, daemon=True)
    try:
        serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        serversocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        serversocket.bind((ADDRESS, PORT))
        # serversocket.bind(('172.20.10.13', 8989))
        serversocket.listen(1) # become a server socket, maximum 1 connections

        print("Waiting connection...")
        connection, address = serversocket.accept()
        print(f"Connection from: {address}")

        recv_thread.start()
        showImageWithCv2()
    except KeyboardInterrupt:
        print("Keyboard interrupted, closing server...")
    else:
        cv2.destroyAllWindows()