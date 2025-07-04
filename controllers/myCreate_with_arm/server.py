import socket
import cv2
import matplotlib.pyplot as plt
import numpy as np
import threading
import time

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

def showImageWithCv2():
    global received_image, masked_image
    while True:
        time.sleep(0.1)
        if cv2.waitKey(1) == ord('q'):
            received_image = None
            masked_image = None
            cv2.destroyAllWindows()
        else:
            if received_image is not None and masked_image is not None:
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

def recv_image():
    global serversocket, connection, address
    global received_image, masked_image
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
        received_image = camImage

        if camImage is not None:
            print(f"img size: {camImage.size}")

            image = predict_image(None, camImage, False)
            masked_image = image
            row_line_ok = 180 # 128 + 32
            cpDot, middleDots = getControlPoint(image, row_line_ok)
            print(f"cp dot is: {cpDot}")

            # Some parameters
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