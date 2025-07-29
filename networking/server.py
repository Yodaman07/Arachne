import socket
import cv2 as cv
import numpy as np
import struct
import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))) #ai help
from vision.vision import CompactVision



# chatgpt help
def receive_image(conn):
    # Read length first
    raw_len = conn.recv(4)  # 4 bytes - size of >L allways
    if not raw_len:
        return None
    img_len = struct.unpack(">L", raw_len)[0]  # unpack gives a tuple, but there is normally 1 element
    # Read image data
    data = b''
    while len(data) < img_len:
        # this while loop ensures you actually receive all the data that is being sent as TCP doesn't guarantee that
        packet = conn.recv(img_len - len(data))
        if not packet:
            return None
        data += packet

    # Decode image
    img_array = np.frombuffer(data, dtype=np.uint8)
    img = cv.imdecode(img_array, cv.IMREAD_COLOR)
    return img


ip = "0.0.0.0" # allows connections from anywhere (127.0.0.1 would only be local connections)
port = 5001

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((ip, port))
    s.listen()
    connection, addr = s.accept()
    c = CompactVision()

    print(f"Connected by {addr}")
    while True:
        img = receive_image(connection)

        if img is None:
            break
        else:
            cv.imshow("Stream", c.process_frame(img))
            if cv.waitKey(1) == ord("q"):  # gets the unicode value for q
                cv.destroyAllWindows()
                break
