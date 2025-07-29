import socket
import struct
import cv2 as cv
import sys
import os
import threading

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  #ai help
from arachne_control import ArachneController

# https://github.com/deirvlon/Python-TCP-Image/tree/master
ip = "127.0.0.1"  # replace with computers local ip
port = 5001


def controller_mixin(pt):  # points is a String to be parsed
    controller = ArachneController(debug=False)
    while True:
        print("IN WHILE LOOP")
        if pt[0] == "NA":  # scanning for an object
            print("TURN")
            controller.crab_walk_turn(10)
        else:
            print("CRAB WALK")
            controller.crab_walk_2(0, 30, 1)


# chatgpt help for encoding the image
with (socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s):

    s.connect((ip, port))
    input("Start recording")

    data = {"point": "NA"}
    movementThread = threading.Thread(target=controller_mixin, args=data)

    cap = cv.VideoCapture(0)

    movementThread.start()
    while cap.isOpened():
        ret, frame = cap.read()

        # Encode frame
        _, img_encoded = cv.imencode('.jpg', frame)  #encodes image as a jpeg which also reduces size
        img_bytes = img_encoded.tobytes()

        # Send length first
        s.sendall(struct.pack(">L", len(img_bytes)))
        # > L is big-endian format, and L is a long which has a certain size of 4 bytes - can be used for a certain size when unpacking
        s.sendall(img_bytes)  # Then send image

        points = s.recv(1024).decode()
        print(points)
        data["point"] = points

        if cv.waitKey(1) == ord("q"):  # gets the unicode value for q
            cap.release()
            cv.destroyAllWindows()
            movementThread.join()
            break
