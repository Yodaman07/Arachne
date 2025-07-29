import socket
import struct

import cv2 as cv
# https://github.com/deirvlon/Python-TCP-Image/tree/master
ip = "127.0.0.1" # replace with computers local ip
port = 5001

#chatgpt help
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((ip, port))
    # s.sendall(b"Hello, world")
    input("Start recording")

    cap = cv.VideoCapture(0)
    while cap.isOpened():
        ret, frame = cap.read()

        # Encode frame
        _, img_encoded = cv.imencode('.jpg', frame) #encodes image as a jpeg which also reduces size
        img_bytes = img_encoded.tobytes()

        # Send length first
        s.sendall(struct.pack(">L", len(img_bytes))) # > L is big-endian format, and L is a long which has a certain size of 4 bytes - can be used for a certain size when unpacking
        s.sendall(img_bytes) # Then send image

        if cv.waitKey(1) == ord("q"):  # gets the unicode value for q
            cap.release()
            cv.destroyAllWindows()
            break
