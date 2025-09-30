import struct

import cv2
import cv2 as cv
import sys
import os
import regex as re

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  #ai help

# https://github.com/deirvlon/Python-TCP-Image/tree/master
# chatgpt help for encoding the image
pattern = re.compile(r"((\d+),(\d+))")


def client_tick(cap, socket):  # returns if you should break
    ret, frame = cap.read()

    frame = cv.resize(frame, (640, 384), interpolation=cv2.INTER_AREA)

    # Encode frame
    _, img_encoded = cv.imencode('.jpg', frame)  # encodes image as a jpeg which also reduces size
    img_bytes = img_encoded.tobytes()

    # Send length first
    socket.sendall(struct.pack(">L", len(img_bytes)))
    # > L is big-endian format, and L is a long which has a certain size of 4 bytes - can be used for a certain size when unpacking
    socket.sendall(img_bytes)  # Then send image

    info = socket.recv(1024).decode()
    if info != "NA":
        result = pattern.findall(info)
        pts = []
        for pt in result:
            entry = [int(pt[1]), int(pt[2])]
            pts.append(entry)
        # print(points)
        data = pts
    else:
        data = info

    return data

    # if cv.waitKey(1) == ord("q"):  # gets the unicode value for q
    #     cap.release()
    #     cv.destroyAllWindows()
    #     return True
    #
    # return False
