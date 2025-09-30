import cv2.typing
from ultralytics import YOLO
import cv2 as cv
# from vision.YoloVision import YoloVision
from YoloVision import YoloVision
# Load a pretrained YOLO11n model
y = YoloVision()

cam = cv.VideoCapture(0)

if not cam.isOpened():
    print("Unable to access camera")  # kill the program if the camera is not accessed
    cam.release()
    exit()

while True:
    retrieved, frame = cam.read()

    if not retrieved:
        print("Stream has likely ended")
        break

    frame = cv.resize(frame, (640, 384), interpolation=cv2.INTER_AREA)
    a = y.process_frame(frame)

    cv.imshow("stream", frame)
    if cv.waitKey(1) == ord("q"):  # gets the unicode value for q
        break

cam.release()
cv.destroyAllWindows()
