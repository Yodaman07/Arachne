import cv2 as cv


cam = cv.VideoCapture(0)

if not cam.isOpened():
    print("Unable to access camera")  # kill the program if the camera is not accessed
    cam.release()
    exit()

while True:
    retrieved, frame = cam.read()
    # https://stackoverflow.com/questions/19062875/how-to-get-the-number-of-channels-from-an-image-in-opencv-2

    if not retrieved:
        print("Stream has likely ended")
        break
    print(frame.shape)

    cv.imshow("stream", frame)
    # https://stackoverflow.com/questions/5217519/what-does-opencvs-cvwaitkey-function-do <-- how waitKey works
    if cv.waitKey(1) == ord("q"):  # gets the unicode value for q
        break

cam.release()

cv.destroyAllWindows()
