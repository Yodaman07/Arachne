from typing import Any

import cv2 as cv
from ultralytics import YOLO


class YoloVision:
    def __init__(self):
        self.model = YOLO("vision/yolov8n.pt")
        # look at phyz vision - check for bounding box area (bigger is better) also  average position in a circle around
    def process_frame(self, frame) -> list[Any]:
        global center_x, center_y
        results = self.model(frame, conf=0.5)[0]
        print(results)
        # print(results) or print(box) for good data
        if len(results.boxes) > 0:  # the result obj will have a box list which distinguishes how many items there are
            for box in results.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # maps the box to x & y coordinates
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2
                id = box.cls[0].item()
                name = results.names[id]
                # Draw detection (optional)
                cv.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv.circle(frame, (center_x, center_y), 4, (0, 0, 255), -1)
                cv.putText(frame, f"{name}", (center_x, center_y), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 3)


            return [center_x, center_y] # only returning the first one found

        return ["NA", "NA"]

    def tick(self, cap) -> bool:  # 1 frame

        retrieved, frame = cap.read()

        if not retrieved:
            print("Stream has likely ended")
            return False

        self.process_frame(frame)

        if cv.waitKey(1) == ord("q"):  # ESC to quit
            return False

        return True
