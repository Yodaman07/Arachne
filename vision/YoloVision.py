from typing import Any
import cv2 as cv
from ultralytics import YOLO


class YoloVision:
    def __init__(self):
        self.model = YOLO("vision/yolov8n.pt")
        self.obj_filter = ["bottle"]  # filter so only these objects get sent to the client and are highlighted (if left empty, anything will work)
        self.to_send = []
        self.last_sent = []
        self.tolerance = 5  # 10%
        # look at phyz vision - check for bounding box area (bigger is better) also  average position in a circle around

    def process_frame(self, frame) -> list[Any]:
        self.to_send = []
        results = self.model(frame, conf=0.5)[0]
        # print(results)
        # print(results) or print(box) for good data
        if len(results.boxes) > 0:  # the result obj will have a box list which distinguishes how many items there are
            for box in results.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # maps the box to x & y coordinates
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2
                id = box.cls[0].item()
                name = results.names[id]
                if self.obj_filter == [] or name in self.obj_filter:
                    # Draw detection & send data, either on items in filter, or on everything
                    cv.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv.circle(frame, (center_x, center_y), 4, (0, 0, 255), -1)
                    cv.putText(frame, f"{name}", (center_x, center_y), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 3)
                    self.to_send.append([center_x, center_y, name, (x1, y1), (x2, y2)])

                    # TODO: Write alg to refine which objects are chosen in the list

        self.verify()
        self.last_sent = self.to_send
        return self.to_send  # list of all detected objects that fall under the filter

    def verify(self):
        if not self.to_send: return

        # print(self.to_send)
        # for c,i in enumerate(self.to_send):
        #     print(self.area(c))

        biggestIndex = 0
        for index, res in enumerate(self.to_send):
            if self.area(index) > self.area(biggestIndex):
                biggestIndex = index
        self.to_send = [self.to_send[biggestIndex]]
        print(self.to_send)

        # for item in self.to_send: #measures based on tolerance
        #     for last_item in self.last_sent:
        #         diff_x = abs(last_item[0] - item[0])
        #         diff_y = abs(last_item[1] - item[1])
        #
        #         if (diff_x / item[0]) * 100 <= self.tolerance and (diff_y / item[1]) * 100 <= self.tolerance:
        #             print("okay")
        #         else:
        #             print(f"x:{(diff_x / item[0]) * 100}, y:{(diff_y / item[1]) * 100}")

    def area(self, i):
        y_dist = abs(self.to_send[i][3][1] - self.to_send[i][4][1])
        x_dist = abs(self.to_send[i][3][0] - self.to_send[i][4][0])
        return x_dist * y_dist

    def tick(self, cap) -> bool:  # 1 frame

        retrieved, frame = cap.read()

        if not retrieved:
            print("Stream has likely ended")
            return False

        self.process_frame(frame)

        if cv.waitKey(1) == ord("q"):  # ESC to quit
            return False

        return True
