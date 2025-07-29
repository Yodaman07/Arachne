import cv2 as cv
import numpy as np
from ai_edge_litert.interpreter import Interpreter
import time


# File courtesy of messing around with chatgpt
class CompactVision:
    def __init__(self):
        # Paths
        self.MODEL_PATH = "vision/model/coco_ssd_mobilenet_v1_1.0_quant_2018_06_29.tflite"
        self.LABEL_PATH = "vision/model/labelmap.txt"  # COCO Dataset

        # Load labels
        with open(self.LABEL_PATH, 'r') as f:
            self.labels = [line.strip() for line in f.readlines()]

        # Load model
        self.interpreter = Interpreter(model_path=self.MODEL_PATH)
        self.interpreter.allocate_tensors()

        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

        input_shape = self.input_details[0]['shape']
        self.height = input_shape[1]
        self.width = input_shape[2]
        self.floating_model = self.input_details[0]['dtype'] == np.float32

        self.new_frame_time = 0
        self.prev_frame_time = 0
        # self.cap = cv.VideoCapture(0)

    def process_frame(self, frame) -> np.ndarray:
        image_rgb = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        image_resized = cv.resize(image_rgb, (self.width, self.height))
        input_data = np.expand_dims(image_resized, axis=0)

        if self.floating_model:
            input_data = (np.float32(input_data) - 127.5) / 127.5
        else:
            input_data = np.uint8(input_data)

        # Run inference
        self.interpreter.set_tensor(self.input_details[0]['index'], input_data)
        self.interpreter.invoke()

        # Get outputs
        boxes = self.interpreter.get_tensor(self.output_details[0]['index'])[0]
        classes = self.interpreter.get_tensor(self.output_details[1]['index'])[0]
        scores = self.interpreter.get_tensor(self.output_details[2]['index'])[0]
        num = self.interpreter.get_tensor(self.output_details[3]['index'])[0]

        # Draw detections
        imH, imW, _ = frame.shape
        for i in range(int(num)):
            if scores[i] > 0.5:
                ymin = int(max(1, boxes[i][0] * imH))
                xmin = int(max(1, boxes[i][1] * imW))
                ymax = int(min(imH, boxes[i][2] * imH))
                xmax = int(min(imW, boxes[i][3] * imW))

                class_id = int(classes[i])
                label = self.labels[class_id] if class_id < len(self.labels) else "N/A"
                confidence = int(scores[i] * 100)

                center = (xmin + int(abs(xmin - xmax) / 2), ymin + int(abs(ymax - ymin) / 2))
                print(center)
                cv.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
                cv.putText(frame, f"{label} ({confidence}%)", (xmin, ymin - 10),
                           cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                # Show output

        self.new_frame_time = time.time()
        fps = 1 / (self.new_frame_time - self.prev_frame_time)
        self.prev_frame_time = self.new_frame_time  # Get fps

        # Display FPS on the frame (optional)
        cv.putText(frame, f"FPS: {fps}", (10, 30), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        return frame

    def tick(self, cap) -> bool:  # 1 frame

        retrieved, frame = cap.read()

        if not retrieved:
            print("Stream has likely ended")
            return False

        cv.imshow("Object Detection", self.process_frame(frame))

        if cv.waitKey(1) == ord("q"):  # ESC to quit
            return False

        return True

        # # if joystick.get_button(1): # exit program if the toggle is pressed
        # #     self.cap.release()
        # #     cv.destroyAllWindows()
