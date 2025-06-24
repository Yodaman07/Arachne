import cv2
import numpy as np
import time
from ai_edge_litert.interpreter import Interpreter

# Path to the TFLite model and label file
MODEL_PATH = "coco_ssd_mobilenet_v1_1.0_quant_2018_06_29/detect.tflite"
LABEL_PATH = "coco_ssd_mobilenet_v1_1.0_quant_2018_06_29/labelmap.txt"

# Load labels
with open(LABEL_PATH, 'r') as f:
    labels = [line.strip() for line in f.readlines()]

# Initialize interpreter
interpreter = Interpreter(model_path=MODEL_PATH)
interpreter.allocate_tensors()

# Get input and output details
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

height = input_details[0]['shape'][1]
width = input_details[0]['shape'][2]

# Start video capture (0 for USB webcam, use PiCamera module if needed)
cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Preprocess frame
    input_frame = cv2.resize(frame, (width, height))
    input_data = np.expand_dims(input_frame, axis=0)
    input_data = np.uint8(input_data)

    # Run inference
    interpreter.set_tensor(input_details[0]['index'], input_data)
    interpreter.invoke()

    # Extract output
    boxes = interpreter.get_tensor(output_details[0]['index'])[0]       # Bounding box coordinates
    classes = interpreter.get_tensor(output_details[1]['index'])[0]     # Class index
    scores = interpreter.get_tensor(output_details[2]['index'])[0]      # Confidence scores

    # Draw results
    for i in range(len(scores)):
        if scores[i] > 0.5:
            ymin, xmin, ymax, xmax = boxes[i]
            x1 = int(xmin * frame.shape[1])
            y1 = int(ymin * frame.shape[0])
            x2 = int(xmax * frame.shape[1])
            y2 = int(ymax * frame.shape[0])
            class_id = int(classes[i])
            label = labels[class_id]

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"{label}: {int(scores[i]*100)}%", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    cv2.imshow("TFLite Object Detection", frame)
    if cv2.waitKey(1) == 27:  # ESC key to exit
        break

cap.release()
cv2.destroyAllWindows()