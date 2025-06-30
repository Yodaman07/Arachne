import cv2
import numpy as np
from ai_edge_litert.interpreter import Interpreter
import time

# File courtesy of messing around with chatgpt

# Paths
MODEL_PATH = "vision/model/coco_ssd_mobilenet_v1_1.0_quant_2018_06_29.tflite"
LABEL_PATH = "vision/model/labelmap.txt"  # COCO Dataset

# Load labels
with open(LABEL_PATH, 'r') as f:
    labels = [line.strip() for line in f.readlines()]

# Load model
interpreter = Interpreter(model_path=MODEL_PATH)
interpreter.allocate_tensors()

input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

input_shape = input_details[0]['shape']
height = input_shape[1]
width = input_shape[2]
floating_model = input_details[0]['dtype'] == np.float32

# Start webcam
cap = cv2.VideoCapture(0)

new_frame_time = 0
prev_frame_time = 0
while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Preprocess
    image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    image_resized = cv2.resize(image_rgb, (width, height))
    input_data = np.expand_dims(image_resized, axis=0)

    if floating_model:
        input_data = (np.float32(input_data) - 127.5) / 127.5
    else:
        input_data = np.uint8(input_data)

    # Run inference
    interpreter.set_tensor(input_details[0]['index'], input_data)
    interpreter.invoke()

    # Get outputs
    boxes = interpreter.get_tensor(output_details[0]['index'])[0]
    classes = interpreter.get_tensor(output_details[1]['index'])[0]
    scores = interpreter.get_tensor(output_details[2]['index'])[0]
    num = interpreter.get_tensor(output_details[3]['index'])[0]

    # Draw detections
    imH, imW, _ = frame.shape
    for i in range(int(num)):
        if scores[i] > 0.5:
            ymin = int(max(1, boxes[i][0] * imH))
            xmin = int(max(1, boxes[i][1] * imW))
            ymax = int(min(imH, boxes[i][2] * imH))
            xmax = int(min(imW, boxes[i][3] * imW))

            class_id = int(classes[i])
            label = labels[class_id] if class_id < len(labels) else "N/A"
            confidence = int(scores[i] * 100)

            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
            cv2.putText(frame, f"{label} ({confidence}%)", (xmin, ymin - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            # Show output


    new_frame_time = time.time()
    fps = 1 / (new_frame_time - prev_frame_time)
    prev_frame_time = new_frame_time # Get fps

    # Display FPS on the frame (optional)
    cv2.putText(frame, f"FPS: {fps}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.imshow("Object Detection", frame)

    if cv2.waitKey(1) == ord("q"):  # ESC to quit
        break

cap.release()
cv2.destroyAllWindows()