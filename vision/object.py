import cv2 as cv
import numpy as np
from ai_edge_litert.interpreter import Interpreter

# Labels for the COCO dataset (used by SSD MobileNetV2)
COCO_LABELS = {
    1: 'person', 2: 'bicycle', 3: 'car', 4: 'motorcycle', 5: 'airplane',
    6: 'bus', 7: 'train', 8: 'truck', 9: 'boat', 10: 'traffic light',
    11: 'fire hydrant', 13: 'stop sign', 14: 'parking meter', 15: 'bench',
    16: 'bird', 17: 'cat', 18: 'dog', 19: 'horse', 20: 'sheep', 21: 'cow',
    22: 'elephant', 23: 'bear', 24: 'zebra', 25: 'giraffe', 27: 'backpack',
    28: 'umbrella', 31: 'handbag', 32: 'tie', 33: 'suitcase', 34: 'frisbee',
    35: 'skis', 36: 'snowboard', 37: 'sports ball', 38: 'kite', 39: 'baseball bat',
    40: 'baseball glove', 41: 'skateboard', 42: 'surfboard', 43: 'tennis racket',
    44: 'bottle', 46: 'wine glass', 47: 'cup', 48: 'fork', 49: 'knife', 50: 'spoon',
    51: 'bowl', 52: 'banana', 53: 'apple', 54: 'sandwich', 55: 'orange',
    56: 'broccoli', 57: 'carrot', 58: 'hot dog', 59: 'pizza', 60: 'donut',
    61: 'cake', 62: 'chair', 63: 'couch', 64: 'potted plant', 65: 'bed',
    67: 'dining table', 70: 'toilet', 72: 'tv', 73: 'laptop', 74: 'mouse',
    75: 'remote', 76: 'keyboard', 77: 'cell phone', 78: 'microwave', 79: 'oven',
    80: 'toaster', 81: 'sink', 82: 'refrigerator', 84: 'book', 85: 'clock',
    86: 'vase', 87: 'scissors', 88: 'teddy bear', 89: 'hair drier', 90: 'toothbrush'
}

# Initialize TFLite interpreter
interpreter = Interpreter(model_path="vision/ssd_mobilenet_v2.tflite")
interpreter.allocate_tensors()

# Get model details
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
print(input_details)


def process_image(image, input_shape):
    # Get exact dimensions from input shape
    _, height, width, channels = input_shape
    # Resize to 1x1 as per input shape
    img = cv.resize(image, (width, height))
    # Convert to float32
    img = img.astype(np.float32)
    # Add batch dimension
    img = np.expand_dims(img, axis=0)
    return img


def detect_objects(interpreter, image, threshold=0.5):
    input_shape = interpreter.get_input_details()[0]['shape']
    processed_image = process_image(image, input_shape)
    
    # Set input tensor
    interpreter.set_tensor(input_details[0]['index'], processed_image)
    
    # Run inference
    interpreter.invoke()
    
    # Get output tensors
    boxes = interpreter.get_tensor(output_details[0]['index'])[0]
    classes = interpreter.get_tensor(output_details[1]['index'])[0]
    scores = interpreter.get_tensor(output_details[2]['index'])[0]
    
    return boxes, classes, scores

# Initialize camera
cam = cv.VideoCapture(0)

if not cam.isOpened():
    print("Unable to access camera")
    cam.release()
    exit()

while True:
    retrieved, frame = cam.read()

    if not retrieved:
        print("Stream has likely ended")
        break

    # Get detections
    try:
        boxes, classes, scores = detect_objects(interpreter, frame)
        
        # Process detections
        height, width, _ = frame.shape
        for i in range(len(scores)):
            if scores[i] > 0.5:
                # Get bounding box
                ymin, xmin, ymax, xmax = boxes[i]
                
                # Convert normalized coordinates to pixel coordinates
                xmin = int(xmin * width)
                xmax = int(xmax * width)
                ymin = int(ymin * height)
                ymax = int(ymax * height)
                
                # Get class name
                class_id = int(classes[i])
                if class_id in COCO_LABELS:
                    class_name = COCO_LABELS[class_id]
                else:
                    class_name = f"Class {class_id}"
                    
                # Draw bounding box and label
                cv.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
                label = f"{class_name}: {scores[i]:.2f}"
                cv.putText(frame, label, (xmin, ymin - 10),
                          cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    except Exception as e:
        print(f"Error during detection: {e}")

    cv.imshow("Object Detection", frame)

    if cv.waitKey(1) == ord("q"):
        break

cam.release()
cv.destroyAllWindows()