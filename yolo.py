import cv2
import torch
import serial
import time
from ultralytics import YOLO

# Load YOLOv5 model (Use 'yolov5s.pt' or 'yolov5n.pt' for faster speed)
model = YOLO("yolov5s.pt")  

# Open laptop camera
cap = cv2.VideoCapture(0)

# Connect to Arduino (Change COM port as needed)
arduino = serial.Serial(port='COM3', baudrate=9600, timeout=1)
time.sleep(2)  # Wait for Arduino to initialize

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Run YOLOv5 inference
    results = model(frame)

    # Check if humans are detected
    human_detected = False
    for r in results:
        for box in r.boxes:
            cls = int(box.cls[0])  # Get class index
            if cls == 0:  # Class 0 is 'person' in COCO dataset
                human_detected = True
                break

    # Send signal to Arduino
    if human_detected:
        arduino.write(b'1')  # Turn LED ON
        print("Human detected! LED ON")
    else:
        arduino.write(b'0')  # Turn LED OFF
        print("No human detected. LED OFF")

    # Display results
    frame_with_boxes = results[0].plot()  # Draw bounding boxes
    cv2.imshow("YOLOv5 Human Detection", frame_with_boxes)

    # Exit when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
cap.release()
cv2.destroyAllWindows()
arduino.close()
