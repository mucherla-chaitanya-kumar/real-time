import cv2
import numpy as np
import tensorflow as tf
import serial
import time

# Load a pre-trained MobileNet SSD model for human detection
model = tf.keras.applications.MobileNetV2(weights="imagenet")

# Open the laptop camera
cap = cv2.VideoCapture(0)

# Connect to Arduino (Change the COM port as per your system)
arduino = serial.Serial(port='COM3', baudrate=9600, timeout=1)
time.sleep(2)  # Allow Arduino to initialize

# Define class labels for MobileNet (only checking for 'person')
LABELS = {15: 'person'}  # Class index 15 corresponds to "person" in ImageNet

def preprocess_image(frame):
    """ Preprocess the image for CNN model input """
    frame_resized = cv2.resize(frame, (224, 224))  # Resize to 224x224 for MobileNet
    frame_array = np.expand_dims(frame_resized, axis=0)  # Add batch dimension
    frame_array = tf.keras.applications.mobilenet_v2.preprocess_input(frame_array)  # Normalize
    return frame_array

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Preprocess the frame for MobileNetV2
    processed_frame = preprocess_image(frame)

    # Make predictions using the CNN model
    predictions = model.predict(processed_frame)
    decoded_predictions = tf.keras.applications.mobilenet_v2.decode_predictions(predictions, top=3)[0]

    human_detected = any(label == "person" for (_, label, _) in decoded_predictions)

    # Send signal to Arduino
    if human_detected:
        arduino.write(b'1')  # Send '1' to turn LED ON
        print("Human detected! LED ON")
    else:
        arduino.write(b'0')  # Send '0' to turn LED OFF
        print("No human detected. LED OFF")

    # Display output
    cv2.imshow("Camera Feed", frame)
    
    # Exit on pressing 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
arduino.close()
