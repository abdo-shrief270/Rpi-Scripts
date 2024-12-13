import cv2
import numpy as np
from picamera2 import Picamera2, Preview
import time

# Initialize the camera
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration())
picam2.start()

# Load the Haar Cascade for face detection
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

try:
    while True:
        # Capture an image
        image = picam2.capture_array()

        # Convert the image to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Detect faces
        faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

        # Draw rectangles around detected faces
        for (x, y, w, h) in faces:
            cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 2)

        # Show the image
        cv2.imshow("Frame", image)

        # Exit on 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    picam2.stop()
    cv2.destroyAllWindows()
