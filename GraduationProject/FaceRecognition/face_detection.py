import cv2
import numpy as np
import subprocess
import time
from datetime import datetime
import os

# Define the command to use libcamera-vid to stream to stdout
command = [
    "libcamera-vid",
    "--width", "640",
    "--height", "480",
    "--framerate", "30",
    "-t", "0",           # 0 means unlimited streaming time
    "--codec", "mjpeg",
    "-o", "-"            # Stream to stdout
]

# Start the libcamera-vid subprocess
camera_process = subprocess.Popen(command, stdout=subprocess.PIPE, bufsize=10**6)

# Load the Haar Cascade for face detection
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# Allow the camera to warm up
time.sleep(0.1)

# Check if a display is available
display_available = os.environ.get('DISPLAY') is not None

# Start reading the video stream
try:
    while True:
        # Read a frame from the stream
        frame_bytes = camera_process.stdout.read(640 * 480 * 2)  # adjust if needed
        if not frame_bytes:
            print("Error: Could not read frame.")
            break

        # Decode the JPEG image to a numpy array
        frame = cv2.imdecode(np.frombuffer(frame_bytes, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            print("Error: Could not decode frame.")
            continue

        # Convert the frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect faces
        faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

        # Draw rectangles around detected faces and log their positions
        for (x, y, w, h) in faces:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            print(f"Detected face at x={x}, y={y}, width={w}, height={h}")

        # Save each frame with a unique filename
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        output_filename = f"output_frame_{timestamp}.jpg"
        cv2.imwrite(output_filename, frame)
        print(f"Frame saved as {output_filename}")

        # Display the frame if a display is available
        if display_available:
            cv2.imshow("Frame", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
finally:
    # Cleanup
    camera_process.terminate()
    cv2.destroyAllWindows()

