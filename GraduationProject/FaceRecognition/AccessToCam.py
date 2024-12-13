from flask import Flask, Response
from picamera2 import Picamera2
import cv2

# Initialize the Flask application
app = Flask(__name__)

# Initialize the camera
picam2 = Picamera2()
camera_config = picam2.create_video_configuration(main={"format":"XRGB8888","size": (640, 480)})
picam2.configure(camera_config)
picam2.start()


def generate_frames():
    """Generator function that captures frames from the camera and encodes them as JPEG."""
    while True:
        # Capture frame-by-frame
        frame = picam2.capture_array()

        # Convert the frame to JPEG
        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()

        # Yield frame in byte format for streaming
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


@app.route('/video_feed')
def video_feed():
    """Route to stream video from the camera."""
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    # Run the Flask app
    app.run(host='0.0.0.0', port=5000)
