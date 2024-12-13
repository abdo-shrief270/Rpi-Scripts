import asyncio
import cv2
import websockets
import base64
from webcamvideostream import WebcamVideoStream

# Define WebSocket server settings
HOST = '0.0.0.0'  # Listen on all interfaces
PORT = 8765  # You can choose another port if needed

def gen(camera):
    while True:
        if camera.stopped:
            break
        frame = camera.read()
        ret, jpeg = cv2.imencode('.jpg',frame)
        if jpeg is not None:
            return jpeg
        else:
            print("frame is none")

async def send_video(websocket):
    try:
        while True:
            # Capture a frame from the camera
            frame = gen(WebcamVideoStream().start())

            # Convert the frame to base64
            encoded_frame = base64.b64encode(frame).decode('utf-8')

            # Send the frame as base64-encoded data over WebSocket
            await websocket.send(encoded_frame)

            # Sleep for a short time to simulate 30 FPS
            await asyncio.sleep(1/30)
    except websockets.exceptions.ConnectionClosed:
        print("Connection closed.")
    finally:
        # Cleanup resources if connection is closed
        pass

# Start the WebSocket server
async def main():
    # The handler now accepts both websocket and path arguments
    async with websockets.serve(send_video, HOST, PORT):
        print(f"WebSocket server started at ws://{HOST}:{PORT}")
        await asyncio.Future()  # Run forever

# Run the WebSocket server
asyncio.run(main())











# import cv2
# import sys
# from flask import Flask, render_template, Response
# from webcamvideostream import WebcamVideoStream
# from flask_basicauth import BasicAuth
# import time
# import threading

# app = Flask(__name__)

# last_epoch = 0


# @app.route('/')

# def index():
#     return render_template('index.html')



# @app.route('/video_feed')
# def video_feed():
#     return Response(),
#                     mimetype='multipart/x-mixed-replace; boundary=frame')

# if __name__ == '__main__':
#     app.run(host='0.0.0.0', debug=True, threaded=True)