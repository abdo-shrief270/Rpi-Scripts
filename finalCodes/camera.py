import asyncio
import cv2
import websockets
from picamera2 import Picamera2
import base64

# Initialize the camera using Picamera2
picam2 = Picamera2()
camera_config = picam2.create_video_configuration(main={"format": "RGB888", "size": (1920, 1080)})
picam2.configure(camera_config)
picam2.start()

# Define WebSocket server settings
HOST = '0.0.0.0'
PORT = 8765

async def send_video(websocket):
    try:
        while True:
            # Capture a frame from the camera
            frame = picam2.capture_array()

            # Encode frame as JPEG
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 95]
            ret, jpeg_frame = cv2.imencode('.jpg', frame, encode_param)
            if not ret:
                print("Error: Failed to encode frame.")
                continue

            # Convert the frame to base64
            encoded_frame = base64.b64encode(jpeg_frame).decode('utf-8')

            # Send the frame as base64-encoded data over WebSocket
            await websocket.send(encoded_frame)

            # Sleep to maintain frame rate
            await asyncio.sleep(1 / 30)
    except websockets.exceptions.ConnectionClosed:
        print("Connection closed.")

async def main():
    async with websockets.serve(send_video, HOST, PORT):
        print(f"WebSocket server started at ws://{HOST}:{PORT}")
        await asyncio.Future()  # Run forever

asyncio.run(main())
