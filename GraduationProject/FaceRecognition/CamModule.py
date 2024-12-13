import time
import os
import subprocess
from datetime import datetime

def test_camera_connection() -> str:

    try:
        photo_path = f"media/photos/image_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
        # Run the rpicam-still command with arguments
        subprocess.run(["rpicam-still", "--output", photo_path])
        return photo_path
    except Exception as e:
        return 'null'

def capture_image() -> str:
    
    photo_path = f"media/photos/image_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
    subprocess.run(["rpicam-still", "--output", photo_path])
    return photo_path
    
def record_video(sec : int =5) -> str:
    video_path = f"media/videos/video_{datetime.now().strftime('%Y%m%d_%H%M%S')}.h264"
    subprocess.run(["rpicam-vid", "-t",f'{sec}s', "-o",video_path])
    return video_path
