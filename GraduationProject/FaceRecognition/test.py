import subprocess
from datetime import datetime

# Construct the file path with formatted date and time
file_path = f"media/photos/image_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"

# Run the rpicam-still command with arguments
subprocess.run(["rpicam-still", "--encoding", "png", "--output", file_path])

