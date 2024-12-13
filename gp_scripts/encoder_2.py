import RPi.GPIO as GPIO
import time

# Define pin for encoder
ENCODER_PIN = 37  # Replace with your actual pin number

# Set up GPIO mode
GPIO.setmode(GPIO.BCM)
GPIO.setup(ENCODER_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

try:
    print("Testing encoder pin...")
    while True:
        pin_state = GPIO.input(ENCODER_PIN)
        print(f"Pin state: {pin_state}")
        time.sleep(0.5)
except KeyboardInterrupt:
    print("Interrupted by user.")
finally:
    GPIO.cleanup()
    print("GPIO cleanup complete.")
