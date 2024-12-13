import RPi.GPIO as GPIO
import time

# Setup GPIO mode
GPIO.setmode(GPIO.BCM)

# Define encoder pin
ENCODER_PIN = 26

# Setup encoder pin as input with internal pull-down resistor
GPIO.setup(ENCODER_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# Initialize pulse count and previous state
distance    = 0 # disntace in meter
pulse_count = 0
last_state  = GPIO.LOW

print("Starting Optical Encoder Test...")

try:
    while True:
        # Read the current state of the encoder pin
        current_state = GPIO.input(ENCODER_PIN)

        # Detect a rising edge (LOW to HIGH transition)
        if last_state == GPIO.LOW and current_state == GPIO.HIGH:
            pulse_count += 1
            distance     = pulse_count / 90
            print(f"Pulse count: {pulse_count} , distance: {distance:.2f}")

        # Update the last state to the current state
        last_state = current_state

        # Small delay to avoid high CPU usage
        time.sleep(0.001)

except KeyboardInterrupt:
    # Gracefully handle program exit
    print("\nExiting program.")
    GPIO.cleanup()
finally:
    # Clean up GPIO settings
    GPIO.cleanup()
