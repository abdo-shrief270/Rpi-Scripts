import RPi.GPIO as GPIO
import time
import math

GPIO.cleanup()  # Reset GPIO setup before initializing

# Constants
PULSES_PER_REVOLUTION = 90  # Number of encoder pulses per wheel revolution
WHEEL_DIAMETER_CM = 6.5     # Diameter of the wheel in cm
ENCODER_PIN = 26            # GPIO pin for the encoder

# Calculate the distance per pulse
DISTANCE_PER_PULSE = (math.pi * WHEEL_DIAMETER_CM) / PULSES_PER_REVOLUTION

# Variables
pulse_count = 0  # Number of encoder pulses detected
total_distance = 0.0  # Distance traveled in cm

def setup_encoder():
    """
    Sets up the Raspberry Pi GPIO for the encoder.
    """
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(ENCODER_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(ENCODER_PIN, GPIO.RISING, callback=encoder_callback)

def encoder_callback(channel):
    """
    Callback function for the encoder pulse detection.
    Updates the pulse count and calculates the traveled distance.
    
    Parameters:
        channel (int): The GPIO channel that triggered the callback.
    """
    global pulse_count, total_distance
    pulse_count += 1
    # Update total distance traveled
    total_distance = pulse_count * DISTANCE_PER_PULSE

def reset_distance():
    """
    Resets the pulse count and total distance to zero.
    """
    global pulse_count, total_distance
    pulse_count = 0
    total_distance = 0.0

def get_distance():
    """
    Retrieves the current distance traveled.
    
    Returns:
        float: Total distance traveled in centimeters.
    """
    return total_distance

def main():
    """
    Main function to initialize the encoder setup and periodically print the distance traveled.
    """
    try:
        setup_encoder()
        print("Encoder initialized. Measuring distance...")

        while True:
            time.sleep(1)  # Adjust sleep time as needed
            print(f"Distance traveled: {get_distance():.2f} cm")

    except KeyboardInterrupt:
        print("\nStopping program.")
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()
