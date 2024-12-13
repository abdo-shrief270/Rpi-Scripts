import RPi.GPIO as GPIO
import time
from collections import deque

# Set the GPIO mode to BOARD
GPIO.setmode(GPIO.BOARD)

class UltrasonicSensor:
    def __init__(self, trig_pin, echo_pin, window_size=3):
        """
        Initializes the ultrasonic sensor with the specified GPIO pins and optional window size.
        
        :param trig_pin: GPIO pin connected to the TRIG of the ultrasonic sensor.
        :param echo_pin: GPIO pin connected to the ECHO of the ultrasonic sensor.
        :param window_size: Number of measurements to consider for the moving average filter.
        """
        self.trig_pin = trig_pin
        self.echo_pin = echo_pin
        self.window_size = window_size
        self.distances = deque(maxlen=window_size)  # To store the last 'window_size' measurements

        # Set up the TRIG and ECHO pins
        GPIO.setup(self.trig_pin, GPIO.OUT)  # TRIG pin as output
        GPIO.setup(self.echo_pin, GPIO.IN)   # ECHO pin as input

        # Ensure the TRIG pin is LOW initially
        GPIO.output(self.trig_pin, GPIO.LOW)

        # Wait for the sensor to stabilize
        time.sleep(0.5)

    def measure_distance(self):
        """
        Measures the distance using the ultrasonic sensor and applies a moving average filter.
        
        :return: Averaged distance in centimeters (float).
        """
        # Send a 10us pulse to the TRIG pin to trigger the sensor
        GPIO.output(self.trig_pin, GPIO.HIGH)
        time.sleep(0.00001)  # 10 microseconds
        GPIO.output(self.trig_pin, GPIO.LOW)

        # Record the start time and wait for the echo to be received
        start_time = time.time()
        while GPIO.input(self.echo_pin) == GPIO.LOW:
            start_time = time.time()

        # Record the time the echo is received
        stop_time = time.time()
        while GPIO.input(self.echo_pin) == GPIO.HIGH:
            stop_time = time.time()

        # Calculate the duration of the pulse
        pulse_duration = stop_time - start_time

        # Calculate the distance (in centimeters)
        distance = pulse_duration * 17150  # Speed of sound is 34300 cm/s (divide by 2 for round-trip)
        distance = round(distance)  # Round to integer

        # Store the current distance measurement in the deque (circular buffer)
        self.distances.append(distance)

        # Compute the average distance from the last 'window_size' measurements
        if len(self.distances) == self.window_size:
            avg_distance = sum(self.distances) / len(self.distances)
        else:
            # If not enough measurements yet, return the latest measurement
            avg_distance = distance

        return round(avg_distance, 2)  # Return the average, rounded to two decimal places

    def cleanup(self):
        """
        Cleans up the GPIO settings. Should be called when done using the sensor.
        """
        GPIO.cleanup()
