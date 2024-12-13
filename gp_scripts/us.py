import time
from ultrasonic_sensor import UltrasonicSensor

# Define GPIO pins for the ultrasonic sensor
TRIG_PIN = 38  # TRIG pin connected to GPIO pin 38
ECHO_PIN = 40  # ECHO pin connected to GPIO pin 40

# Create an instance of the UltrasonicSensor class with a window size of 5
sensor = UltrasonicSensor(TRIG_PIN, ECHO_PIN, window_size=5)

try:
    while True:
        # Measure and print the filtered (average) distance
        distance = sensor.measure_distance()
        print(f"Filtered Distance: {distance} cm")

        # Wait for a while before taking the next measurement
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Measurement stopped by user")

finally:
    sensor.cleanup()  # Clean up GPIO settings when done
