import RPi.GPIO as GPIO
import time
import statistics

# GPIO pin configuration (ensure unique pins for each sensor)
SENSOR_PINS = {
    'sensor1': {'trig': 38, 'echo': 40},
    'sensor2': {'trig': 16, 'echo': 18},
    'sensor3': {'trig': 22, 'echo': 24}  # New unique pins for sensor3
}

# Set up GPIO
GPIO.setmode(GPIO.BOARD)
for sensor, pins in SENSOR_PINS.items():
    GPIO.setup(pins['trig'], GPIO.OUT)
    GPIO.setup(pins['echo'], GPIO.IN)
    print(f"Configured {sensor}: TRIG={pins['trig']} ECHO={pins['echo']}")

def get_distance(trig, echo, timeout=0.01):
    """
    Measure the distance using an ultrasonic sensor.

    Parameters:
        trig (int): GPIO pin for the trigger signal.
        echo (int): GPIO pin for the echo signal.
        timeout (float): Maximum time (in seconds) to wait for the signal.

    Returns:
        float: Distance in centimeters, or None if no measurement.
    """
    try:
        # Ensure the trigger is low
        GPIO.output(trig, False)
        time.sleep(0.0002)

        # Send a 10us pulse to trigger
        GPIO.output(trig, True)
        time.sleep(0.00001)
        GPIO.output(trig, False)

        # Wait for echo to start (rising edge)
        start_time = time.time()
        while GPIO.input(echo) == 0:
            if time.time() - start_time > timeout:
                print(f"Timeout waiting for ECHO signal to go HIGH on TRIG={trig}, ECHO={echo}")
                return None

        # Wait for echo to end (falling edge)
        stop_time = time.time()
        while GPIO.input(echo) == 1:
            if time.time() - stop_time > timeout:
                print(f"Timeout waiting for ECHO signal to go LOW on TRIG={trig}, ECHO={echo}")
                return None

        # Calculate time difference and convert to distance
        elapsed_time = stop_time - start_time
        distance = (elapsed_time * 34300) / 2  # cm

        if 2 < distance < 400:  # Filter out-of-range values
            return distance
        else:
            print(f"Distance out of range: {distance:.2f} cm")
    except Exception as e:
        print(f"Error measuring distance on TRIG={trig}, ECHO={echo}: {e}")
    return None

def measure_with_filter(trig, echo, samples=5):
    """
    Measure distance using multiple samples and apply a median filter.

    Parameters:
        trig (int): GPIO pin for the trigger signal.
        echo (int): GPIO pin for the echo signal.
        samples (int): Number of samples to take for filtering.

    Returns:
        float: Filtered distance in centimeters.
    """
    print(f"Starting filtered measurement on TRIG={trig}, ECHO={echo} with {samples} samples")
    measurements = []
    for i in range(samples):
        measurement = get_distance(trig, echo)
        if measurement is not None:
            measurements.append(measurement)
        else:
            print(f"Invalid measurement for sample {i + 1}")

    if measurements:
        filtered_distance = statistics.median(measurements)
        print(f"Filtered distance: {filtered_distance:.2f} cm")
        return filtered_distance
    else:
        print(f"No valid measurements for TRIG={trig}, ECHO={echo}")
        return None

try:
    print("Starting distance measurement loop...")
    while True:
        distances = {}
        for sensor, pins in SENSOR_PINS.items():
            print(f"\nMeasuring {sensor}...")
            distance = measure_with_filter(pins['trig'], pins['echo'])
            distances[sensor] = distance
            if distance:
                print(f"{sensor}: {distance:.2f} cm")
            else:
                print(f"{sensor}: Out of range or no measurement")

        print("\nAll measurements complete. Waiting for the next cycle...")
        time.sleep(0.5)

except KeyboardInterrupt:
    print("\nMeasurement stopped by user.")

finally:
    GPIO.cleanup()
    print("GPIO cleaned up.")
