import time
import threading
from ultrasonic_sensor import UltrasonicSensor
from mpu6050 import MPU6050
import serial
import RPi.GPIO as GPIO

# GPIO pins for ultrasonic sensors
ULTRASONIC_PINS = [
    {"trig": 38, "echo": 40},  # Front sensor
    {"trig": 16, "echo": 18},  # Left sensor
    {"trig": 22, "echo": 24},  # Right sensor
]

# Initialize Ultrasonic Sensors
ultrasonics = [
    UltrasonicSensor(sensor["trig"], sensor["echo"], window_size=5)
    for sensor in ULTRASONIC_PINS
]

# Initialize MPU6050
mpu = MPU6050()

# Initialize Serial Connection
try:
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    time.sleep(0.5)
except serial.SerialException as e:
    print(f"Error initializing serial: {e}")
    exit()

ser_lock = threading.Lock()
stop_event = threading.Event()

# Encoder Setup
ENCODER_PIN = 37
GPIO.setup(ENCODER_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
pulse_count = 0
distance = 0
last_state = GPIO.LOW

# Function to monitor encoder pulses
def monitor_encoder():
    global pulse_count, distance, last_state
    print("Starting Encoder Monitoring...")
    try:
        while not stop_event.is_set():
            current_state = GPIO.input(ENCODER_PIN)
            if last_state == GPIO.LOW and current_state == GPIO.HIGH:
                pulse_count += 1
                distance = pulse_count / 90  # Convert pulses to meters (adjust as needed)
                print(f"Pulse count: {pulse_count}, Distance: {distance:.2f} meters")
            last_state = current_state
            time.sleep(0.001)
    except Exception as e:
        print(f"Encoder error: {e}")
    finally:
        GPIO.cleanup()

# Function to send command to Arduino
def send_command(command):
    with ser_lock:
        ser.write((command + "\n").encode())

# Function to read serial feedback
def read_serial():
    while not stop_event.is_set():
        if ser.in_waiting > 0:
            try:
                with ser_lock:
                    data = ser.readline().decode('utf-8', errors='ignore').strip()
                if data:
                    pass
                    # print(f"Arduino: {data}")
            except UnicodeDecodeError as e:
                print(f"Error decoding serial data: {e}")

# Function to perform sequence
def perform_sequence(sequence_type):
    send_command(f"executeSequence:{sequence_type}")

# Function to perform path
def perform_path(x, y, speed):
    command = f"executeCoordinates:xValue={x}&yValue={y}&speed={speed}"
    send_command(command)

# Function to initiate obstacle avoidance
def obstacle_avoidance():
    print("Obstacle avoidance activated")
    while not stop_event.is_set():
        distances = [sensor.measure_distance() for sensor in ultrasonics]
        print(f"Distances: {distances} cm")
        
        if distances[1] <= 50:
            send_command("stop,0")
        else:
            send_command("forward,50")
        time.sleep(0.1)

# Main control logic
def control_logic():
    try:
        while not stop_event.is_set():
            user_input = input("Enter mode (sequence/path/avoid): ").strip().lower()
            if user_input == "sequence":
                sequence_type = input("Enter sequence type (moveSquare/moveRectangle/moveTriangle): ")
                perform_sequence(sequence_type)
            elif user_input == "path":
                x = int(input("Enter X coordinate: "))
                y = int(input("Enter Y coordinate: "))
                speed = int(input("Enter speed: "))
                perform_path(x, y, speed)
            elif user_input == "avoid":
                obstacle_avoidance()
            else:
                print("Invalid mode.")
            time.sleep(0.5)
    except Exception as e:
        print(f"Control logic error: {e}")

# Main function
if __name__ == "__main__":
    try:
        # Start serial read thread
        serial_thread = threading.Thread(target=read_serial, daemon=True)
        serial_thread.start()

        # Start encoder monitoring thread
        # encoder_thread = threading.Thread(target=monitor_encoder, daemon=True)
        # encoder_thread.start()

        # Start control logic
        control_logic()
    except KeyboardInterrupt:
        print("Program interrupted by user.")
        stop_event.set()
    finally:
        for sensor in ultrasonics:
            sensor.cleanup()
        if ser.is_open:
            ser.close()
        GPIO.cleanup()
        print("Program terminated.")
