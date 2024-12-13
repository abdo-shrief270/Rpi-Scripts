import time
import threading
from ultrasonic_sensor import UltrasonicSensor
from mpu6050 import MPU6050
import serial

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
    time.sleep(2)
except serial.SerialException as e:
    print(f"Error initializing serial: {e}")
    exit()

ser_lock = threading.Lock()
stop_event = threading.Event()

# Function to send command to Arduino
def send_command(command):
    with ser_lock:
        ser.write((command + "\n").encode())
        print(f"Sent command: {command}")

# Function to read serial feedback
def read_serial():
    while not stop_event.is_set():
        if ser.in_waiting > 0:
            try:
                with ser_lock:
                    data = ser.readline().decode('utf-8', errors='ignore').strip()
                if data:
                    print(f"Arduino: {data}")
            except UnicodeDecodeError as e:
                print(f"Error decoding serial data: {e}")

# Function to monitor sensors and send movement commands
def sensor_logic():
    try:
        while not stop_event.is_set():
            # Measure distances
            distances = [sensor.measure_distance() for sensor in ultrasonics]
            print(f"Distances: {distances} cm")
            
            # Get yaw angle
            yaw = mpu.get_stable_yaw()
            print(f"Yaw angle: {yaw:.2f}°")
            
            # Movement logic
            if distances[0] < 20:  # Front obstacle
                command = "stop,0"
            elif distances[1] < 15:  # Left obstacle
                command = "stop,0"
            elif distances[2] < 15:  # Right obstacle
                command = "stop,0"
            else:
                command = "forward,50"
            
            # Adjust direction with yaw
            if abs(yaw) > 10:
                command = "ADJUST RIGHT" if yaw > 0 else "ADJUST LEFT"
            
            send_command(command)
            time.sleep(0.1)
    except Exception as e:
        print(f"Sensor logic error: {e}")

# Main function
if __name__ == "__main__":
    try:
        # Start serial read thread
        serial_thread = threading.Thread(target=read_serial, daemon=True)
        serial_thread.start()

        # Start sensor logic
        sensor_logic()
    except KeyboardInterrupt:
        print("Program interrupted by user.")
        stop_event.set()
    finally:
        for sensor in ultrasonics:
            sensor.cleanup()
        if ser.is_open:
            ser.close()
        print("Program terminated.")
