import time
from ultrasonic_sensor import UltrasonicSensor
from mpu6050 import MPU6050
import serial
import threading
import pyrebase
import math
import RPi.GPIO as GPIO
import pyproj
import subprocess
from picamera2 import Picamera2
import cv2
import io

def check_internet():
    try:
        # Ping a reliable server (Google DNS in this case)
        subprocess.check_call(['ping', '-c', '1', '8.8.8.8'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        return True
    except subprocess.CalledProcessError:
        return False
    
car_x=0
car_y=0
car_speed=0
steps = 0
movedDistance = 0

def geodetic_to_ecef(lat, lon, alt):
    transformer = pyproj.Transformer.from_crs("EPSG:4326", "EPSG:4978", always_xy=True)
    x, y, z = transformer.transform(lon, lat, alt)
    return x, y, z

encoder_pin = 37
GPIO.setup(encoder_pin, GPIO.IN)

# Firebase configuration
firebase_config = {
    "apiKey": "AIzaSyCt3PpT9MV8TBlsgDmjvUngq-ZF1lNYQ2c",
    "authDomain": "rpi-car-track.firebaseapp.com",
    "databaseURL": "https://rpi-car-track-default-rtdb.firebaseio.com",
    "projectId": "rpi-car-track",
    "storageBucket": "rpi-car-track.firebasestorage.app",
    "messagingSenderId": "603017187064",
    "appId": "1:603017187064:web:c8da4c7a71d9745fe8b50d",
    "measurementId": "G-GEM4R145LE"
}

# Initialize Firebase
firebase = pyrebase.initialize_app(firebase_config)
db = firebase.database()
storage = firebase.storage()
picam2 = Picamera2()
camera_config = picam2.create_video_configuration(main={"format":"XRGB8888","size": (640, 480)})
picam2.configure(camera_config)
picam2.start()

def save_log(status,text):
    log = {
        "status": status,
        "text": text,
        "timestamp": time.time()
    }
    db.child("logs").push(log)
    db.child("log").update(log)


# Initialize Serial Connection
try:
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    save_log("info","Arudino Serial is Running now")
    
    time.sleep(0.25)
except serial.SerialException as e:
    save_log("error","Arudino Serial is not Running")
    exit()




# Function to upload image to Firebase Storage
def upload_to_firebase(image_data):
    # Create a unique name for the image file
    timestamp = int(time.time())
    image_filename = f"image_{timestamp}.jpg"
    
    # Upload the image to Firebase Storage
    storage.child(f"images/{image_filename}").put(io.BytesIO(image_data))
    print(f"Image uploaded: {image_filename}")

# Function to continuously capture frames and upload images to Firebase in a separate thread
def capture_and_upload():
    while True:
        # Capture frame-by-frame
        frame = picam2.capture_array()

        # Convert the frame to JPEG
        ret, buffer = cv2.imencode('.jpg', frame)
        frame_data = buffer.tobytes()

        # Upload to Firebase
        upload_to_firebase(frame_data)

        # Sleep for a bit to control the upload rate (e.g., once every 5 seconds)
        time.sleep(3)  # Adjust the sleep time as necessary

# Start the capture and upload thread
upload_image_thread = threading.Thread(target=capture_and_upload, daemon=True)
upload_image_thread.start()


def send_command(command):
    ser.write((command + "\n").encode())
    

def update_online():
    while True:
        db.child('last_online').set(time.time())
        db.child('distance').set(movedDistance)
        db.child('speed').set(car_speed)
        time.sleep(0.01)  # Update every 10ms

# Start the thread
online_thread = threading.Thread(target=update_online, daemon=True)
online_thread.start()
save_log("info","Project is Starting")
save_log("info","Online Thread is Running now")

# GPIO pins for ultrasonic sensors
ULTRASONIC_PINS = [
    {"trig": 38, "echo": 40},  # Front sensor
    {"trig": 16, "echo": 18},  # Left sensor
    {"trig": 22, "echo": 24},  # Right sensor
]

save_log("warning","Ultrasonics is intializing now")

# Initialize Ultrasonic Sensors
ultrasonics = [
    UltrasonicSensor(sensor["trig"], sensor["echo"], window_size=5)
    for sensor in ULTRASONIC_PINS
]

save_log("info","Ultrasonics is intialized now")

# Initialize MPU6050
save_log("warning","MPU is intializing now")
mpu = MPU6050()
save_log("info","MPU is intialized now")

mpu_yaw = 0
def update_mpu_yaw():
    global mpu_yaw
    while True:
        mpu_yaw = mpu.get_stable_yaw()
        db.child('angle').set(mpu_yaw)
        time.sleep(0.01)  # Update every 10ms

# Start the thread
mpu_thread = threading.Thread(target=update_mpu_yaw, daemon=True)
mpu_thread.start()
save_log("info","MPU Yaw Thread is Running now")


encoder_pin = 37
GPIO.setup(encoder_pin, GPIO.IN)
def update_steps():
    global steps,movedDistance
    while True:      
        if GPIO.input(encoder_pin):
            steps += 1
            movedDistance=(steps / 90.0) * 100
            while GPIO.input(encoder_pin):
                time.sleep(0.001)

# Start the thread
encoder_thread = threading.Thread(target=update_steps, daemon=True)
encoder_thread.start()
save_log("info","Encoder Distance Thread is Running now")


def calculate_diameter_and_angle(x=0, y=0):
    diameter = math.sqrt(x**2 + y**2)
    angle_radians = math.atan2(y, x)
    angle_degrees = math.degrees(angle_radians)

    return diameter, angle_degrees


def update_gps_data():
    while True:
        send_command("getGpsData")
        if ser.in_waiting > 0:
            try:
                data = ser.readline().decode('utf-8', errors='ignore').strip()
                if data:
                    data=data.split("&")
                    if float(data[2])==0.00:
                        continue
                    
                    gpsData = {
                        "latitude": data[0],
                        "longitude": data[1],
                        "altitude": data[2],
                        "timestamp": time.time()
                    }
                    db.child("location").update(gpsData)
                    db.child("locations").push(gpsData) 
                    x,y,z = geodetic_to_ecef(data[0],data[1],data[2])
                    gpsEcefData = {
                        "x": x,
                        "y": y,
                        "z": z,
                        "timestamp": time.time()
                    }
                    #db.child("locationEcef").update(gpsEcefData)
                    
            except UnicodeDecodeError as e:
                print(f"Error decoding serial data: {e}")
        time.sleep(0.25)

# Start the thread
gps_thread = threading.Thread(target=update_gps_data, daemon=True)
gps_thread.start()

save_log("info","GPS Thread is Running now")


distance = [0,0,0]

def update_us():
    global distances
    while True:
        distances = [sensor.measure_distance() for sensor in ultrasonics]
        db.child('us_2').set(distances[0])
        db.child('us_1').set(distances[1])
        db.child('us_3').set(distances[2])
        time.sleep(0.001)  # Update every 1ms

# Start the thread
update_us_thread = threading.Thread(target=update_us, daemon=True)
update_us_thread.start()
save_log("info","Ultrasonic Thread is Running now")


# Function to initiate obstacle avoidance
def obstacle_avoidance(threshold,speed,turn_speed):
    global car_speed,distances
    save_log("info","Car is running now on obstacle avoidance mode")
    db.child("moving").set(1)
    
    while True:
        # distance = ultrasonics[1].measure_distance()        
        if distances[1] <= threshold:
            car_speed=turn_speed
            send_command("s,0")
        else:
            car_speed=speed
            send_command("f,"+str(speed))
        time.sleep(0.001)
        
def turn_car(target_angle=mpu_yaw, speed=70):
    global car_speed
    car_speed = speed
    save_log("info","Car is turning now on"+str(target_angle)+" angle")
    db.child("moving").set(1)
    while True:    
        if mpu_yaw > (target_angle + 1):  # If the car needs to turn right
            send_command("r,"+str(speed))  # Turn the car to the right
        elif mpu_yaw < (target_angle - 1):  # If the car needs to turn left
            send_command("l,"+str(speed))  # Turn the car to the left
        else:
            car_speed = 0
            db.child("moving").set(0)
            send_command("s,0")  # Stop the car when the desired angle is reached
            break
            
        time.sleep(0.01)  # Brief delay for stability

def move_distance(distance=0,speed=0):
    db.child("moving").set(1)
    global movedDistance,steps,car_speed
    car_speed = speed
    steps=0
    send_command('f,'+str(speed))
    while(movedDistance < distance):
        pass
    car_speed = 0
    send_command('s,0')
    db.child("moving").set(0)
    

def executePath(x=50,y=50,speed=50,turnSpeed=70):
    global movedDistance
    diameter, angle = calculate_diameter_and_angle(x, y)
    turn_car(angle,turnSpeed)
    movedDistance=0
    move_distance(diameter,speed)
    
def set_car_path(angle=mpu_yaw,distance=50,speed=50,turnSpeed=70):
    global steps
    steps = 0
    turn_car(angle,turnSpeed)
    move_distance(distance,speed)

def perform_sequence(shape = 's', length=50, speed=60,turnSpeed=70, width=50):
    if shape == 's':
        move_square(length,speed,turnSpeed)
    elif shape == 'r':
        move_rectangle(length, width, speed,turnSpeed)
    elif shape == 't':
        move_triangle(length, speed,turnSpeed)
    else:
        save_log("error","Invalid Shape for Sequence")

def move_square(length, speed,turnSpeed):
    set_car_path(mpu_yaw, length, speed,turnSpeed)
    set_car_path(mpu_yaw - 90, length, speed,turnSpeed)
    set_car_path(mpu_yaw - 90, length, speed,turnSpeed)
    set_car_path(mpu_yaw - 90, length, speed,turnSpeed)

def move_triangle(length, speed,turnSpeed):
    set_car_path(mpu_yaw, length, speed,turnSpeed)
    set_car_path(mpu_yaw - 120, length, speed,turnSpeed)
    set_car_path(mpu_yaw - 120, length, speed,turnSpeed)

def move_rectangle(length, width, speed,turnSpeed):
    set_car_path(mpu_yaw, length, speed,turnSpeed)
    set_car_path(mpu_yaw - 90, width, speed,turnSpeed)
    set_car_path(mpu_yaw - 90, length, speed,turnSpeed)
    set_car_path(mpu_yaw - 90, width, speed,turnSpeed)


serial_lock = threading.Lock()
def check_command(command, timeout=0.5):
    with serial_lock:
        # Flush the input buffer before sending the command
        ser.reset_input_buffer()
        
        # Send the command via serial
        ser.write((command + "\n").encode())

        # Wait for the response for up to 'timeout' seconds
        start_time = time.time()
        while ser.in_waiting > 0:
            ser.readline()
        while time.time() - start_time < timeout:
            if ser.in_waiting > 0:  # Check if data is available
                response = ser.readline().decode().strip()
                return response
            time.sleep(0.01)

        return "none"


# Reference to the desired node
command_ref = db.child('rpi_command')

# Define a listener function
def listener(event):
    res=check_command(event['data'])
    print(res)
    db.child('arduino_echo').set(res)

# Attach the listener to the reference
stream = command_ref.stream(listener)


# Main control logic
def control_logic():
    try:
        save_log("warning","Waiting for a command")
        while 1:
            time.sleep(0.1)
            
    except Exception as e:
        print(f"Control logic error: {e}")


# Main function
if __name__ == "__main__":
    try:
        # Wait until there is an internet connection
        while not check_internet():
            time.sleep(0.5)  # Check every 0.5 seconds
            
        # Start control logic
        control_logic()
        while(1):
            pass
    except KeyboardInterrupt:
        save_log("warning","Program interrupted by user.")
    finally:
        for sensor in ultrasonics:
            sensor.cleanup()
        if ser.is_open:
            ser.close()
        save_log("warning","Program terminated.")
        
