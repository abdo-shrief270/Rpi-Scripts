import time
import smbus
import math

# I2C bus (1 for Raspberry Pi 4)
bus = smbus.SMBus(1)

# MPU6050 Address
MPU_ADDRESS = 0x68

# Accelerometer and Gyroscope Sensitivity
ACCEL_SENSITIVITY = 16384.0
GYRO_SENSITIVITY = 131.0

# Initialize MPU6050
def init_mpu():
    bus.write_byte_data(MPU_ADDRESS, 0x6B, 0)  # Wake up the MPU6050

def read_accel_gyro():
    # Read accelerometer and gyroscope data (6 bytes each)
    accel_data = bus.read_i2c_block_data(MPU_ADDRESS, 0x3B, 6)
    gyro_data = bus.read_i2c_block_data(MPU_ADDRESS, 0x43, 6)
    
    # Process accelerometer data
    ax = (accel_data[0] << 8 | accel_data[1]) / ACCEL_SENSITIVITY
    ay = (accel_data[2] << 8 | accel_data[3]) / ACCEL_SENSITIVITY
    az = (accel_data[4] << 8 | accel_data[5]) / ACCEL_SENSITIVITY
    
    # Process gyroscope data
    gx = (gyro_data[0] << 8 | gyro_data[1]) / GYRO_SENSITIVITY
    gy = (gyro_data[2] << 8 | gyro_data[3]) / GYRO_SENSITIVITY
    gz = (gyro_data[4] << 8 | gyro_data[5]) / GYRO_SENSITIVITY
    
    return ax, ay, az, gx, gy, gz

# Initialize MPU6050
init_mpu()

# Variables to store previous readings
acc_error_x = 0.0
acc_error_y = 0.0
gyro_error_x = 0.0
gyro_error_y = 0.0
gyro_error_z = 0.0

# Time variables for gyroscope integration
prev_time = time.time()
gyro_angle_x = 0.0
gyro_angle_y = 0.0
yaw = 0.0

# Main loop to continuously update readings
while True:
    # Read accelerometer and gyroscope data
    ax, ay, az, gx, gy, gz = read_accel_gyro()
    
    # Calculate roll and pitch from accelerometer data
    acc_angle_x = math.atan(ay / math.sqrt(ax**2 + az**2)) * 180 / math.pi
    acc_angle_y = math.atan(-ax / math.sqrt(ay**2 + az**2)) * 180 / math.pi
    
    # Apply accelerometer error correction
    acc_angle_x -= acc_error_x
    acc_angle_y -= acc_error_y
    
    # Get elapsed time for gyroscope integration
    current_time = time.time()
    elapsed_time = current_time - prev_time
    prev_time = current_time
    
    # Integrate gyroscope data to calculate angles
    gyro_angle_x += gx * elapsed_time
    gyro_angle_y += gy * elapsed_time
    yaw += gz * elapsed_time
    
    # Apply gyro error correction
    gyro_angle_x -= gyro_error_x
    gyro_angle_y -= gyro_error_y
    yaw -= gyro_error_z
    
    # Use a complementary filter to combine accelerometer and gyroscope angles
    roll = 0.96 * gyro_angle_x + 0.04 * acc_angle_x
    pitch = 0.96 * gyro_angle_y + 0.04 * acc_angle_y
    
    # Print the current angles (in degrees)
    print(f"Roll: {roll:.2f}° Pitch: {pitch:.2f}° Yaw: {yaw:.2f}°")
    
    time.sleep(0.1)
