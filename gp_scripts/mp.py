import time
import smbus
import math
import numpy as np

class MPU6050:
    def __init__(self):
        self.bus = smbus.SMBus(1)
        self.address = 0x68
        
        # Sensitivity scale factors
        self.accel_scale = 16384.0  # For ±2g
        self.gyro_scale = 131.0     # For ±250°/s
        
        # Error compensation
        self.AccErrorX = 0
        self.AccErrorY = 0
        self.GyroErrorX = 0
        self.GyroErrorY = 0
        self.GyroErrorZ = 0
        
        # Timing variables
        self.prev_time = time.time()
        self.current_time = self.prev_time
        
        # Angle variables
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.gyro_angle_x = 0
        self.gyro_angle_y = 0
        
        # Initialize the sensor
        self.init_mpu()
        self.calculate_IMU_error()

    def init_mpu(self):
        # Wake up the MPU6050
        self.bus.write_byte_data(self.address, 0x6B, 0)
        time.sleep(0.1)
        # Configure gyroscope range (±250°/s)
        self.bus.write_byte_data(self.address, 0x1B, 0x00)
        # Configure accelerometer range (±2g)
        self.bus.write_byte_data(self.address, 0x1C, 0x00)

    def read_raw_data(self):
        # Read accelerometer data
        accel_data = self.bus.read_i2c_block_data(self.address, 0x3B, 6)
        ax = (accel_data[0] << 8 | accel_data[1]) / self.accel_scale
        ay = (accel_data[2] << 8 | accel_data[3]) / self.accel_scale
        az = (accel_data[4] << 8 | accel_data[5]) / self.accel_scale

        # Read gyroscope data
        gyro_data = self.bus.read_i2c_block_data(self.address, 0x43, 6)
        gx = (gyro_data[0] << 8 | gyro_data[1]) / self.gyro_scale
        gy = (gyro_data[2] << 8 | gyro_data[3]) / self.gyro_scale
        gz = (gyro_data[4] << 8 | gyro_data[5]) / self.gyro_scale

        return ax, ay, az, gx, gy, gz

    def calculate_IMU_error(self):
        print("Calculating IMU errors... Please keep the sensor still.")
        acc_x_sum = acc_y_sum = 0
        gyro_x_sum = gyro_y_sum = gyro_z_sum = 0
        samples = 200

        for _ in range(samples):
            ax, ay, az, gx, gy, gz = self.read_raw_data()
            
            # Accumulate accelerometer angles
            acc_x_sum += math.atan(ay / math.sqrt(ax**2 + az**2)) * 180 / math.pi
            acc_y_sum += math.atan(-1 * ax / math.sqrt(ay**2 + az**2)) * 180 / math.pi
            
            # Accumulate gyroscope readings
            gyro_x_sum += gx
            gyro_y_sum += gy
            gyro_z_sum += gz
            
            time.sleep(0.003)

        # Calculate average errors
        self.AccErrorX = acc_x_sum / samples
        self.AccErrorY = acc_y_sum / samples
        self.GyroErrorX = gyro_x_sum / samples
        self.GyroErrorY = gyro_y_sum / samples
        self.GyroErrorZ = gyro_z_sum / samples
        
        print("IMU calibration complete!")

    def update(self):
        # Read sensor data
        ax, ay, az, gx, gy, gz = self.read_raw_data()
        
        # Calculate accelerometer angles
        acc_angle_x = (math.atan(ay / math.sqrt(ax**2 + az**2)) * 180 / math.pi) - self.AccErrorX
        acc_angle_y = (math.atan(-1 * ax / math.sqrt(ay**2 + az**2)) * 180 / math.pi) - self.AccErrorY
        
        # Update timing
        self.current_time = time.time()
        dt = self.current_time - self.prev_time
        self.prev_time = self.current_time
        
        # Correct gyroscope readings
        gx -= self.GyroErrorX
        gy -= self.GyroErrorY
        gz -= self.GyroErrorZ
        
        # Calculate gyroscope angles
        self.gyro_angle_x += gx * dt
        self.gyro_angle_y += gy * dt
        self.yaw += gz * dt
        
        # Complementary filter for roll and pitch
        self.roll = 0.96 * self.gyro_angle_x + 0.04 * acc_angle_x
        self.pitch = 0.96 * self.gyro_angle_y + 0.04 * acc_angle_y
        
        # Normalize yaw to 0-360 degrees
        self.yaw = self.yaw % 360
        if self.yaw < 0:
            self.yaw += 360
            
        return self.roll, self.pitch, self.yaw

def main():
    try:
        mpu = MPU6050()
        print("MPU6050 initialized. Starting measurements...")
        
        while True:
            roll, pitch, yaw = mpu.update()
            print(f"Roll: {roll:6.1f}° Pitch: {pitch:6.1f}° Yaw: {yaw:6.1f}°")
            time.sleep(0.02)  # 50Hz update rate
            
    except KeyboardInterrupt:
        print("\nProgram terminated by user")
    except Exception as e:
        print(f"Error: {str(e)}")

if __name__ == "__main__":
    main()
