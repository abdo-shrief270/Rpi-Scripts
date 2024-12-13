import time
import smbus
import math
import numpy as np

class IMU10DOF:
    def __init__(self):
        self.bus = smbus.SMBus(1)
        self.mpu_address = 0x68
        self.mag_address = 0x0C  # Default HMC5883L address
        self.mag_available = False
        
        # Sensitivities
        self.accel_sensitivity = 16384.0  # For ±2g
        self.gyro_sensitivity = 131.0     # For ±250°/s
        self.mag_sensitivity = 0.15       # Magnetometer sensitivity in µT/LSB

        # Calibration offsets
        self.mag_offset_x = 0
        self.mag_offset_y = 0
        self.mag_offset_z = 0
        
        # Declination angle for your location (adjust this!)
        self.declination = 0  # Set your local magnetic declination in degrees

    def detect_magnetometer(self):
        """Try to detect the magnetometer by scanning common I2C addresses"""
        common_mag_addresses = [0x0C, 0x0D, 0x1E]  # Common addresses for HMC5883L, QMC5883L
        
        for addr in common_mag_addresses:
            try:
                self.mag_address = addr
                # Try to read from the device
                self.bus.read_byte_data(self.mag_address, 0x00)
                print(f"Magnetometer found at address 0x{addr:02X}")
                self.mag_available = True
                return True
            except Exception as e:
                continue
        
        print("Warning: Magnetometer not found! Using gyro-only mode.")
        self.mag_available = False
        return False

    def init_sensors(self):
        # Initialize MPU6050
        try:
            # Wake up the MPU6050
            self.bus.write_byte_data(self.mpu_address, 0x6B, 0)
            time.sleep(0.1)
            # Configure gyroscope range (±250°/s)
            self.bus.write_byte_data(self.mpu_address, 0x1B, 0x00)
            # Configure accelerometer range (±2g)
            self.bus.write_byte_data(self.mpu_address, 0x1C, 0x00)
            print("MPU6050 initialized successfully")
        except Exception as e:
            print(f"Error initializing MPU6050: {str(e)}")
            raise
        
        # Try to initialize magnetometer
        if self.detect_magnetometer():
            try:
                # Initialize magnetometer (adjust registers based on your specific magnetometer)
                self.bus.write_byte_data(self.mag_address, 0x0A, 0x12)  # Example: Set continuous measurement mode
                print("Magnetometer initialized successfully")
            except Exception as e:
                print(f"Warning: Could not initialize magnetometer: {str(e)}")
                self.mag_available = False

    def read_magnetometer(self):
        """Read magnetometer data with error handling"""
        if not self.mag_available:
            return 0, 0, 0
            
        try:
            data = self.bus.read_i2c_block_data(self.mag_address, 0x03, 6)
            
            # Convert to µT and apply calibration
            x = ((data[1] << 8) | data[0]) * self.mag_sensitivity - self.mag_offset_x
            y = ((data[3] << 8) | data[2]) * self.mag_sensitivity - self.mag_offset_y
            z = ((data[5] << 8) | data[4]) * self.mag_sensitivity - self.mag_offset_z
            
            return x, y, z
        except Exception as e:
            print(f"Warning: Error reading magnetometer: {str(e)}")
            return 0, 0, 0

    def read_mpu(self):
        """Read MPU6050 data with error handling"""
        try:
            accel_data = self.bus.read_i2c_block_data(self.mpu_address, 0x3B, 6)
            gyro_data = self.bus.read_i2c_block_data(self.mpu_address, 0x43, 6)

            # Convert accelerometer values
            ax = (accel_data[0] << 8 | accel_data[1]) / self.accel_sensitivity
            ay = (accel_data[2] << 8 | accel_data[3]) / self.accel_sensitivity
            az = (accel_data[4] << 8 | accel_data[5]) / self.accel_sensitivity

            # Convert gyroscope values
            gx = (gyro_data[0] << 8 | gyro_data[1]) / self.gyro_sensitivity
            gy = (gyro_data[2] << 8 | gyro_data[3]) / self.gyro_sensitivity
            gz = (gyro_data[4] << 8 | gyro_data[5]) / self.gyro_sensitivity

            return ax, ay, az, gx, gy, gz
        except Exception as e:
            print(f"Error reading MPU6050: {str(e)}")
            raise

    def calibrate_magnetometer(self):
        """Calibrate magnetometer if available"""
        if not self.mag_available:
            print("Magnetometer not available, skipping calibration")
            return
            
        print("Calibrating magnetometer... Rotate the sensor in all directions...")
        samples = []
        start_time = time.time()
        
        # Collect samples for 20 seconds
        while time.time() - start_time < 20:
            x, y, z = self.read_magnetometer()
            if x != 0 or y != 0 or z != 0:  # Only add valid readings
                samples.append((x, y, z))
            time.sleep(0.1)
        
        if not samples:
            print("No valid magnetometer readings during calibration")
            self.mag_available = False
            return
            
        # Calculate offsets
        x_samples, y_samples, z_samples = zip(*samples)
        self.mag_offset_x = (max(x_samples) + min(x_samples)) / 2
        self.mag_offset_y = (max(y_samples) + min(y_samples)) / 2
        self.mag_offset_z = (max(z_samples) + min(z_samples)) / 2
        
        print("Calibration complete!")
