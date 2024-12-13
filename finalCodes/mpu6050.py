import smbus
import math
import time
from typing import Tuple, Dict
from collections import deque

class MPU6050:
    # MPU6050 Registers
    PWR_MGMT_1 = 0x6B
    SMPLRT_DIV = 0x19
    CONFIG = 0x1A
    GYRO_CONFIG = 0x1B
    ACCEL_CONFIG = 0x1C
    INT_ENABLE = 0x38
    ACCEL_XOUT_H = 0x3B
    ACCEL_YOUT_H = 0x3D
    ACCEL_ZOUT_H = 0x3F
    GYRO_XOUT_H = 0x43
    GYRO_YOUT_H = 0x45
    GYRO_ZOUT_H = 0x47

    def __init__(self, bus: int = 1, device_address: int = 0x68):
        self.bus = smbus.SMBus(bus)
        self.device_address = device_address
        self.gyro_scale = 131.0  # For 250deg/s
        self.accel_scale = 16384.0  # For ±2g
        
        # Enhanced stability parameters
        self.deadzone = 0.25  # Adjusted deadzone
        self.motion_threshold = 0.12
        self.stable_threshold = 0.05
        self.stable_count = 0
        self.stable_count_threshold = 15
        
        # Orientation parameters
        self.pitch = 0
        self.roll = 0
        self.initial_yaw = 0
        self.yaw_offset = 0
        
        # Moving average filter
        self.yaw_history = deque(maxlen=5)
        
        # Kalman filter variables
        self.Q_angle = 0.008
        self.Q_bias = 0.003
        self.R_measure = 0.03
        self.angle = 0
        self.bias = 0
        self.P = [[0, 0], [0, 0]]
        
        # Calibration offsets
        self.gyro_offset = {'x': 0, 'y': 0, 'z': 0}
        self.accel_offset = {'x': 0, 'y': 0, 'z': 0}
        self.last_time = time.time()
        self.yaw = 0
        self.last_yaw = 0
        
        self._setup()
        self.calibrate_sensors()

    def _setup(self):
        # Wake up the MPU6050
        self.bus.write_byte_data(self.device_address, self.PWR_MGMT_1, 0)
        time.sleep(0.1)
        
        # Configure gyroscope (250deg/s)
        self.bus.write_byte_data(self.device_address, self.GYRO_CONFIG, 0)
        
        # Configure accelerometer (±2g)
        self.bus.write_byte_data(self.device_address, self.ACCEL_CONFIG, 0)
        
        # Set sample rate to 1kHz
        self.bus.write_byte_data(self.device_address, self.SMPLRT_DIV, 7)
        
        # Enhanced DLPF setting (5Hz)
        self.bus.write_byte_data(self.device_address, self.CONFIG, 6)

    def _read_word(self, register: int) -> int:
        high = self.bus.read_byte_data(self.device_address, register)
        low = self.bus.read_byte_data(self.device_address, register + 1)
        value = (high << 8) + low
        
        if value >= 0x8000:
            value = -((65535 - value) + 1)
        
        return value

    def read_accel_data(self) -> Tuple[float, float, float]:
        x = self._read_word(self.ACCEL_XOUT_H) / self.accel_scale - self.accel_offset['x']
        y = self._read_word(self.ACCEL_YOUT_H) / self.accel_scale - self.accel_offset['y']
        z = self._read_word(self.ACCEL_ZOUT_H) / self.accel_scale - self.accel_offset['z']
        return (x, y, z)

    def read_gyro_data(self) -> Tuple[float, float, float]:
        x = self._read_word(self.GYRO_XOUT_H) / self.gyro_scale - self.gyro_offset['x']
        y = self._read_word(self.GYRO_YOUT_H) / self.gyro_scale - self.gyro_offset['y']
        z = self._read_word(self.GYRO_ZOUT_H) / self.gyro_scale - self.gyro_offset['z']
        return (x, y, z)

    def calibrate_sensors(self):
        # Initialize sums
        accel_sums = {'x': 0, 'y': 0, 'z': 0}
        gyro_sums = {'x': 0, 'y': 0, 'z': 0}
        samples = 300  # Increased samples for better accuracy
        
        # Collect calibration data
        for _ in range(samples):
            ax, ay, az = self.read_accel_data()
            gx, gy, gz = self.read_gyro_data()
            
            accel_sums['x'] += ax
            accel_sums['y'] += ay
            accel_sums['z'] += az
            gyro_sums['x'] += gx
            gyro_sums['y'] += gy
            gyro_sums['z'] += gz
            
            time.sleep(0.01)
        
        # Calculate offsets
        self.accel_offset = {
            'x': accel_sums['x'] / samples,
            'y': accel_sums['y'] / samples,
            'z': (accel_sums['z'] / samples) - 1.0  # Account for gravity
        }
        
        self.gyro_offset = {
            'x': gyro_sums['x'] / samples,
            'y': gyro_sums['y'] / samples,
            'z': gyro_sums['z'] / samples
        }
        
        # Calculate initial orientation
        ax, ay, az = self.read_accel_data()
        self.pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))
        self.roll = math.atan2(ay, az)
        self.initial_yaw = math.atan2(ay, ax)
        self.yaw_offset = math.degrees(self.initial_yaw)
              
        # Set current position as forward (0 degrees)
        _, _, gz = self.read_gyro_data()
        self.yaw = 0
        self.last_yaw = 0
        self.yaw_history.clear()
        for _ in range(self.yaw_history.maxlen):
            self.yaw_history.append(0)

    def get_tilt_compensated_yaw(self, gz: float, ax: float, ay: float, az: float) -> float:
        # Update pitch and roll
        self.pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))
        self.roll = math.atan2(ay, az)
        
        # Compensate gyro reading for tilt
        cos_pitch = math.cos(self.pitch)
        sin_pitch = math.sin(self.pitch)
        cos_roll = math.cos(self.roll)
        sin_roll = math.sin(self.roll)
        
        gz_compensated = gz * cos_pitch * cos_roll + \
                        gz * sin_pitch * sin_roll
        
        return gz_compensated

    def moving_average(self, new_value: float) -> float:
        self.yaw_history.append(new_value)
        return sum(self.yaw_history) / len(self.yaw_history)

    def get_stable_yaw(self) -> float:
        current_time = time.time()
        dt = current_time - self.last_time
        
        # Read sensor data
        ax, ay, az = self.read_accel_data()
        _, _, gz = self.read_gyro_data()
        
        # Get tilt-compensated yaw rate
        gz_compensated = self.get_tilt_compensated_yaw(gz, ax, ay, az)
        
        # Apply deadzone to gyro data
        if abs(gz_compensated) < self.deadzone:
            gz_compensated = 0
        
        # Enhanced motion detection
        total_accel = math.sqrt(ax*ax + ay*ay + az*az)
        is_stationary = (abs(total_accel - 1.0) < self.motion_threshold and 
                        abs(gz_compensated) < self.motion_threshold)
        
        # Drift compensation
        if is_stationary:
            self.stable_count += 1
            if self.stable_count > self.stable_count_threshold:
                gz_compensated = 0
                if abs(self.yaw - self.last_yaw) < self.stable_threshold:
                    self.yaw = self.last_yaw
        else:
            self.stable_count = 0
            self.last_yaw = self.yaw
        
        # Update yaw angle
        if gz_compensated != 0:
            self.yaw += gz_compensated * dt
        
        # Apply moving average filter
        smoothed_yaw = self.moving_average(self.yaw)
        
        # Normalize to -360 to 360 degrees
        if smoothed_yaw < -360:
            smoothed_yaw += 360
        if smoothed_yaw > 360:
            smoothed_yaw -=360
        
        self.last_time = current_time
        return round(smoothed_yaw, 2)
