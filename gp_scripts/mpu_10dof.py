import smbus
import math
import time

class MPU10DOF:
    def __init__(self, bus_number=1, mpu_address=0x68, mag_address=0x0C):
        """
        Initialize MPU10DOF with MPU6050/9250 and magnetometer.
        
        Args:
            bus_number (int): I2C bus number.
            mpu_address (int): I2C address of the MPU6050/9250.
            mag_address (int): I2C address of the magnetometer (AK8963).
        """
        self.bus = smbus.SMBus(bus_number)
        self.mpu_address = mpu_address
        self.mag_address = mag_address

        self.GyroErrorZ = 0  # Gyroscope Z bias
        self.alpha = 0.98    # Complementary filter constant
        self.yaw = 0.0       # Yaw angle

        self._initialize_mpu()
        self._initialize_magnetometer()

    def _initialize_mpu(self):
        """
        Initialize MPU registers.
        """
        self.bus.write_byte_data(self.mpu_address, 0x6B, 0)  # Wake up MPU
        self.bus.write_byte_data(self.mpu_address, 0x1B, 24)  # Set gyroscope range to ±2000°/s

    def _initialize_magnetometer(self):
        """
        Initialize the AK8963 magnetometer.
        """
        try:
            # Enable bypass mode for direct access to the magnetometer
            self.bus.write_byte_data(self.mpu_address, 0x37, 0x02)
            time.sleep(0.01)

            # Check if the magnetometer is accessible
            who_am_i = self.bus.read_byte_data(self.mag_address, 0x00)
            if who_am_i != 0x48:
                raise RuntimeError("Magnetometer not detected (WHO_AM_I mismatch).")

            # Power up the magnetometer
            self.bus.write_byte_data(self.mag_address, 0x0A, 0x01)
            time.sleep(0.01)

        except OSError as e:
            raise RuntimeError(f"Failed to initialize magnetometer: {e}")

    def _convert_to_signed(self, value):
        """
        Convert a 16-bit value to signed.
        """
        return value if value < 0x8000 else value - 0x10000

    def _read_mpu_gyro(self):
        """
        Read gyroscope data from MPU6050/9250.
        
        Returns:
            float: Gyroscope Z-axis angular velocity (degrees per second).
        """
        data = self.bus.read_i2c_block_data(self.mpu_address, 0x43, 6)
        gyro_z = self._convert_to_signed(data[4] << 8 | data[5]) / 131.0
        return gyro_z

    def _read_magnetometer(self):
        """
        Read magnetometer data (AK8963).
        
        Returns:
            tuple: (MagX, MagY) values in microteslas.
        """
        data = self.bus.read_i2c_block_data(self.mag_address, 0x03, 7)
        mag_x = self._convert_to_signed(data[1] << 8 | data[0])
        mag_y = self._convert_to_signed(data[3] << 8 | data[2])
        return mag_x, mag_y

    def calibrate(self, samples=200):
        """
        Calibrate the gyroscope Z-axis for bias.
        
        Args:
            samples (int): Number of samples for calibration.
        """
        print("Calibrating sensor...")
        for _ in range(samples):
            self.GyroErrorZ += self._read_mpu_gyro()
            time.sleep(0.01)
        self.GyroErrorZ /= samples
        print(f"Gyroscope bias (Z-axis): {self.GyroErrorZ:.2f}°/s")

    def calculate_yaw(self, delta_time=0.01):
        """
        Calculate the yaw angle using gyroscope and magnetometer.
        
        Args:
            delta_time (float): Time interval for integration.
        
        Returns:
            float: Yaw angle in degrees.
        """
        # Gyroscope integration
        gyro_z = self._read_mpu_gyro() - self.GyroErrorZ
        gyro_yaw = self.yaw + gyro_z * delta_time

        # Magnetometer calculation
        mag_x, mag_y = self._read_magnetometer()
        magnetometer_yaw = math.atan2(mag_y, mag_x) * (180 / math.pi)

        # Normalize yaw from magnetometer to 0-360
        if magnetometer_yaw < 0:
            magnetometer_yaw += 360

        # Complementary filter
        self.yaw = self.alpha * gyro_yaw + (1 - self.alpha) * magnetometer_yaw

        # Normalize yaw to 0-360
        self.yaw %= 360

        return self.yaw
