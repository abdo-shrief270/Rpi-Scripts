import smbus2
import time
import math

class MPU6050:
    def __init__(self, bus_number=1, mpu_address=0x68):
        self.bus = smbus2.SMBus(bus_number)
        self.MPU = mpu_address
        self.AccErrorX = 0
        self.AccErrorY = 0
        self.GyroErrorX = 0
        self.GyroErrorY = 0
        self.GyroErrorZ = 0
        self.gyroAngleX = 0
        self.gyroAngleY = 0
        self.yaw = 0
        self.previousTime = time.time()
        self._initialize_mpu()

    def _read_mpu_registers(self, register, length):
        """Reads data from the specified MPU register."""
        data = self.bus.read_i2c_block_data(self.MPU, register, length)
        return data

    def _initialize_mpu(self):
        """Initializes the MPU-6050 by waking it up from sleep mode."""
        self.bus.write_byte_data(self.MPU, 0x6B, 0)  # Wake up MPU-6050

    def calculate_IMU_error(self):
        """Calculates accelerometer and gyroscope calibration errors."""
        # Calculate accelerometer error
        for _ in range(200):
            data = self._read_mpu_registers(0x3B, 6)
            AccX = (data[0] << 8 | data[1]) / 16384.0
            AccY = (data[2] << 8 | data[3]) / 16384.0
            AccZ = (data[4] << 8 | data[5]) / 16384.0

            # Avoid division by zero
            denominator1 = math.sqrt(AccX**2 + AccZ**2)
            if denominator1 == 0:
                continue  # Skip iteration if denominator is zero

            self.AccErrorX += math.atan(AccY / denominator1) * 180 / math.pi
            self.AccErrorY += math.atan(-AccX / denominator1) * 180 / math.pi

        self.AccErrorX /= 200
        self.AccErrorY /= 200

        # Calculate gyroscope error
        for _ in range(200):
            data = self._read_mpu_registers(0x43, 6)
            GyroX = (data[0] << 8 | data[1])
            GyroY = (data[2] << 8 | data[3])
            GyroZ = (data[4] << 8 | data[5])

            self.GyroErrorX += GyroX / 131.0
            self.GyroErrorY += GyroY / 131.0
            self.GyroErrorZ += GyroZ / 131.0

        self.GyroErrorX /= 200
        self.GyroErrorY /= 200
        self.GyroErrorZ /= 200

    def update_mpu_readings(self):
        """Updates and calculates roll, pitch, and yaw."""
        # Read accelerometer data
        data = self._read_mpu_registers(0x3B, 6)
        AccX = (data[0] << 8 | data[1]) / 16384.0
        AccY = (data[2] << 8 | data[3]) / 16384.0
        AccZ = (data[4] << 8 | data[5]) / 16384.0

        accAngleX = math.atan(AccY / math.sqrt(AccX**2 + AccZ**2)) * 180 / math.pi - self.AccErrorX
        accAngleY = math.atan(-AccX / math.sqrt(AccY**2 + AccZ**2)) * 180 / math.pi - self.AccErrorY

        # Calculate elapsed time
        currentTime = time.time()
        elapsedTime = currentTime - self.previousTime
        self.previousTime = currentTime

        # Read gyroscope data
        data = self._read_mpu_registers(0x43, 6)
        GyroX = (data[0] << 8 | data[1]) / 131.0
        GyroY = (data[2] << 8 | data[3]) / 131.0
        GyroZ = (data[4] << 8 | data[5]) / 131.0

        GyroX -= self.GyroErrorX
        GyroY -= self.GyroErrorY
        GyroZ -= self.GyroErrorZ

        # Integrate gyroscope data
        self.gyroAngleX += GyroX * elapsedTime
        self.gyroAngleY += GyroY * elapsedTime
        self.yaw += GyroZ * elapsedTime

        # Complementary filter
        roll = 0.96 * self.gyroAngleX + 0.04 * accAngleX
        pitch = 0.96 * self.gyroAngleY + 0.04 * accAngleY

        return roll, pitch, self.yaw
