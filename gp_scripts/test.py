from mpu_lib import MPU6050
import time

def main():
    # Initialize the MPU6050 sensor
    mpu = MPU6050()

    # Calibrate the sensor
    print("Calibrating sensor...")
    mpu.calculate_IMU_error()
    print("Calibration complete!")

    # Continuously read sensor data
    print("Reading data...")
    try:
        while True:
            roll, pitch, yaw = mpu.update_mpu_readings()
            print(f"Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nExiting...")

if __name__ == "__main__":
    main()
