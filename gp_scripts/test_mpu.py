from mpu6050 import MPU6050
import time

def main():
    # Initialize MPU6050
    mpu = MPU6050()
    
    try:
        while True:
            # Get stable yaw angle
            yaw = mpu.get_stable_yaw()
            print(f"Yaw angle: {yaw:.2f}°")
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nProgram stopped by user")

if __name__ == "__main__":
    main()