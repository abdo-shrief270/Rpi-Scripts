from imu_sensor import IMU10DOF
from heading_calculator import HeadingCalculator
import time

def main():
    imu = IMU10DOF()
    heading_calc = HeadingCalculator()
    
    print("Initializing sensors...")
    try:
        imu.init_sensors()
        time.sleep(1)
        
        if imu.mag_available:
            print("Calibrating magnetometer...")
            imu.calibrate_magnetometer()
        
        prev_time = time.time()
        print("\nStarting heading measurements...")
        
        while True:
            try:
                # Read sensor data
                ax, ay, az, gx, gy, gz = imu.read_mpu()
                mx, my, mz = imu.read_magnetometer()
                
                # Calculate time delta
                current_time = time.time()
                dt = current_time - prev_time
                prev_time = current_time
                
                # Calculate heading
                heading = heading_calc.update(mx, my, gz, dt)
                
                # Get heading mode
                mode = "Gyro Only" if heading_calc.gyro_only else "Fusion"
                
                # Print heading (0-360 degrees, where 0/360 is North)
                print(f"Heading: {heading:.1f}° [{mode}] | " + 
                      f"{'N' if 337.5 <= heading <= 360 or 0 <= heading < 22.5 else ''}" +
                      f"{'NE' if 22.5 <= heading < 67.5 else ''}" +
                      f"{'E' if 67.5 <= heading < 112.5 else ''}" +
                      f"{'SE' if 112.5 <= heading < 157.5 else ''}" +
                      f"{'S' if 157.5 <= heading < 202.5 else ''}" +
                      f"{'SW' if 202.5 <= heading < 247.5 else ''}" +
                      f"{'W' if 247.5 <= heading < 292.5 else ''}" +
                      f"{'NW' if 292.5 <= heading < 337.5 else ''}")
                
                time.sleep(0.1)  # 10Hz update rate
                
            except Exception as e:
                print(f"Error reading sensors: {str(e)}")
                time.sleep(1)  # Wait before retrying
                
    except KeyboardInterrupt:
        print("\nProgram terminated by user")
    except Exception as e:
        print(f"Fatal error: {str(e)}")

if __name__ == "__main__":
    main()
