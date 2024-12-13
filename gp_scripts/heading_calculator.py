import math
import numpy as np

class HeadingCalculator:
    def __init__(self):
        self.prev_time = None
        self.yaw = 0.0
        self.Q = np.array([[0.001, 0], [0, 0.003]])  # Process noise
        self.R = 0.03  # Measurement noise
        self.P = np.zeros((2, 2))  # Error covariance
        self.x = np.zeros(2)  # State vector [angle, bias]
        self.gyro_only = False

    def update(self, mag_x, mag_y, gyro_z, dt):
        """
        Calculate heading using magnetometer and gyroscope fusion
        Falls back to gyro-only integration if magnetometer data is unavailable
        """
        # If no valid magnetometer data, switch to gyro-only mode
        if mag_x == 0 and mag_y == 0:
            if not self.gyro_only:
                print("Switching to gyro-only mode")
                self.gyro_only = True
            
            # Simple gyro integration
            self.yaw += gyro_z * dt
            # Normalize to 0-360
            self.yaw = self.yaw % 360
            return self.yaw
            
        # If we have valid magnetometer data, use sensor fusion
        self.gyro_only = False
        
        # Calculate magnetic heading
        mag_heading = math.atan2(mag_y, mag_x) * 180 / math.pi
        
        # Normalize to 0-360 degrees
        if mag_heading < 0:
            mag_heading += 360
            
        # Prediction step
        self.x[0] += dt * (gyro_z - self.x[1])
        self.P += dt * np.array([[self.P[1,1], -self.P[1,1]], 
                                [-self.P[1,1], self.P[1,1]]]) + self.Q
        
        # Update step
        y = mag_heading - self.x[0]
        # Normalize the difference
        if y > 180:
            y -= 360
        elif y < -180:
            y += 360
            
        S = self.P[0,0] + self.R
        K = self.P[:, 0] / S
        
        self.x += K * y
        self.P -= np.outer(K, K) * S
        
        # Normalize final angle to 0-360
        self.yaw = self.x[0] % 360
        
        return self.yaw
