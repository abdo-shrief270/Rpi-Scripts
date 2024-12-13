import serial
import time

# Setup serial connection with Arduino (through Raspberry Pi's UART pins 8 and 9)
ser = serial.Serial('/dev/ttyS0', 9600, timeout=1)  # Open serial port
time.sleep(2)  # Allow time for the serial connection to establish

# Function to read data from Arduino
def read_data():
    if ser.in_waiting > 0:
        data = ser.readline().decode('utf-8').strip()  # Read data and decode it
        print(f"Received GPS Data: {data}")

# Example to read data
try:
    while True:
        read_data()
        time.sleep(0.5)  # Wait a bit before reading again

except KeyboardInterrupt:
    print("Program interrupted")

finally:
    ser.close()  # Close the serial connection
