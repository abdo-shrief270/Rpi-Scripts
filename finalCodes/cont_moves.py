import serial
import time
import threading

# Initialize serial connection
try:
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)  # Update port if needed
    time.sleep(2)  # Allow time for the connection to stabilize
except serial.SerialException as e:
    print(f"Error: {e}")
    exit()

# Lock for thread-safe serial operations
ser_lock = threading.Lock()

# Event to signal thread termination
stop_event = threading.Event()

# Function to send a command to Arduino
def send_command(command):
    with ser_lock:  # Ensure thread-safe write
        command = command.strip()
        ser.write((command + "\n").encode())  # Send command followed by newline
        print(f"Sent: {command}")

# Function to read echoed data from Arduino
def read_echo():
    while not stop_event.is_set():
        if ser.in_waiting > 0:  # If data is available to read
            try:
                with ser_lock:  # Ensure thread-safe read
                    data = ser.readline().decode('utf-8', errors='ignore').strip()
                if data:
                    print(f"Arduino Echo: {data}")
            except UnicodeDecodeError as e:
                print(f"Error decoding data: {e}")

# Function to read user input
def read_user_input():
    while not stop_event.is_set():
        command = input("Enter command: ")
        if command.lower() == "exit":
            print("Exiting program...")
            stop_event.set()  # Signal threads to stop
            break
        send_command(command)  # Send user command to Arduino

# Main script
if __name__ == "__main__":
    try:
        print("Enter commands to send to Arduino (type 'exit' to quit):")
        
        # Start a thread for reading data from Arduino
        read_thread = threading.Thread(target=read_echo, daemon=True)
        read_thread.start()
        
        # Read user input in the main thread
        read_user_input()
    except KeyboardInterrupt:
        print("\nProgram interrupted.")
        stop_event.set()  # Signal threads to stop
    finally:
        if ser.is_open:
            ser.close()  # Close the serial connection
            print("Serial connection closed.")
