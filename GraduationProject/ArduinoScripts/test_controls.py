import serial
import time

def main():
    # Replace '/dev/ttyUSB0' with the correct port for your Arduino
    # For Windows, use something like 'COM3' or 'COM4'
    arduino_port = '/dev/ttyS0'
    baud_rate = 9600

    try:
        # Establish a serial connection to the Arduino
        print(f"Connecting to Arduino on {arduino_port}...")
        ser = serial.Serial(arduino_port, baud_rate)
        time.sleep(2)  # Wait for Arduino to reset
        print("Connected to Arduino!")

        print("Enter commands in the format: dir,speed")
        print("Examples: forward,100 | backward,50 | right,70 | stop,0")
        print("Type 'exit' to quit the program.")

        while True:
            # Get user input from terminal
            command = input("Enter command: ").strip()

            if command.lower() == "exit":
                print("Exiting program.")
                break

            # Validate command format
            if "," not in command or len(command.split(",")) != 2:
                print("Invalid command format. Use dir,speed (e.g., forward,100).")
                continue

            # Send command to Arduino
            ser.write((command + "\n").encode())
            print(f"Sent: {command}")

            # Optionally, read response from Arduino
            if ser.in_waiting > 0:
                response = ser.readline().decode().strip()
                print(f"Arduino: {response}")

    except serial.SerialException as e:
        print(f"Error: Could not open serial port {arduino_port}.")
        print(e)
    except Exception as e:
        print("An unexpected error occurred:")
        print(e)
    finally:
        # Close the serial connection
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial connection closed.")

if __name__ == "__main__":
    main()
