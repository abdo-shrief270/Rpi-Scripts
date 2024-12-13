import serial # type: ignore
import time

def setup_serial(port, baudrate=9600, timeout=1):
    """
    Set up the serial communication with the SIM808 module.
    """
    try:
        ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
        if ser.is_open:
            print("Serial port opened successfully.")
        else:
            ser.open()
        return ser
    except serial.SerialException as e:
        print(f"Failed to open serial port: {e}")
        raise

def send_at_command(ser, command, delay=1):
    """
    Send an AT command to the SIM808 module and return the response.
    """
    ser.write((command + "\r\n").encode())
    time.sleep(delay)
    #response = ser.readlines()
    return 'ok'

def make_call(ser, phone_number):
    """
    Make a call to a specified phone number.
    """
    print("Dialing number:", phone_number)
    time.sleep(4)
    # Send the command to dial the specified number
    response = send_at_command(ser, f"ATD{phone_number};")
    print("Dial command response:", response)

    # Check for any error messages in the response
    for line in response:
        if "ERROR" in line:
            print("Failed to make call.")
            return "Failed to make call."

    # Check if the call was successfully placed
    return "Call in progress..."

def end_call(ser):
    """
    End the ongoing call.
    """
    response = send_at_command(ser, "ATH")  # 'ATH' command hangs up the call
    print("End call response:", response)
    return "Call ended."

# Main script
if __name__ == "__main__":
    try:
        # Set up serial communication
        ser = setup_serial('/dev/ttyS0', baudrate=115200)

        # Make the call
        result = make_call(ser, '+201270989676')  # Replace with actual phone number
        print(result)

        # End the call after a delay (optional)
        time.sleep(10)  # Wait 5 seconds before ending the call
        end_result = end_call(ser)
        print(end_result)

    except Exception as e:
        print("Error:", e)
    finally:
        # Close the serial connection
        if ser.is_open:
            ser.close()
            print("Serial port closed.")
