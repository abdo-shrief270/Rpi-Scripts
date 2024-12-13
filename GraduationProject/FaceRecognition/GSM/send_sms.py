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
    response = ser.readlines()
    return [line.decode('utf-8').strip() for line in response]

def send_sms(ser, phone_number, message):
    """
    Send an SMS message to a specified phone number.
    """
    # Set SMS text mode
    response = send_at_command(ser, "AT+CMGF=1")  # Set SMS mode to text
    print("Setting SMS mode:", response)

    time.sleep(0.5)

    # Set the recipient phone number
    response = send_at_command(ser, f'AT+CMGS="{phone_number}"')
    print("Recipient set:", response)
    
    time.sleep(0.5)

    # Send the message
    ser.write(message.encode() + b'\x1A')  # End message with Ctrl+Z
    time.sleep(3)  # Wait for message to be sent
    response = ser.readlines()

    # Check response for SMS success
    for line in response:
        if "OK" in line.decode('utf-8'):
            return "SMS sent successfully!"
    return "Failed to send SMS."

# Main script
if __name__ == "__main__":
    try:
        # Set up serial communication
        ser = setup_serial('/dev/ttyS0', baudrate=115200)

        # Send SMS
        result = send_sms(ser, '+201270989676', 'Hello From Script')
        print(result)
    except Exception as e:
        print("Error:", e)
    finally:
        # Close the serial connection
        if ser.is_open:
            ser.close()
            print("Serial port closed.")

