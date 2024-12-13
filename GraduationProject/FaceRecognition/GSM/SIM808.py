import serial # type: ignore
import time

def setup_serial(port, baudrate=9600, timeout=1):
    """
    Set up the serial communication with the SIM808 module.
    """
    ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
    if ser.is_open:
        print("Serial port opened successfully.")
    else:
        ser.open()
    return ser

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
    send_at_command(ser, "AT+CMGF=1")  # Set SMS mode to text
    time.sleep(0.5)
    
    # Set the recipient phone number
    send_at_command(ser, f'AT+CMGS="{phone_number}"')
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

