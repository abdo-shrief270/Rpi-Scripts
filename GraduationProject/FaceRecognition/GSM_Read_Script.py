import serial
import time
import re

def setup_serial(port='/dev/ttyS0', baudrate=9600):
    """
    Initializes serial communication with the SIM808 module.

    Parameters:
    - port (str): The serial port to which the SIM808 module is connected.
    - baudrate (int): The communication speed (baud rate). Default is 9600.

    Returns:
    - ser (serial.Serial): Configured serial object for communication.
    """
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        ser.flush()
        print(f"Serial port {port} opened successfully.")
        return ser
    except Exception as e:
        print(f"Error opening serial port {port}: {e}")
        return None

def send_at_command(ser, command, delay=0.5):
    """
    Sends an AT command to the SIM808 module and reads the response.

    Parameters:
    - ser (serial.Serial): The serial object for communication.
    - command (str): The AT command to send.
    - delay (float): The time delay after sending the command (default is 0.5 seconds).

    Returns:
    - response (str): The response received from the SIM808.
    """
    ser.write((command + '\r').encode())
    time.sleep(delay)
    response =  ser.read_all().decode(errors='ignore')
    return response

def parse_sms(response):
    """
    Parses the SMS response to extract the sender's number and message content.

    Parameters:
    - response (str): The raw response from the SIM808 module.

    Returns:
    - messages (list of tuples): List of (number, message) tuples.
    """
    # Regex pattern to match the phone number and the message
    pattern = r'\+CMGL: \d+,"REC UNREAD","(.+?)",.*\n(.+?)(?=\+CMGL|\Z)'
    matches = re.findall(pattern, response, re.DOTALL)

    messages = [(match[0], match[1].strip()) for match in matches]
    return messages

def read_sms(ser):
    """
    Reads the list of SMS messages stored in the SIM808 module.

    Parameters:
    - ser (serial.Serial): The serial object for communication.

    Returns:
    - sms_list (list of tuples): List of (number, message) tuples.
    """
    # Set SMS mode to text mode
    send_at_command(ser, 'AT+CMGF=1')

    # List unread SMS messages
    response = send_at_command(ser, 'AT+CMGL="REC UNREAD"', delay=2)
    print(response)
    # Parse the response to extract phone numbers and messages
    sms_list = parse_sms(response)
    return sms_list
def make_call(ser, number):
    """
    Makes a voice call to the specified number.

    Parameters:
    - ser (serial.Serial): The serial object for communication.
    - number (str): The phone number to call.

    Returns:
    - response (str): The response from the SIM808 module.
    """
    command = f'ATD{number};'  # Dial command
    response = send_at_command(ser, command)
    return response

def hang_up(ser):
    """
    Hangs up the current call.

    Parameters:
    - ser (serial.Serial): The serial object for communication.

    Returns:
    - response (str): The response from the SIM808 module.
    """
    response = send_at_command(ser, 'ATH')  # Hang up command
    return response

def main():
    """
    Main function to initialize serial communication and read SMS messages.

    Returns:
    - None
    """
    # Setup serial communication
    ser = setup_serial('/dev/ttyS0', 9600)
    if ser is None:
        print("Failed to initialize serial communication.")
        return

    # Check if SIM808 module responds to AT commands
    if "OK" not in send_at_command(ser,'AT'):
        print("SIM808 module not responding.")
        print(send_at_command(ser,'AT'))
        ser.close()
        return
    # Read SMS messages
    sms_list = read_sms(ser)

    # Print each SMS's number and content
    if sms_list:
        print("Received SMS Messages:")
        for number, message in sms_list:
            print(f"From: {number}")
            print(f"Message: {message}")
            print("-" * 30)
    else:
        print("No unread SMS messages found.")

    res=make_call(ser,'+201270989676')
    sleep(10)
    res=hang_up(ser)
    # Close serial connection
    ser.close()

if __name__ == "__main__":
    main()
