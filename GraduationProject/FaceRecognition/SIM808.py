import serial
import time
import re

def setup_serial(port='/dev/ttyS0', baudrate=9600):
    """
    Initializes serial communication with the SIM808 module.

    Parameters:
    - port (str): The serial port to which the SIM808 module is connected.
    - baudrate (int): The communication speed (baud rate). Default is 115200.

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

def send_at_command(ser, command, delay=1):
    """
    Sends an AT command to the SIM808 module and reads the response.

    Parameters:
    - ser (serial.Serial): The serial object for communication.
    - command (str): The AT command to send.
    - delay (float): The time delay after sending the command (default is 1 second).

    Returns:
    - response (str): The response received from the SIM808.
    """
    ser.write((command + '\r').encode())
    time.sleep(delay)
    response = ser.read_all().decode(errors='ignore').strip()
    return response

def parse_sms(response):
    """
    Parses the SMS response to extract the sender's number and message content.

    Parameters:
    - response (str): The raw response from the SIM808 module.

    Returns:
    - messages (list of tuples): List of (number, message) tuples.
    """
    # Regex pattern to match the sender's number and the message content
    pattern = r'\+CMGL: \d+,"(.*?)","(.*?)",.*?\n(.*?)(?=\n\n|\+CMGL:|\Z)'
    matches = re.findall(pattern, response, re.DOTALL)

    # Clean up the message content and extract relevant information
    messages = [(match[1].strip('"'), match[2].replace('\r', '').strip()) for match in matches]

    return messages

def read_sms(ser):
    """
    Reads the list of SMS messages stored in the SIM808 module.

    Parameters:
    - ser (serial.Serial): The serial object for communication.

    Returns:
    - sms_list (list of tuples): List of (number, message) tuples.
    """
    send_at_command(ser, 'AT+CMGF=1')  # Set SMS mode to text
    response = send_at_command(ser, 'AT+CMGL="REC UNREAD"', delay=2)
    sms_list = parse_sms(response)
    return sms_list

def send_sms(ser, number, message):
    """
    Sends an SMS message to a specified number.

    Parameters:
    - ser (serial.Serial): The serial object for communication.
    - number (str): The recipient's phone number.
    - message (str): The message content to send.

    Returns:
    - response (str): The response from the SIM808 module.
    """
    send_at_command(ser, 'AT+CMGF=1')  # Set SMS mode to text
    command = f'AT+CMGS="{number}"'
    send_at_command(ser, command)  # Send the command
    ser.write((message + chr(26)).encode())  # Send message and CTRL+Z
    time.sleep(1)  # Allow some time for processing
    response = ser.read_all().decode(errors='ignore').strip()
    print(f"Sent SMS to {number}: {message}, Response: {response}")
    return response

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

def check_network(ser):
    """
    Checks the network registration status.

    Parameters:
    - ser (serial.Serial): The serial object for communication.

    Returns:
    - response (str): The response from the SIM808 module regarding network status.
    """
    response = send_at_command(ser, 'AT+CREG?')  # Check network registration
    return response

def http_get(ser, url):
    """
    Performs an HTTP GET request to the specified URL.

    Parameters:
    - ser (serial.Serial): The serial object for communication.
    - url (str): The URL to send the GET request to.

    Returns:
    - response (str): The response from the HTTP GET request.
    """
    send_at_command(ser, 'AT+HTTPINIT')  # Initialize HTTP service
    send_at_command(ser, f'AT+HTTPPARA="CID",1')  # Set the context ID
    send_at_command(ser, f'AT+HTTPPARA="URL","{url}"')  # Set the URL
    response = send_at_command(ser, 'AT+HTTPGET')  # Send the GET request
    send_at_command(ser, 'AT+HTTPTERM')  # Terminate HTTP service
    return response

def http_post(ser, url, data):
    """
    Performs an HTTP POST request to the specified URL with the given data.

    Parameters:
    - ser (serial.Serial): The serial object for communication.
    - url (str): The URL to send the POST request to.
    - data (str): The data to send in the POST request.

    Returns:
    - response (str): The response from the HTTP POST request.
    """
    send_at_command(ser, 'AT+HTTPINIT')  # Initialize HTTP service
    send_at_command(ser, f'AT+HTTPPARA="CID",1')  # Set the context ID
    send_at_command(ser, f'AT+HTTPPARA="URL","{url}"')  # Set the URL
    send_at_command(ser, 'AT+HTTPPARA="CONTENT","application/x-www-form-urlencoded"')  # Set content type
    send_at_command(ser, f'AT+HTTPDATA={len(data)},10000')  # Prepare to send data
    time.sleep(1)  # Wait for the buffer to be ready
    ser.write(data.encode())  # Send the data
    response = send_at_command(ser, 'AT+HTTPACTION=1')  # Send the POST request
    send_at_command(ser, 'AT+HTTPTERM')  # Terminate HTTP service
    return response

def check_sim_status(ser):
    """
    Checks the status of the SIM card.

    Parameters:
    - ser (serial.Serial): The serial object for communication.

    Returns:
    - response (str): The response from the SIM808 module regarding SIM status.
    """
    response = send_at_command(ser, 'AT+CPIN?')  # Check SIM status
    return response

def get_battery_status(ser):
    """
    Gets the battery status of the SIM808 module.

    Parameters:
    - ser (serial.Serial): The serial object for communication.

    Returns:
    - response (str): The response from the SIM808 module regarding battery status.
    """
    response = send_at_command(ser, 'AT+CBC')  # Get battery status
    return response

# Example usage:
if __name__ == "__main__":
    # Setup the serial connection to the SIM808
    ser = setup_serial('/dev/ttyS0',baudrate=9600)  # Adjust as necessary for your setup
    if ser:
        print('true')
        # Test sending an SMS
        #send_sms(ser, '+201270989676', 'Hello from SIM808!')

        # Test making a call
        #make_call(ser,'+201270989676')
        #time.sleep(5)  # Wait for 10 seconds to simulate a call duration
        #hang_up(ser)

        # Check network status
        #print(check_network(ser))

        # Check SIM status
        #print(check_sim_status(ser))

        # Get battery status
        #print(get_battery_status(ser))

        # Test HTTP GET
        #print(http_get(ser, 'http://httpbin.org/get'))

        # Test HTTP POST
        #print(http_post(ser, 'http://httpbin.org/post', 'key=value'))

        # Read SMS
        #messages = read_sms(ser)
        #print("Received SMS Messages:", messages)
        # Close the serial connection
        res=send_at_command(ser,'AT')
        print(res)
        ser.close()
