import serial
import time

class SIM808:
    def __init__(self, port='/dev/serial0', baud_rate=9600, timeout=1):
        """
        Initializes the SIM808 module.
        
        :param port: Serial port connected to SIM808 (default '/dev/serial0')
        :param baud_rate: Baud rate for communication (default 9600)
        :param timeout: Timeout for serial read (default 1 second)
        """
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.serial = serial.Serial(port, baud_rate, timeout=timeout)
        time.sleep(2)  # Allow time for initialization

    def send_at_command(self, command, wait_for_response=True):
        """
        Sends an AT command to the SIM808 module.

        :param command: AT command string to send
        :param wait_for_response: If True, waits and returns the response
        :return: Response from SIM808 if wait_for_response is True, else None
        """
        self.serial.write((command + '\r\n').encode())
        time.sleep(0.5)  # Small delay to allow processing
        
        if wait_for_response:
            response = self.serial.readlines()
            return [line.decode().strip() for line in response]
        return None

    def get_signal_strength(self):
        """
        Retrieves the signal strength (CSQ) from SIM808.

        :return: Signal strength as a tuple (RSSI, BER)
        """
        response = self.send_at_command("AT+CSQ")
        if response and len(response) > 1:
            parts = response[1].split(": ")[1].split(",")
            return int(parts[0]), int(parts[1])
        return None

    def check_network_registration(self):
        """
        Checks if the SIM808 module is registered on the network.

        :return: Registration status as integer (1 = registered, 0 = not registered)
        """
        response = self.send_at_command("AT+CREG?")
        if response and len(response) > 1:
            parts = response[1].split(": ")[1].split(",")
            return int(parts[1])
        return None

    def get_location(self):
        """
        Retrieves GPS location data from the SIM808 module.

        :return: Location data as dictionary with latitude, longitude, and timestamp if available
        """
        self.send_at_command("AT+CGPSPWR=1")  # Enable GPS power
        time.sleep(2)  # Wait for GPS to warm up

        response = self.send_at_command("AT+CGPSINF=0")
        if response and len(response) > 1:
            parts = response[1].split(": ")[1].split(",")
            return {
                "latitude": parts[2],
                "longitude": parts[3],
                "timestamp": parts[4]  # or more as per need
            }
        return None

    def send_sms(self, phone_number, message):
        """
        Sends an SMS message via the SIM808 module.

        :param phone_number: Recipient phone number
        :param message: Message text to send
        :return: Confirmation response
        """
        self.send_at_command("AT+CMGF=1")  # Set SMS to text mode
        self.send_at_command(f'AT+CMGS="{phone_number}"')
        self.serial.write((message + "\x1A").encode())  # End message with Ctrl+Z
        time.sleep(3)  # Wait for message to be sent
        return self.serial.readlines()

    def read_sms(self, index):
        """
        Reads an SMS message from a specific index in the SIM808 storage.

        :param index: Index of the SMS to read
        :return: Message content and metadata (sender number, date, time) if available
        """
        self.send_at_command("AT+CMGF=1")  # Set SMS to text mode
        response = self.send_at_command(f"AT+CMGR={index}")
        if response and len(response) > 2:
            # Extract metadata and message content
            metadata = response[1].split(",")
            sender = metadata[1].replace('"', '')
            date = metadata[3].replace('"', '')
            message = response[2]
            return {
                "sender": sender,
                "date": date,
                "message": message
            }
        return None

    def delete_sms(self, index):
        """
        Deletes an SMS message from a specific index.

        :param index: Index of the SMS to delete
        :return: Confirmation response
        """
        return self.send_at_command(f"AT+CMGD={index}")

    def make_call(self, phone_number):
        """
        Initiates a call to a specified phone number.

        :param phone_number: Phone number to call
        :return: Confirmation response
        """
        return self.send_at_command(f"ATD{phone_number};")

    def hang_up_call(self):
        """
        Hangs up an ongoing call.

        :return: Confirmation response
        """
        return self.send_at_command("ATH")

    def send_http_request(self, url):
        """
        Sends an HTTP GET request via GPRS on the SIM808.

        :param url: URL to request
        :return: Response content from the server
        """
        self.send_at_command("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"")  # Set connection type to GPRS
        self.send_at_command("AT+SAPBR=3,1,\"APN\",\"your_apn\"")  # Set APN
        self.send_at_command("AT+SAPBR=1,1")  # Open the bearer
        self.send_at_command("AT+HTTPINIT")   # Initialize HTTP service

        self.send_at_command(f"AT+HTTPPARA=\"URL\",\"{url}\"")
        self.send_at_command("AT+HTTPACTION=0")  # 0 = GET request
        time.sleep(3)  # Wait for response

        response = self.send_at_command("AT+HTTPREAD")  # Read the response
        self.send_at_command("AT+HTTPTERM")  # Terminate HTTP session
        self.send_at_command("AT+SAPBR=0,1")  # Close the bearer
        return response

    def close(self):
        """
        Closes the serial connection to SIM808.
        """
        if self.serial.is_open:
            self.serial.close()
