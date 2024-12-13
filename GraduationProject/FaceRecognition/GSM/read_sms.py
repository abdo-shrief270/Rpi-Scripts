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

def read_sms(ser):
    """
    Read received SMS messages from the SIM808 module.
    """
    # Set SMS text mode
    send_at_command(ser, "AT+CMGF=1")  # Set SMS mode to text
    time.sleep(0.5)

    # List all received SMS messages
    response = send_at_command(ser, "AT+CMGL=\"ALL\"")  # Retrieve all SMS messages
    messages = []
    current_message = {}

    for line in response:
        if line.startswith("+CMGL"):
            # If there is a current message, save it before starting a new one
            if current_message:
                messages.append(current_message)
            # Parse message metadata
            parts = line.split(',')
            message_index = parts[0].split(": ")[1]
            sender = parts[2].strip('"')
            timestamp = parts[4].strip('"')
            current_message = {
                "index": message_index,
                "sender": sender,
                "timestamp": timestamp,
                "text": ""
            }
        elif line == "OK":
            # If we reach OK, save the last message if it exists
            if current_message:
                messages.append(current_message)
            break
        elif line:
            # Append SMS content line to current message text
            if 'text' in current_message:  # Ensure the key exists
                current_message['text'] += line + "\n"  # Adding newline for readability

    if messages:
        for msg in messages:
            print(f"Message Index: {msg['index']}")
            print(f"From: {msg['sender']}")
            print(f"Timestamp: {msg['timestamp']}")
            print(f"Text: {msg['text'].strip()}")  # Strip any trailing newlines
            print("=" * 40)
    else:
        print("No new messages.")
    
    return messages

# Main script
if __name__ == "__main__":
    try:
        # Set up serial communication
        ser = setup_serial('/dev/ttyS0', baudrate=115200)
        
        # Read SMS messages
        print("Reading received SMS messages...")
        read_sms(ser)
        
    except Exception as e:
        print("Error:", e)
    finally:
        # Close the serial connection
        if ser.is_open:
            ser.close()
            print("Serial port closed.")

