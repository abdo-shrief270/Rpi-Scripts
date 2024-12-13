import serial # type: ignore
import time

ser = serial.Serial(port="/dev/ttyS0", baudrate=115200, timeout=1)

def send_at_command(command, delay=1):
    ser.write((command + "\r\n").encode())
    time.sleep(delay)
    response = ser.readlines()
    return [line.decode('utf-8').strip() for line in response]

def setup_gps():
    print("Turning on GPS...")
    send_at_command("AT+CGPSPWR=1")
    send_at_command("AT+CGPSRST=0")
    send_at_command("AT+CGPSOUT=32")  # Enable continuous NMEA output

def parse_gprmc(sentence):
    parts = sentence.split(',')
    if parts[2] == 'A':  # 'A' means data is valid
        raw_latitude = parts[3]
        lat_direction = parts[4]
        raw_longitude = parts[5]
        lon_direction = parts[6]
        
        # Convert latitude to decimal degrees
        lat_deg = float(raw_latitude[:2])
        lat_min = float(raw_latitude[2:])
        latitude = lat_deg + lat_min / 60.0
        if lat_direction == 'S':
            latitude = -latitude

        # Convert longitude to decimal degrees
        lon_deg = float(raw_longitude[:3])
        lon_min = float(raw_longitude[3:])
        longitude = lon_deg + lon_min / 60.0
        if lon_direction == 'W':
            longitude = -longitude

        return latitude, longitude
    return None

def main():
    setup_gps()
    print("Waiting for GPS fix...")
    
    last_latitude, last_longitude = None, None

    while True:
        line = ser.readline().decode('utf-8').strip()
        if line.startswith('$GPRMC'):
            gps_data = parse_gprmc(line)
            if gps_data:
                latitude, longitude = gps_data

                # Only report if coordinates have changed
                if (latitude != last_latitude) or (longitude != last_longitude):
                    last_latitude, last_longitude = latitude, longitude

                    # Generate Google Maps link
                    google_maps_link = f"https://www.google.com/maps/search/?api=1&query={latitude},{longitude}"
                    print(f"Latitude: {latitude:.6f}, Longitude: {longitude:.6f}")
                    print("Google Maps Link :")
                    print(f"{google_maps_link}\n")

            else:
                print("GPS data invalid or not fixed yet.")

        time.sleep(1)  # Adjust delay as needed

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Turning off GPS...")
        send_at_command("AT+CGPSPWR=0")
        ser.close()
        print("GPS turned off. Exiting...")

