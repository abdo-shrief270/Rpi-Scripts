from SIM808 import SIM808
from time import sleep
# Initialize SIM808
sim = SIM808(port='/dev/ttyS0', baud_rate=115200)

# send AT Command
response = sim.send_at_command("AT")
print("AT Response:",response)

# Send SMS
response = sim.send_sms("+201270989676", "Hello from SIM808!")
print("SMS Response:", response)

# Read an SMS at index 1
sms = sim.read_sms(1)
print("Received SMS:", sms)

# Make a call
sim.make_call("+201270989676")
sleep(10)  # Let the call ring for 10 seconds
sim.hang_up_call()

# Send HTTP request
http_response = sim.send_http_request("http://example.com")
print("HTTP Response:", http_response)

# Close the connection
sim.close()
