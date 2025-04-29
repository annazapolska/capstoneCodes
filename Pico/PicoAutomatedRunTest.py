#This code tests different commands sent to Pico when Automated run is 
# running on Pico. This code is for Pi 5.

import serial
import time

PORT_PICO = "/dev/ttyACM0"  # Adjust if necessary
BAUD_RATE = 115200

ser_pico = serial.Serial(PORT_PICO, BAUD_RATE, timeout=1)

def send_command(command):
    ser_pico.write(f"{command}\n".encode())
    ser_pico.flush()
    print(f"Sent command: {command}")
    time.sleep(0.1)
    
    # Read response from Pico
    response = ser_pico.readline().decode().strip()
    if response:
        print(f"Pico Response: {response}")
    
    #time.sleep(2)  # Delay to allow command execution

print("Testing commands on Pico")

# Test commands
send_command("STOP-10-25 225")
#send_command("T16150")
send_command("STOP000000 2")


# Continuous listening (optional)
while True:
    response = ser_pico.readline().decode().strip()
    if response:
        print(f"Pico Response: {response}")
