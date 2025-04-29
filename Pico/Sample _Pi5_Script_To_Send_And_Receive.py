#This code is to test communication between Pico and Pi 5 through USB
# This code simply sends the command, Pico mirrors it, and send 
# the same thing back

import serial
import time

PICO_PORT = "/dev/ttyACM0"  # Adjust this if needed
BAUD_RATE = 115200

try:
    with serial.Serial(PICO_PORT, BAUD_RATE, timeout=1) as ser:
        print("Connected to Pico")
        time.sleep(2)  # Wait for Pico to initialize

        message = "Hello, Pico\n"  # Ensure a newline is sent
        ser.write(message.encode())  # Send the message
        print(f"Sent: {message.strip()}")

        response = ser.readline().decode().strip()  # Read response
        print(f"Received from Pico: {response}")

except serial.SerialException as e:
    print(f"Error: {e}")
