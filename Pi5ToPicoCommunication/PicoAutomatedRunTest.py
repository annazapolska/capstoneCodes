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
send_command("STR3")  # Move forward at default speed
time.sleep(2)
send_command("STOP")    # Stop motors
time.sleep(5)
send_command("AR")    # Stop motors
time.sleep(4)
send_command("STOP")    # Stop motors
time.sleep(5)

# Continuous listening (optional)
while True:
    response = ser_pico.readline().decode().strip()
    if response:
        print(f"Pico Response: {response}")
