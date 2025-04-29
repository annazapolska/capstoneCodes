#This code is the code ran on Pico for the automated run

import sys 
import time 
from machine import Pin, PWM 

# Define the PWM pins for motor control 
pwm_pinL = PWM(Pin(12))  # Left motor PWM 
pwm_pinR = PWM(Pin(10))  # Right motor PWM 

# Set PWM frequency 
pwm_pinL.freq(50) 
pwm_pinR.freq(50) 

# Define the LED indicator 
led = Pin(25, Pin.OUT)  

# Function to set motor speed 
def set_motor_speed(pwm, speed): 
    """ 
    Set motor speed. 
    :param pwm: PWM pin (pwm_pinL or pwm_pinR) 
    :param speed: Speed (-100 to 100), where: 
                 -100 = Full reverse 
                   0  = Neutral (stop) 
                 100  = Full forward 
    """ 
    try:
        speed = int(speed)
        min_duty = 3000  # 1ms pulse (full reverse) 
        neutral_duty = 5000  # 1.5ms pulse (neutral) 
        max_duty = 7000  # 2ms pulse (full forward) 

        # Map speed (-100 to 100) to duty cycle (3000 to 7000) 
        duty_cycle = int(((speed + 100) / 200) * (max_duty - min_duty) + min_duty) 
        pwm.duty_u16(duty_cycle)
    except ValueError:
        print(f"Invalid speed value: {speed}")

# Function to move straight  
def goStraight(speed): 
    print(f"Moving forward at speed {speed}") 
    set_motor_speed(pwm_pinL, speed) 
    set_motor_speed(pwm_pinR, speed) 

# Function to stop motors 
def stopMotors(speedL, speedR, stop_time): 
    ms_delay = int(stop_time * 10)
    print(f"Stopping motors: left at {speedL} and right at {speedR} for {ms_delay} ms") 
    set_motor_speed(pwm_pinL, speedL) 
    set_motor_speed(pwm_pinR, speedR)
    time.sleep_ms(ms_delay)

# Function to turn right for hard turns
def turn(speedL, speedR, degrees): 
    ms_delay = int(degrees * 10)  # Adjust based on robot turning characteristics 
    print(f"Turning with left {speedL} and right {speedR} for {ms_delay} ms") 
    set_motor_speed(pwm_pinL, speedL)  # Left wheel forward 
    set_motor_speed(pwm_pinR, speedR)  # Right wheel reverse 
    time.sleep_ms(ms_delay)

# Function to adjust right (without delay)
def adjustRight():
    """ Adjusts the motor speed to slightly turn right without a delay. """
    print(f"Adjusting right")
    set_motor_speed(pwm_pinL, 3) 
    set_motor_speed(pwm_pinR, 1) 

# Function to adjust left (without delay)
def adjustLeft():
    """ Adjusts the motor speed to slightly turn left without a delay. """
    print(f"Adjusting left")
    set_motor_speed(pwm_pinL, 1) 
    set_motor_speed(pwm_pinR, 3) 

print("Pico is ready to receive commands")

# Main loop to receive commands 
try: 
    while True: 
        command = sys.stdin.readline().strip() 
        if command.startswith("T"):  # Turn Right 
            speedL=int(command[1])
            speedR=int(command[2])
            degrees = int(command[3:]) 
            turn(speedL, speedR, degrees) 

        elif command.startswith("STR"):  # Go Straight 
            speed = int(command[3:]) 
            goStraight(speed)

        elif command.startswith("AR"):  # Adjust Right 
            adjustRight()

        elif command.startswith("AL"):  # Adjust Left 
            adjustLeft()

        elif command.startswith("STOP"):  # Stop motors
            speedL=int(command[4:7])
            speedR=int(command[7:10])
            stop_time = int(command[11:])
            stopMotors(speedL, speedR, stop_time)
            

        else: 
            print(f"Unknown command: {command}") 

except KeyboardInterrupt: 
    print("Program interrupted.") 

finally: 
    # Ensure motors stop on exit 
    stopMotors(0,0,2) 
    pwm_pinL.deinit() 
    pwm_pinR.deinit() 
    led.value(0) 


