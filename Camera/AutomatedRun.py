import cv2
import time
import serial
import numpy as np
from picamera2 import Picamera2
from gpiozero import DistanceSensor

sensor = DistanceSensor(echo=22, trigger=27, max_distance=4)

# Initialize Serial Communication
ser_pico = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # Adjust for Pico
time.sleep(2)  # Allow time for serial connection

# Define frame properties
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
CENTER_X = FRAME_WIDTH // 2

# Initialize PiCamera2
picam2 = Picamera2()
config = picam2.create_video_configuration(main={"size": (FRAME_WIDTH, FRAME_HEIGHT)})
picam2.configure(config)
picam2.set_controls({"FrameRate": 10})
picam2.start()

# Define Color Ranges (HSV)
COLOR_THRESHOLDS = {
    "blue": (np.array([100, 150, 50]), np.array([140, 255, 255])),
    "yellow": (np.array([20, 100, 100]), np.array([30, 255, 255]))
}

# Define tracking zones
left_zone = [CENTER_X - 250, CENTER_X - 100]
right_zone = [CENTER_X - 70, CENTER_X + 70]

# Video Writer (Save Video)
fourcc = cv2.VideoWriter_fourcc(*'XVID')  # Codec for .avi file
video_writer = cv2.VideoWriter('output.avi', fourcc, 20.0, (FRAME_WIDTH, FRAME_HEIGHT))


def detect_object(color_name):
    """Detects the specified color object and returns centroid & distance."""
    lower_color, upper_color = COLOR_THRESHOLDS[color_name]

    # Capture frame
    frame = picam2.capture_array()
    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    # Convert frame to HSV
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_color, upper_color)

    # Apply preprocessing
    mask = cv2.GaussianBlur(mask, (5, 5), 0)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # Detect contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    centroid_x = None
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)

        if M["m00"] != 0:
            centroid_x = int(M["m10"] / M["m00"])

            # Visual aid for debugging
            cv2.drawContours(frame_bgr, [largest_contour], -1, (0, 255, 0), 2)  # Green contours
            cv2.circle(frame_bgr, (centroid_x, FRAME_HEIGHT // 2), 5, (0, 0, 255), -1)  # Red dot at centroid
            cv2.line(frame_bgr, (centroid_x, 0), (centroid_x, FRAME_HEIGHT), (255, 0, 0), 2)  # Blue tracking line
    
    video_writer.write(frame_bgr)
    distance = sensor.distance * 100
    
    # Display the frame
    #cv2.imshow("Blue Object Tracking", frame_bgr)
    
    return centroid_x, distance

def adjust_movement(centroid_x):
    """Adjust movement based on object position."""
    if centroid_x > right_zone[1]:
        ser_pico.reset_input_buffer()
        ser_pico.write(b'AR\n')
        print("Pi 5: Adjusting right")
    elif centroid_x < right_zone[0]:
        ser_pico.reset_input_buffer()
        ser_pico.write(b'AL\n')
        print("Pi 5: Adjusting left")
    else:
        ser_pico.reset_input_buffer()
        ser_pico.write(b'STR3\n')
        print("Pi 5: Moving straight")
    ser_pico.flush()

def state_1():
    """State 1: Detect blue bucket and turn right."""
    print("Pi 5: Detecting Blue Bucket (State 1)")
    centroid_x, distance = detect_object("blue")
    print(f"Distance to blue bucket: {distance:.2f} cm")

    if distance < 150:
        print("Pi 5: Turning Right 315 degrees")
        ser_pico.write(b'STR-25\n')  # Stop
        time.sleep(0.65)
        ser_pico.write(b'T13150\n')  # Turn Left
        ser_pico.write(b'STR-25\n')
        time.sleep(0.85)
        ser_pico.write(b'T61150\n')  # Turn Right
        ser_pico.write(b'STR2\n')  # Move forward
        time.sleep(2)
        return 2  # Move to next state

    adjust_movement(centroid_x)
    return 1  # Stay in current state

def state_2():
    """State 2: Detect yellow bucket and turn left."""
    print("Pi 5: Detecting Yellow Bucket (State 2)")
    centroid_x, distance = detect_object("yellow")
    print(f"Distance to yellow bucket: {distance:.2f} cm")

    if distance < 150:
        print("Pi 5: Turning Left 315 degrees")
        ser_pico.write(b'STR-25\n')  # Stop
        time.sleep(0.65)
        ser_pico.write(b'T31150\n')  # Slight Right Turn
        ser_pico.write(b'STR-25\n')
        time.sleep(0.85)
        ser_pico.write(b'T16150\n')  # Full Left Turn
        ser_pico.write(b'STR2\n')  # Move forward
        time.sleep(1)
        return None  # End state machine

    adjust_movement(centroid_x)
    return 2  # Stay in current state

# State machine dictionary
state_functions = {
    1: state_1,
    2: state_2
}

# State machine loop
current_state = 1
print("Pi 5: Starting State Machine")
try:
    while current_state is not None:
        current_state = state_functions[current_state]()  # Call the function for the current state
finally:
    print("Pi 5: Stopping Detection")
    ser_pico.write(b'STOP\n')
    picam2.stop()
    cv2.destroyAllWindows()
    print("Pi 5: State Machine Complete")
