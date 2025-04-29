#This file is the automated run file which runs automatically on the computer.
# It contains all of the states on the course and uses while loops for each state

import cv2
import time
import serial
import numpy as np
from picamera2 import Picamera2
from gpiozero import DistanceSensor

sensor = DistanceSensor(echo=22, trigger=27, max_distance =4)

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

# Define Blue Color Range (HSV)
lower_blue = np.array([100, 150, 50])
upper_blue = np.array([110, 255, 255])
lower_yellow = np.array([20, 100, 100]) 
upper_yellow = np.array([30, 255, 255])
lower_red1 = np.array([0, 120, 70])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([170, 120, 70]) 
upper_red2 = np.array([180, 255, 255])

# Define tracking zones
left_zone = [CENTER_X - 30, CENTER_X + 30]
right_zone = [CENTER_X -30, CENTER_X + 30]
ramp_zone = [CENTER_X -10, CENTER_X + 10]
correction_angle = 10  # Angle adjustment

# Video Writer (Save Video)
fourcc = cv2.VideoWriter_fourcc(*'XVID')  # Codec for .avi file
video_writer = cv2.VideoWriter('output.avi', fourcc, 20.0, (FRAME_WIDTH, FRAME_HEIGHT))

print("Pi 5: Started Blue Object Detection (State B1)")
B1=True #stops the code when state 1 is done

try:
    #BLUE BUCKET 1
    while B1==True: 
        print("Detecting Blue Bucket 1")
        # Capture frame from PiCamera2
        frame = picam2.capture_array()
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # Convert frame to HSV
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # Apply preprocessing
        mask = cv2.GaussianBlur(mask, (5, 5), 0)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # Detect contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            
            if M["m00"] != 0:
                centroid_x = int(M["m10"] / M["m00"])
                centroid_y = int(M["m01"] / M["m00"])
                
                # Vizual aid for the bucket
                cv2.drawContours(frame_bgr, [largest_contour], -1, (0, 255, 0), 2)  # Green contours
                cv2.circle(frame_bgr, (centroid_x, centroid_y), 5, (0, 0, 255), -1)  # Red dot at centroid
                cv2.line(frame_bgr, (centroid_x, 0), (centroid_x, FRAME_HEIGHT), (255, 0, 0), 2)  # Blue line for x-axis tracking
                # Display centroid value
                cv2.putText(frame_bgr, f"Centroid X: {centroid_x}", (centroid_x + 10, centroid_y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                print("Centroid is on: ", centroid_x)
                # Adjust movement
                if centroid_x > right_zone[1]:
                    ser_pico.reset_input_buffer()
                    ser_pico.write(b'AR\n')
                    ser_pico.flush()
                    print("Pi 5: Adjusting right")
                elif centroid_x < right_zone[0]:
                    ser_pico.reset_input_buffer()
                    ser_pico.write(b'AL\n')
                    ser_pico.flush()
                    print("Pi 5: Adjusting left")
                else:
                    ser_pico.reset_input_buffer()
                    ser_pico.write(b'STR1\n')
                    ser_pico.flush()
                    print("Pi 5: Moving straight")
                
                distance=sensor.distance * 100
                print("Distance to the bucket: ", distance)
                if distance<160:
                    print("Pi 5: Turning Right 315 degrees")
                    ser_pico.reset_input_buffer()
                    ser_pico.write(b'STOP-25-25 065\n') #stop
                    ser_pico.write(b'T13130\n') #turn left
                    ser_pico.write(b'T62250\n') #turn right
                    ser_pico.write(b'STOP-25-25 065\n')
                    time.sleep(4)
                    ser_pico.write(b'STR2\n')
                    time.sleep(4)
                    B1=False
                    continue
        else:
            ser_pico.reset_input_buffer()
            print("Don't see the object, stopping")
            ser_pico.write(b'STOP-05-05 001\n')
        # Write the frame to video file
        #video_writer.write(frame_bgr)
        
    #YELLOW BUCKET
    Y1=True
    while Y1==True:
        print("Detecting Yellow bucket")
        # Capture frame from PiCamera2
        frame = picam2.capture_array()
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # Convert frame to HSV
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Apply preprocessing
        mask = cv2.GaussianBlur(mask, (5, 5), 0)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # Detect contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            
            if M["m00"] != 0:
                centroid_x = int(M["m10"] / M["m00"])
                centroid_y = int(M["m01"] / M["m00"])
                
                # Vizual aid for the bucket
                cv2.drawContours(frame_bgr, [largest_contour], -1, (0, 255, 0), 2)  # Green contours
                cv2.circle(frame_bgr, (centroid_x, centroid_y), 5, (0, 0, 255), -1)  # Red dot at centroid
                cv2.line(frame_bgr, (centroid_x, 0), (centroid_x, FRAME_HEIGHT), (255, 0, 0), 2)  # Blue line for x-axis tracking
                # Display centroid value
                cv2.putText(frame_bgr, f"Centroid X: {centroid_x}", (centroid_x + 10, centroid_y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                print("Centroid is on: ", centroid_x)
                # Adjust movement
                if centroid_x > left_zone[1]:
                    ser_pico.reset_input_buffer()
                    ser_pico.write(b'AR\n')
                    ser_pico.flush()
                    print("Pi 5: Adjusting right")
                elif centroid_x < left_zone[0]:
                    ser_pico.reset_input_buffer()
                    ser_pico.write(b'AL\n')
                    ser_pico.flush()
                    print("Pi 5: Adjusting left")
                else:
                    ser_pico.reset_input_buffer()
                    ser_pico.write(b'STR1\n') #decreased the speed
                    ser_pico.flush()
                    print("Pi 5: Moving straight")
                
                distance=sensor.distance * 100
                print("Distance to the bucket: ", distance)
                if distance<140:
                    print("Pi 5: Turning Left 60 degrees")
                    ser_pico.write(b'STOP-10-75 190\n')
                    ser_pico.write(b'T60150\n') #turn right
                    #ser_pico.write(b'STOP-25-25 085\n')
                    ser_pico.write(b'T09230\n')#turn left, was 245
                    time.sleep(5)
                    Y1=False
        
        # Write the frame to video file
        #video_writer.write(frame_bgr)
    
    #BLUE BUCKET 2 BEFORE RAMP
    B2=True
    while B2==True: 
        print("Detecting Blue Bucket 2")
        # Capture frame from PiCamera2
        frame = picam2.capture_array()
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # Convert frame to HSV
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # Apply preprocessing
        mask = cv2.GaussianBlur(mask, (5, 5), 0)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # Detect contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            
            if M["m00"] != 0:
                centroid_x = int(M["m10"] / M["m00"])
                centroid_y = int(M["m01"] / M["m00"])
                
                # Vizual aid for the bucket
                cv2.drawContours(frame_bgr, [largest_contour], -1, (0, 255, 0), 2)  # Green contours
                cv2.circle(frame_bgr, (centroid_x, centroid_y), 5, (0, 0, 255), -1)  # Red dot at centroid
                cv2.line(frame_bgr, (centroid_x, 0), (centroid_x, FRAME_HEIGHT), (255, 0, 0), 2)  # Blue line for x-axis tracking
                # Display centroid value
                cv2.putText(frame_bgr, f"Centroid X: {centroid_x}", (centroid_x + 10, centroid_y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                print("Centroid is on: ", centroid_x)
                # Adjust movement
                if centroid_x > right_zone[1]:
                    ser_pico.reset_input_buffer()
                    ser_pico.write(b'AR\n')
                    ser_pico.flush()
                    print("Pi 5: Adjusting right")
                elif centroid_x < right_zone[0]:
                    ser_pico.reset_input_buffer()
                    ser_pico.write(b'AL\n')
                    ser_pico.flush()
                    print("Pi 5: Adjusting left")
                else:
                    ser_pico.reset_input_buffer()
                    ser_pico.write(b'STR1\n')
                    ser_pico.flush()
                    print("Pi 5: Moving straight")
                
                distance=sensor.distance * 100
                print("Distance to the bucket: ", distance)
                if distance<160:
                    print("Pi 5: Turning Right 315 degrees")
                    ser_pico.reset_input_buffer()
                    ser_pico.write(b'STOP-25-25 065\n') #stop
                    ser_pico.write(b'T13110\n') #turn left
                    ser_pico.write(b'T62250\n') #turn right
                    ser_pico.write(b'STOP-25-25 065\n')
                    time.sleep(4)
                    ser_pico.write(b'STR2\n')
                    time.sleep(2)
                    B2=False
                    continue
        else:
            ser_pico.reset_input_buffer()
            print("Don't see the object, stopping")
            ser_pico.write(b'STOP-05-05 001\n')
        # Write the frame to video file
        #video_writer.write(frame_bgr)
    
    
    #GOING OVER THE RAMP
    Ramp=True
    while Ramp==True: 
        # Capture frame from PiCamera2
        frame = picam2.capture_array()
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # Convert frame to HSV
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_blue, upper_blue) #detecting for next bluie bucket instead

        # Apply preprocessing
        mask = cv2.GaussianBlur(mask, (5, 5), 0)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # Detect contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            
            if M["m00"] != 0:
                centroid_x = int(M["m10"] / M["m00"])
                centroid_y = int(M["m01"] / M["m00"])
                
                # Vizual aid for the bucket
                cv2.drawContours(frame_bgr, [largest_contour], -1, (0, 255, 0), 2)  # Green contours
                cv2.circle(frame_bgr, (centroid_x, centroid_y), 5, (0, 0, 255), -1)  # Red dot at centroid
                cv2.line(frame_bgr, (centroid_x, 0), (centroid_x, FRAME_HEIGHT), (255, 0, 0), 2)  # Blue line for x-axis tracking
                # Display centroid value
                cv2.putText(frame_bgr, f"Centroid X: {centroid_x}", (centroid_x + 10, centroid_y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                print("Centroid is on: ", centroid_x)
                # Adjust movement
                if centroid_x > ramp_zone[1]:
                    ser_pico.reset_input_buffer()
                    ser_pico.write(b'AR\n')
                    ser_pico.flush()
                    print("Pi 5: Adjusting right")
                elif centroid_x < ramp_zone[0]:
                    ser_pico.reset_input_buffer()
                    ser_pico.write(b'AL\n')
                    ser_pico.flush()
                    print("Pi 5: Adjusting left")
                else:
                    ser_pico.reset_input_buffer()
                    ser_pico.write(b'STR5\n')
                    time.sleep(4)
                    Ramp=False
                    ser_pico.flush()
                    print("Pi 5: Moving straight")
        
        # Write the frame to video file
        #video_writer.write(frame_bgr)
        
    #BLUE BUCKET 3 AFTER RAMP
    B3=True
    while B3==True: 
        print("Detecting Blue Bucket 3")
        # Capture frame from PiCamera2
        frame = picam2.capture_array()
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # Convert frame to HSV
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # Apply preprocessing
        mask = cv2.GaussianBlur(mask, (5, 5), 0)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # Detect contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            
            if M["m00"] != 0:
                centroid_x = int(M["m10"] / M["m00"])
                centroid_y = int(M["m01"] / M["m00"])
                
                # Vizual aid for the bucket
                cv2.drawContours(frame_bgr, [largest_contour], -1, (0, 255, 0), 2)  # Green contours
                cv2.circle(frame_bgr, (centroid_x, centroid_y), 5, (0, 0, 255), -1)  # Red dot at centroid
                cv2.line(frame_bgr, (centroid_x, 0), (centroid_x, FRAME_HEIGHT), (255, 0, 0), 2)  # Blue line for x-axis tracking
                # Display centroid value
                cv2.putText(frame_bgr, f"Centroid X: {centroid_x}", (centroid_x + 10, centroid_y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                print("Centroid is on: ", centroid_x)
                # Adjust movement
                if centroid_x > right_zone[1]:
                    ser_pico.reset_input_buffer()
                    ser_pico.write(b'AR\n')
                    ser_pico.flush()
                    print("Pi 5: Adjusting right")
                elif centroid_x < right_zone[0]:
                    ser_pico.reset_input_buffer()
                    ser_pico.write(b'AL\n')
                    ser_pico.flush()
                    print("Pi 5: Adjusting left")
                else:
                    ser_pico.reset_input_buffer()
                    ser_pico.write(b'STR1\n')
                    ser_pico.flush()
                    print("Pi 5: Moving straight")
                
                distance=sensor.distance * 100
                print("Distance to the bucket: ", distance)
                
                if distance<160:
                    print("Pi 5: Turning Right 315 degrees")
                    ser_pico.reset_input_buffer()
                    ser_pico.write(b'STOP-25-25 065\n') #stop
                    ser_pico.write(b'T13140\n') #turn left
                    ser_pico.write(b'T62250\n') #turn right
                    ser_pico.write(b'STOP-25-25 065\n')
                    time.sleep(4)
                    B3=False
                    continue
        else:
            ser_pico.reset_input_buffer()
            print("Don't see the object, stopping")
            ser_pico.write(b'STOP-05-05 001\n')
        # Write the frame to video file
        #video_writer.write(frame_bgr)
        
        
    # ARCH - TWO RED OBJECTS AND GOING IN BETWEEN        
    Arch = True
    AREA_THRESHOLD = 5500  # Minimum total area of red objects

    while Arch: 
        print("Detecting Red Buckets")
        
        # Capture frame from PiCamera2
        frame = picam2.capture_array()
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # Convert frame to HSV
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 + mask2

        # Apply preprocessing
        mask = cv2.GaussianBlur(mask, (5, 5), 0)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # Detect contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) >= 2:  # Ensure at least two red objects are detected
            # Get the two largest contours
            contours = sorted(contours, key=cv2.contourArea, reverse=True)[:2]

            total_area = sum(cv2.contourArea(contour) for contour in contours)
            print("Total Red Area:", total_area)

            if total_area < AREA_THRESHOLD:
                print("Red objects too small - stopping")
                ser_pico.reset_input_buffer()
                ser_pico.flush()
                Arch = False  # Exit the loop
                continue  # Skip the rest of the loop

            centroids = []
            for contour in contours:
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    centroids.append((cx, cy))
                    
                    # Draw visual indicators
                    cv2.drawContours(frame_bgr, [contour], -1, (0, 255, 0), 2)  # Green contour
                    cv2.circle(frame_bgr, (cx, cy), 5, (0, 0, 255), -1)  # Red dot at centroid
                    cv2.putText(frame_bgr, f"({cx}, {cy})", (cx + 10, cy),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            # Calculate midpoint between two centroids
            midpoint_x = (centroids[0][0] + centroids[1][0]) // 2
            midpoint_y = (centroids[0][1] + centroids[1][1]) // 2
            cv2.circle(frame_bgr, (midpoint_x, midpoint_y), 5, (255, 255, 0), -1)  # Blue midpoint indicator
            
            print("Midpoint X:", midpoint_x)

            # Adjust movement based on midpoint position
            if midpoint_x > right_zone[1]:  # Midpoint is to the right
                ser_pico.reset_input_buffer()
                ser_pico.write(b'AR\n')
                ser_pico.flush()
                print("Pi 5: Adjusting right")
            elif midpoint_x < right_zone[0]:  # Midpoint is to the left
                ser_pico.reset_input_buffer()
                ser_pico.write(b'AL\n')
                ser_pico.flush()
                print("Pi 5: Adjusting left")
            else:
                ser_pico.reset_input_buffer()
                ser_pico.write(b'STR2\n')
                ser_pico.flush()
                print("Pi 5: Moving straight")

        else:
            print("No red detected - stopping")
            ser_pico.reset_input_buffer()
            ser_pico.write(b'STOP-05-05 001\n')
            ser_pico.flush()
            Arch = False  # Exit the loop

    # Write the frame to video file
    #video_writer.write(frame_bgr)

    
    #BLUE BUCKET 4 - FINAL BUCKET
    B4=True
    while B4==True: 
        print("Detecting Blue Bucket 3")
        # Capture frame from PiCamera2
        frame = picam2.capture_array()
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # Convert frame to HSV
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # Apply preprocessing
        mask = cv2.GaussianBlur(mask, (5, 5), 0)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # Detect contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            
            if M["m00"] != 0:
                centroid_x = int(M["m10"] / M["m00"])
                centroid_y = int(M["m01"] / M["m00"])
                
                # Vizual aid for the bucket
                cv2.drawContours(frame_bgr, [largest_contour], -1, (0, 255, 0), 2)  # Green contours
                cv2.circle(frame_bgr, (centroid_x, centroid_y), 5, (0, 0, 255), -1)  # Red dot at centroid
                cv2.line(frame_bgr, (centroid_x, 0), (centroid_x, FRAME_HEIGHT), (255, 0, 0), 2)  # Blue line for x-axis tracking
                # Display centroid value
                cv2.putText(frame_bgr, f"Centroid X: {centroid_x}", (centroid_x + 10, centroid_y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                print("Centroid is on: ", centroid_x)
                # Adjust movement
                if centroid_x > right_zone[1]:
                    ser_pico.reset_input_buffer()
                    ser_pico.write(b'AR\n')
                    ser_pico.flush()
                    print("Pi 5: Adjusting right")
                elif centroid_x < right_zone[0]:
                    ser_pico.reset_input_buffer()
                    ser_pico.write(b'AL\n')
                    ser_pico.flush()
                    print("Pi 5: Adjusting left")
                else:
                    ser_pico.reset_input_buffer()
                    ser_pico.write(b'STR1\n')
                    ser_pico.flush()
                    print("Pi 5: Moving straight")
                
                
                distance=sensor.distance * 100
                print("Distance to the bucket: ", distance)
                if distance<160:
                    print("Pi 5: Turning Right 315 degrees")
                    ser_pico.reset_input_buffer()
                    ser_pico.write(b'STOP-25-25 065\n') #stop
                    ser_pico.write(b'T13140\n') #turn left
                    ser_pico.write(b'T62250\n') #turn right
                    ser_pico.write(b'STOP-25-25 065\n')
                    time.sleep(4)
                    
                    ser_pico.write(b'STR2\n')
                    time.sleep(2)
                    B4=False
                    continue
        else:
            ser_pico.reset_input_buffer()
            print("Don't see the object, stopping")
            ser_pico.write(b'STOP-05-05 001\n')           
        # Write the frame to video file
        #video_writer.write(frame_bgr)
        
        
    #FINISH
    Finish=True
    while Finish==True: 
        print("Detecting Blue Bucket 3")
        # Capture frame from PiCamera2
        frame = picam2.capture_array()
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # Convert frame to HSV
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # Apply preprocessing
        mask = cv2.GaussianBlur(mask, (5, 5), 0)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # Detect contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            
            if M["m00"] != 0:
                centroid_x = int(M["m10"] / M["m00"])
                centroid_y = int(M["m01"] / M["m00"])
                
                # Vizual aid for the bucket
                cv2.drawContours(frame_bgr, [largest_contour], -1, (0, 255, 0), 2)  # Green contours
                cv2.circle(frame_bgr, (centroid_x, centroid_y), 5, (0, 0, 255), -1)  # Red dot at centroid
                cv2.line(frame_bgr, (centroid_x, 0), (centroid_x, FRAME_HEIGHT), (255, 0, 0), 2)  # Blue line for x-axis tracking
                # Display centroid value
                cv2.putText(frame_bgr, f"Centroid X: {centroid_x}", (centroid_x + 10, centroid_y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                print("Centroid is on: ", centroid_x)
                # Adjust movement
                if centroid_x > right_zone[1]:
                    ser_pico.reset_input_buffer()
                    ser_pico.write(b'AR\n')
                    ser_pico.flush()
                    print("Pi 5: Adjusting right")
                elif centroid_x < right_zone[0]:
                    ser_pico.reset_input_buffer()
                    ser_pico.write(b'AL\n')
                    ser_pico.flush()
                    print("Pi 5: Adjusting left")
                else:
                    ser_pico.reset_input_buffer()
                    ser_pico.write(b'STR2\n')
                    time.sleep(5)
                    ser_pico.flush()
                    Finish=False
                    print("Pi 5: Moving straight")
        
        # Write the frame to video file
        #video_writer.write(frame_bgr)'''
        
    print("Finite State Machine Finished")

finally:
    print("Pi 5: Stopping Detection")
    #ser_pico.write(b'STOP-25-25 065\n')
    ser_pico.write(b'FINISH\n')
    picam2.stop()
    video_writer.release()  # Release video writer
    cv2.destroyAllWindows()
    print("Pi 5: Video saved as 'output.avi'")
