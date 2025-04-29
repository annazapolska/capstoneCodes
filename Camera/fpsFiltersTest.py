#This code is another code to calculate fps rate when detecting blue
# objects and applying filters (Gaussian blur, erotion, dilation)

import cv2
import time
from picamera2 import Picamera2
import numpy as np

# Initialize the camera
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.controls.FrameRate = 100
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("video")
picam2.start()

# Define HSV range for blue
lower_blue = (100, 150, 50)
upper_blue = (140, 255, 255)

# Initialize frame counter and storage
frames_processed = 0
captured_frames = []  # Store frames for saving to video
start_time = time.time()

# Define the kernel for erosion and dilation
kernel = np.ones((5, 5), np.uint8)

try:
    while frames_processed < 100:
        # Capture frame
        frame = picam2.capture_array()
        
        # Convert frame to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Create mask to detect blue color
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # Apply Gaussian Blur
        blurred_mask = cv2.GaussianBlur(mask, (5, 5), 0)
        
        # Apply Erosion
        eroded_mask = cv2.erode(blurred_mask, kernel, iterations=1)

        # Apply Dilation
        dilated_mask = cv2.dilate(eroded_mask, kernel, iterations=1)

        # Find contours
        contours, _ = cv2.findContours(dilated_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw contours on the original frame
        for contour in contours:
            if cv2.contourArea(contour) > 1000:  # You can adjust this threshold as needed
                cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)  # Green contours

        # Store the frame (optional, if saving video)
        captured_frames.append(frame)

        frames_processed += 1
    
finally:
    # Calculate and print FPS
    end_time = time.time()
    elapsed_time = end_time - start_time
    fps = frames_processed / elapsed_time
    print(f"Processed {frames_processed} frames in {elapsed_time:.2f} seconds.")
    print(f"Approximate FPS: {fps:.2f}")
    
    # Save frames to video (only if frames were stored)
    if captured_frames:
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        out = cv2.VideoWriter('captured_640x480.avi', fourcc, 10.0, (3840, 2160))
        
        for frame in captured_frames:
            out.write(frame)  # Ensure correct color format
        
        out.release()
        print("Video saved as 'captured_640x480.avi'")
    
    # Cleanup
    picam2.stop()
    cv2.destroyAllWindows()
