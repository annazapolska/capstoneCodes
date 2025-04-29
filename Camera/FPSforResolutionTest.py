#This code is another test for fps rates for specific resolution

import cv2
import time
from picamera2 import Picamera2

picam2 = Picamera2()
config = picam2.create_video_configuration(main={"size": (4608, 2592)})
picam2.configure(config)
picam2.set_controls({"FrameRate": 56})
picam2.start()

'''
# Initialize the camera
picam2 = Picamera2()
picam2.preview_configuration.main.size = (4608, 2592)
picam2.preview_configuration.controls.FrameRate = 100
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("video")
picam2.start()
'''
# Define HSV range for blue
lower_blue = (100, 150, 50)
upper_blue = (140, 255, 255)

# Initialize frame counter and storage
frames_processed = 0
captured_frames = []  # Store frames for saving to video
start_time = time.time()

try:
    while frames_processed < 100:
        # Capture frame
        frame = picam2.capture_array()
        
        # Convert frame to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
         # Create mask to detect blue color
        mask = cv2.inRange(hsv, lower_blue, upper_blue)        

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
        out = cv2.VideoWriter('captured_640x480.avi', fourcc, 10.0, (2592, 1296))
        
        for frame in captured_frames:
            out.write(frame)  # Ensure correct color format
        
        out.release()
        print("Video saved as 'captured_640x480.avi'")
    
    # Cleanup
    picam2.stop()
    cv2.destroyAllWindows()
