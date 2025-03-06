import cv2
import time
from picamera2 import Picamera2

# Initialize the camera
picam2 = Picamera2()
config = picam2.create_video_configuration(main={"size": (2560, 1440)})
picam2.configure(config)
picam2.set_controls({"FrameRate": 120})
picam2.start()


# Define HSV range for blue
lower_blue = (100, 150, 50)
upper_blue = (140, 255, 255)

# Initialize frame counter and storage
frames_processed = 0
captured_frames = []
start_time = time.time()

# Define video writer (MP4 format)
fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # More compatible codec
out = cv2.VideoWriter('captured.mp4', fourcc, 10.0, (2560, 1440))  # Match frame size

try:
    while frames_processed < 100:
        # Capture frame
        frame = picam2.capture_array()

        # Convert RGB to BGR for OpenCV display and processing
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # Convert frame to HSV for color detection
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

        # Create mask to detect blue color
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # Detect contours on the blue mask
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Draw rectangular bounding boxes around detected blue objects
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            if w * h > 500:  # Ignore small detections (adjust threshold as needed)
                cv2.rectangle(frame_bgr, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Green box

        # Store the frame for video saving
        captured_frames.append(frame_bgr)
        out.write(frame_bgr)  # Write frame to video

        '''
        # Display the frames
        cv2.imshow("Camera", frame_bgr)  # Show corrected BGR frame
        cv2.imshow("Blue Detection", mask)  # Show blue mask
        '''
        frames_processed += 1

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Calculate and print FPS
    end_time = time.time()
    elapsed_time = end_time - start_time
    fps = frames_processed / elapsed_time
    print(f"Processed {frames_processed} frames in {elapsed_time:.2f} seconds.")
    print(f"Approximate FPS: {fps:.2f}")

    # Release the video writer
    out.release()
    print("Video saved as 'captured.mp4'")

    # Cleanup
    picam2.stop()
    cv2.destroyAllWindows()
