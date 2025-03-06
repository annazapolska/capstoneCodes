import cv2
import time
import numpy as np
from picamera2 import Picamera2

# Initialize PiCamera2
picam2 = Picamera2()
config = picam2.create_video_configuration(main={"size": (640, 480)})
picam2.configure(config)
picam2.start()

# Define Blue Color Range (HSV)
lower_blue = np.array([100, 150, 50])
upper_blue = np.array([140, 255, 255])

# Define frame properties
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
CENTER_X = FRAME_WIDTH // 2

# Define tracking zones
left_zone = [CENTER_X - 310, CENTER_X - 295]
right_zone = [CENTER_X + 295, CENTER_X + 310]
correction_angle = 10  # Angle adjustment

# Video Writer (Save Video)
fourcc = cv2.VideoWriter_fourcc(*'XVID')  # Codec for .avi file
video_writer = cv2.VideoWriter('output.avi', fourcc, 20.0, (FRAME_WIDTH, FRAME_HEIGHT))

print("Pi 5: Started Blue Object Detection (State B1)")

try:
    while True:
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

                # Draw contours
                cv2.drawContours(frame_bgr, [largest_contour], -1, (0, 255, 0), 2)  # Green contours

                # Draw centroid
                cv2.circle(frame_bgr, (centroid_x, centroid_y), 5, (0, 0, 255), -1)  # Red dot at centroid
                cv2.line(frame_bgr, (centroid_x, 0), (centroid_x, FRAME_HEIGHT), (255, 0, 0), 2)  # Blue line for x-axis tracking

                # Display centroid value
                cv2.putText(frame_bgr, f"Centroid X: {centroid_x}", (centroid_x + 10, centroid_y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                # Adjust movement
                if centroid_x > right_zone[1]:
                    print("Pi 5: Turning right")
                elif centroid_x < right_zone[0]:
                    print(f"Centroid is on: {centroid_x}")
                    print("Pi 5: Turning left")
                else:
                    print("Pi 5: Moving straight")

        # Write the frame to video file
        video_writer.write(frame_bgr)

        # Display the frame
        cv2.imshow("Blue Object Tracking", frame_bgr)

        # Exit with 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    print("Pi 5: Stopping Detection")
    picam2.stop()
    video_writer.release()  # Release video writer
    cv2.destroyAllWindows()
