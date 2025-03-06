#Takes the initial pictures, and then takes pictures of the mask after
# applying gaussian blur, erosion, and dilation
# Test how the mask changes after those filters are applied
import cv2
import time
from picamera2 import Picamera2

# Initialize the camera
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

# Define HSV range for blue
lower_blue = (100, 150, 50)
upper_blue = (140, 255, 255)

# Initialize frame counter and storage
frames_processed = 0
captured_frames = []
start_time = time.time()

# Capture the initial frame from the camera
initial_frame = picam2.capture_array()

# Convert initial frame to HSV
hsv = cv2.cvtColor(initial_frame, cv2.COLOR_RGB2HSV)
# Create mask to detect blue color
mask = cv2.inRange(hsv, lower_blue, upper_blue)

# Save the original camera view
cv2.imwrite("initial_camera_view.png", cv2.cvtColor(initial_frame, cv2.COLOR_RGB2BGR))

# Save the mask as the first processed image
cv2.imwrite("mask_image.png", mask)

# Apply Gaussian blur and save the result
blurred_mask = cv2.GaussianBlur(mask, (5, 5), 0)
cv2.imwrite("blurred_mask.png", blurred_mask)

# Apply erosion and save the result
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
eroded_mask = cv2.erode(blurred_mask, kernel, iterations=1)
cv2.imwrite("eroded_mask.png", eroded_mask)

# Apply dilation and save the result
dilated_mask = cv2.dilate(eroded_mask, kernel, iterations=1)
cv2.imwrite("dilated_mask.png", dilated_mask)

# Cleanup
picam2.stop()
cv2.destroyAllWindows()
