# https://wiki.seeedstudio.com/reTerminal_DM_opencv/
# This code shows the life view of detection. SImple code to check 
# that detection for colors work
import cv2
from picamera2 import Picamera2
picam2 = Picamera2()
picam2.preview_configuration.main.size = (1280,720)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()
while True:
    im= picam2.capture_array()
    hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
    
    lower_blue=(100, 150, 50)
    upper_blue=(140, 255, 255)
    
    mask=cv2.inRange(hsv, lower_blue, upper_blue)
    
    result=cv2.bitwise_and(im, im, mask=mask)
    cv2.imshow("Camera", im)
    cv2.imshow("BlueDetection", result)
    if cv2.waitKey(1)==ord('q'):
        break
cv2.destroyAllWindows()
picam2.stop()
