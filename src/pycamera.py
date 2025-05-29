# Importing all modules
import cv2
import numpy as np


# Lower range of Red
lower_red1 = np.array([0,120, 70])
upper_red1 = np.array([10,255,255])

# Upper range of Red
lower_red2 = np.array([170,120,70])
upper_red2 = np.array([180,255,255])

lower_green = np.array([45, 20, 20])
upper_green = np.array([65, 225, 225])

# Capturing webcam footage
webcam_video = cv2.VideoCapture(0)


while True:

    success, video = webcam_video.read() # Reading webcam footage

    # video frame and flag (type of conversion) --> BGR (Default) to HSV (more apt for color detection)
    hsv = cv2.cvtColor(video, cv2.COLOR_BGR2HSV) 

    
    # Masking the image to find our color
    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    
    # Combined all masks using bitwise_or
    combined_mask = cv2.bitwise_or(mask_red, mask_green)

    # Bitwise-AND --> shows only the colored regions 
    res = cv2.bitwise_and(video,video, mask= combined_mask)
    
    # Finding contours in masked image
    mask_contours, hierarchy = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) 

    # Finding position of all contours
    if len(mask_contours) != 0:
        for mask_contour in mask_contours:
            if cv2.contourArea(mask_contour) > 500:
                x, y, w, h = cv2.boundingRect(mask_contour)
                # Top left corner and bottom right corner, color of rectangle, and thickness
                cv2.rectangle(video, (x, y), (x + w, y + h), (0, 0, 255), 3)


    cv2.imshow('Original',video)
    cv2.imshow('Masked',combined_mask)
    cv2.imshow('Result',res)

    k = cv2.waitKey(60)
    if k == ord('q'):
        break

cv2.destroyAllWindows()
