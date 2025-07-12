# Importing all modules
import cv2 
import numpy as np

"""
Code Logic: 

1. Collect the HSV ranges of all colors
2. Opening cv2 and camera
3. Convert the video color from BGR (Standard) to HSV 
"""


# Lower range of Red
lower_red1 = np.array([0,120, 70])
upper_red1 = np.array([10,255,255])

# Upper range of Red
lower_red2 = np.array([170,120,70])
upper_red2 = np.array([180,255,255])

lower_green = np.array([35, 20, 70])
upper_green = np.array([75, 225, 225])

# Capturing webcam footage
webcam_video = cv2.VideoCapture(0)

ROI1 = [20, 170, 240, 220]
ROI2 = [400, 170, 620, 220] # 380, 600 | 400, 620
ROI3 = [200, 300, 440, 350]

def display_roi(img, ROIs, color):
    for ROI in ROIs: 
        img = cv2.line(img, (ROI[0], ROI[1]), (ROI[2], ROI[1]), color, 4)
        img = cv2.line(img, (ROI[0], ROI[1]), (ROI[0], ROI[3]), color, 4)
        img = cv2.line(img, (ROI[2], ROI[3]), (ROI[2], ROI[1]), color, 4)
        img = cv2.line(img, (ROI[2], ROI[3]), (ROI[0], ROI[3]), color, 4) 
    return img

def find_contours(hsv_blur, ROI):
    
    # Segment the image to only that box using indexing
    # Crop the frame: [y:y+h, x:x+w]
    hsv_blur_segmented = hsv_blur[ROI[1]:ROI[3], ROI[0]:ROI[2]] 

    # Masking the image to find our color - thresholding pixels that fall within color range
    mask_red1 = cv2.inRange(hsv_blur_segmented, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv_blur_segmented, lower_red2, upper_red2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)
    mask_green = cv2.inRange(hsv_blur_segmented, lower_green, upper_green)
    combined_mask = cv2.bitwise_or(mask_red, mask_green) # Combined all masks using bitwise_or

    # kernel 
    kernel = np.ones((5, 5), np.uint8)

    # erode and dilate
    eMask = cv2.erode(combined_mask, kernel, iterations=1)
    dMask = cv2.dilate(eMask, kernel, iterations=1)

    # Find Contours -> -2 determines the return signature (verison-agnostic shortcut to only return contours array)
    mask_contours = cv2.findContours(dMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

    # Bitwise-AND mask shows only the colored regions -> Only the colored region in a thresholded screen
    res_segment = cv2.bitwise_and(
        video[ROI[1]:ROI[3], ROI[0]:ROI[2]],
        video[ROI[1]:ROI[3], ROI[0]:ROI[2]],
        mask=dMask
    )

    full_res = video.copy()
    full_res[ROI[1]: ROI[3], ROI[0]:ROI[2]] = res_segment

    # Returns contour array points in only that region - this way you can draw a square around only those pics in the boxes
    return dMask, mask_contours, full_res  


while True:

    success, video = webcam_video.read() # Reading webcam footage
    
    # video frame and flag (type of conversion) --> BGR (Default) to HSV (more apt for color detection)
    hsv = cv2.cvtColor(video, cv2.COLOR_BGR2HSV) 

    # blur image
    hsv_blur = cv2.GaussianBlur(hsv, (7, 7), 0)

    left_side_mask, left_side_contours, left_side_res = find_contours(hsv_blur, ROI1)
    

    # # Finding position of all contours
    if len(left_side_contours) != 0:
        for mask_contour in left_side_contours:
            if cv2.contourArea(mask_contour) > 500:
                x, y, w, h = cv2.boundingRect(mask_contour)

                
                offset_x = ROI1[0]
                offset_y = ROI1[1]

                # Top left corner and bottom right corner, color of rectangle, and thickness
                cv2.rectangle(video, (x + offset_x, y + offset_y), (x + w + offset_x, y + h + offset_y), (0, 0, 255), 3)
    
    
    video = display_roi(video, [ROI1, ROI2, ROI3], (255, 204, 0))

    cv2.imshow('Masked', left_side_mask)
    cv2.imshow('Result',left_side_res)
    cv2.imshow("finalColor", video)

    k = cv2.waitKey(60)
    if k == ord('q'):
        break

cv2.destroyAllWindows()

"""
Code Block Logic: 

1. Display the 3 ROI boxes on video to capture the contour of the left and right wall alongside the floor for direction. (past comp)
    a. make boxes by drawing a line to shape a box for the (x1,y1) -> top left & (x2,y2) -> bottom right

2. Perform the Threshold and color detection exclusively for those boxes

3. Perform necessary morphological transformations - performed on binary images (thresholded masked v)

"""