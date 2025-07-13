import cv2
import time
import numpy as np
from gpiozero import AngularServo

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

servo = AngularServo(
    13,
    min_pulse_width=0.0005,  
    max_pulse_width=0.0024,  
    frame_width=0.02        
)

# VARIABLES 
MID_SERVO = 0
TURN_DEGREE = 40 # Angle from center to max left or right
DC_SPEED = 1000

# PD STEERING - subject to change via height
kp = 0.02
kd = 0.006 

# areas to find proportional and derivative 
areaDiff = 0
prevAreaDiff = 0 

# Max amount we can turn either direction -> Center is 0 - Left is -90  - Right is +90
furthestLeft = MID_SERVO - 40
furthestRight = MID_SERVO + 40

#Turning Logic Variables
threshold_to_start_turn = 150 # how much the adjacent wall dissapears so that the car knows to turn in that direction
threshold_to_end_turn = 1500  # how much the upcoming wall comes into view so the car knows to stop turning

#Left and Right Turn bool control
left_turn = False 
right_turn = False 

# Counter
line_detected = False 
counter =  0 

# FUNCTIONS
def display_roi(img, ROIs, color):
    for ROI in ROIs:
        img = cv2.line(img, (ROI[0], ROI[1]), (ROI[2], ROI[1]), color, 4)
        img = cv2.line(img, (ROI[0], ROI[1]), (ROI[0], ROI[3]), color, 4)
        img = cv2.line(img, (ROI[2], ROI[3]), (ROI[2], ROI[1]), color, 4)
        img = cv2.line(img, (ROI[2], ROI[3]), (ROI[0], ROI[3]), color, 4)
    
    return img 

def get_contours(ROI, hsv, ranges): 
    
    segmented_area = hsv[ROI[1]: ROI[3], ROI[0]:ROI[2]]
    mask = cv2.inRange(segmented_area, ranges[0], ranges[1])
    kernel = np.ones((5,5), np.uint8)
    erode = cv2.erode(mask, kernel, iterations=1)
    dilate = cv2.dilate(erode, kernel)
    list_contour = cv2.findContours(dilate, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    
    return list_contour

def detect_contour(contour, ROI):
    
    maxArea = 0
    maxY = 0
    maxX = 0
    maxCnt = 0
    
    for cnt in contour: 
        area = cv2.contourArea(cnt)
        if area > 100:
            approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
            x, y, w, h = cv2.boundingRect(approx)
            abs_x = x + ROI[0]
            abs_y = y + ROI[1]
            cv2.rectangle(frame, (abs_x, abs_y), (abs_x + w, abs_y + h ), (0,255,0), 2)  
            
            if area > maxArea:
                maxArea = area 

    return maxArea  
    
    
# Color Masks range defined for black contour (walls) 
rBlack = [np.array([0,0,0]), np.array([180, 255, 75])]

# Color Mask range defined for orangle line (closest to end of turn)
rOrange = [np.array([10, 100, 100]), np.array([25,255,255])]

# Regions of Interest to detect the walls and line
ROI1 = [20, 170, 240, 220] 
ROI2 = [400, 170, 620, 220] 
ROI3 = [200, 300, 440, 350] 

    
# Capture a frame
while True:
    
    ret, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) 
    
    # Add Lines
    frame = display_roi(frame,  [ROI1, ROI2, ROI3], (255, 204, 0))
    
    # Get Contours
    cListLeft = get_contours(ROI1, hsv, rBlack )
    cListRight = get_contours(ROI2, hsv, rBlack )
    cListLine = get_contours(ROI3, hsv, rOrange)

    # Find Area of Left Contours --> Make this into a function
    areaLeft = detect_contour(cListLeft, ROI1)
    areaRight = detect_contour(cListRight, ROI2)
    
    # Line
    areaLine = detect_contour(cListLine, ROI3)
    if areaLine > 100:
        line_detected = True

    # find the area difference (Cross track error)
    areaDiff = areaLeft - areaRight
    derivative_term = areaDiff - prevAreaDiff # Cross track error rate (to control)

    # Clamp the angle bc you dont want the servo to be turning too far
    angle = int(MID_SERVO + (kp * areaDiff) + (kd * derivative_term)) 


    # PROGRAM FLOW -------

    # If not in right or left turn, if the thresholds are met to initiate a turn, then turn, otherwise, pass
    if not right_turn and not left_turn:
        if areaRight < threshold_to_start_turn and areaLeft > threshold_to_start_turn:
            right_turn = True 
            print("RIGHT") 
        
        elif areaLeft < threshold_to_start_turn and areaRight > threshold_to_start_turn:
            left_turn = True 
            print("RIGHT")
        
        else:
            pass
        
    
    # If we are in left turn, either exit turn if threshold is met or continue turning with the calculated angle (clamp values if too far)
    elif left_turn:
        if areaLeft > threshold_to_end_turn:
            left_turn = False 
            prevAreaDiff = 0 # Reset for PD Steering
            if line_detected: counter += 1
            line_detected = False 
            print("Exiting Left Turn")
        
        # Otherwise force angle to continue turning
        angle = min(max(angle, furthestLeft), furthestRight)
    
    # If we are in right turn, either exit turn if threshold is met or continue turning with the calculated angle (clamp values if too far)
    elif right_turn:
        if areaRight > threshold_to_end_turn:
            right_turn = False 
            prevAreaDiff = 0 # Reset for PD Steering
            if line_detected: counter += 1
            line_detected = False 
            print("Exiting Right Turn")
        
        # Otherwise force angle to continue turning
        angle = min(max(angle, furthestLeft), furthestRight)
    
    # WRITE TO SERVO
    servo.angle = angle
    
    # update values for next iteration
    prevAreaDiff = areaDiff 
    
    # Stop Car Logic 
    if counter == 12 and (abs(angle) - MID_SERVO) <= 10:
        # STOP CAR
        sleep(2) # Depends how fast our car is
    

    cv2.imshow('USB Camera Feed', frame)

    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()

"""
Code Block Logic: 

1. Display the 3 ROI boxes on video to capture the contour of the left and right wall alongside the floor for direction. (past comp)
    a. make boxes by drawing a line to shape a box for the (x1,y1) -> top left & (x2,y2) -> bottom right

2. Perform the Threshold and color detection exclusively for those boxes

3. Perform necessary morphological transformations - performed on binary images (thresholded masked v)

    
        Logic: If the adjacent wall is less than start_turn_threshold, enter a turn (make the boolean True)
        
        If the angle value is changing: and we're in a turn 
            And -> Where coming to an end of a turn, change the booeleans, reset the prev Diff for PD, and if line detected increment by one
            Or -> we need to turn more (exit_threshold has not arrived yet), find the new angle to append **
        
        If the angle is changing: and we're not in a turn 
            steer to the same angle, unless its greater to the sharp left or right, than clamp it to those values instead
        
        Update area diff (for the derivative term) and prev Angle (for the loop conditional -> Loop is working on the condition that the car is not driving straight)

        ...

        If its been 12 rounds and the car is basically straight stop the car

         (cross track error) Ep : how far we are from the center (or middle)
     is multiplied with Proportional Gain (usually a high gain means faster proportion to center)

     Proportional Control is better than Bang-Bang control (when there is a fixed angle you turn toward to meet the cenetr)
    # However, there is a chance that the car may meet the center line diagonally and overshoot consistently 

    We can improve it by adding a derivative term (how much we move perpindicularly to the desired direction)
        if we are pefectly follow the center the Cross track error rate ()
        This is also multiplied to a pD gain to create the best steering angle along with Proportion 

    


"""