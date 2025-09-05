import cv2
import time 
from picamera2 import Picamera2
import numpy as np

# Initialize camera
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.controls.FrameRate = 30
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()




ROI_LEFT_WALL = [30, 230, 120, 370] 
ROI_RIGHT_WALL = [510, 230, 600, 370]  

#ROI_MAIN = [0, 180, 640, 350]
#ROI_LINE = [200, 320, 440, 370]

#ROI_CENTER = [150, 270, 490, 330]

#ROI_BLACK = [250, 400, 390, 450]   # x1, y1 (top), x2, y2 (bottom)




    
    
#ROI_UPP1 = [230, 240, 280, 290]
#ROI_UPP2 = [400, 240, 450, 290]

#redTarget = 140 
#greenTarget = 530

# FPS calculation setup
prev_time = time.time()

# Function to draw ROI rectangles

def display_roi(img, ROIs, color):
    for ROI in ROIs:
        img = cv2.line(img, (ROI[0], ROI[1]), (ROI[2], ROI[1]), color, 4)
        img = cv2.line(img, (ROI[0], ROI[1]), (ROI[0], ROI[3]), color, 4)
        img = cv2.line(img, (ROI[2], ROI[3]), (ROI[2], ROI[1]), color, 4)
        img = cv2.line(img, (ROI[2], ROI[3]), (ROI[0], ROI[3]), color, 4)
    return img 

    
while True:
    frame = picam2.capture_array()
    frame = display_roi(frame, (255, 204, 0)) [ROI_LEFT_WALL, ROI_RIGHT_WALL]
	
    y1 = 0
    y2 = frame.shape[0]  # height of the image

    # Draw red line at x = redTarget
   # cv2.line(frame, (redTarget, y1), (redTarget, y2), (0, 0, 255), 2)
    # Draw green line at x = greenTarget
  #  cv2.line(frame, (greenTarget, y1), (greenTarget, y2), (0, 255, 0), 2)
		
#Calculate FPS
    current_time = time.time()
    fps = 1 / (current_time - prev_time)
    prev_time = current_time

    # Display FPS on frame
    cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    cv2.imshow('USB Camera Feed', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        picam2.stop()
        cv2.destroyAllWindows()
        break


# OLD PROGRAM FLOW
"""
	while True:
		frame = picam2.capture_array()
		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) 

		# Add Lines
		frame = display_roi(frame,  [ROI1, ROI2, ROI3], (255, 204, 0))

		# Get Contours
		cListLeft = get_contours(ROI1, hsv, rBlack )
		cListRight = get_contours(ROI2, hsv, rBlack )
		cListLine = get_contours(ROI3, hsv, rOrange)

		# Find Area of Left Contours --> Make this into a function
		areaLeft = detect_contour(cListLeft, ROI1, frame)
		areaRight = detect_contour(cListRight, ROI2, frame)
		
		
		# Line
		areaLine = detect_contour(cListLine, ROI3, frame)
		if areaLine > 100:
				line_detected = True
		else:
			line_detected = False

		# find the area difference (Cross track error)
		areaDiff = areaLeft - areaRight
		print("AREA DIFFERENCE: ", areaDiff)
		derivative_term = areaDiff - prevAreaDiff # Cross track error rate (to control)
		
		last_detection_time = 0  # Timestamp of last orange line count	
		detection_cooldown = 1.5  # Cooldown in seconds between counts

		
		#prev_angle = angle
		
		# PROGRAM FLOW -------
		#If not in right or left turn, if the thresholds are met to initiate a turn, then turn, otherwise, keep correcting to center via pd steering
		if not right_turn and not left_turn:
			if line_detected:
				right_turn = True 
				#print("RIGHT") 
			else:
				angle = int(MID_SERVO + (kp * areaDiff) + (kd * derivative_term)) 
				#angle = max(-TURN_DEGREE, min(angle, TURN_DEGREE))
				if angle>25:
					angle = 25
				elif angle<-25:
					angle = -25
					
				
				#pwm.SweepServo(SERVO_MOTOR_PWM3, angle)  -- COMAND
				servo_angle_queue.put(angle)

			#elif areaLeft < threshold_to_start_turn and areaRight > threshold_to_start_turn:
				#left_turn = True 
				#print("LEFT")

			#if True:
				# Clamp the angle bc you dont want the servo to be turning too far


		# If we are in right turn, either exit turn if threshold is met or continue turning with the calculated angle (clamp values if too far)
		elif right_turn :
			if areaRight < threshold_to_start_turn and areaLeft < threshold_to_start_turn and (not line_detected):
				
				right_turn = False 
				prevAreaDiff = 0 # Reset for PD Steering
				#current_time = time.time()
				counter += 1
				#if line_detected and (current_time - last_detection_time > detection_cooldown):
					
				#q	last_detection_time = current_time
				print(counter)
				
				line_detected = False 
			else:
				angle = 12
				servo_angle_queue.put(angle)
				prev_angle = angle
		
				# update values for next iteration
		
		
		
		# If we are in left turn, either exit turn if threshold is met or continue turning with the calculated angle (clamp values if too far)
		elif left_turn:
			if areaLeft > threshold_to_end_turn:
				left_turn = False 
				prevAreaDiff = 0 # Reset for PD Steering
				if line_detected: counter += 1
				line_detected = False 
					

			else:
				# Otherwise force angle to continue turning
				if angle != prev_angle:
					#pwm.SweepServo(SERVO_MOTOR_PWM3, angle)  -- COMAND
					angle = int(MID_SERVO + (kp * areaDiff) + (kd * derivative_term)) 
					angle = max(-TURN_DEGREE, min(angle, TURN_DEGREE))
					servo_angle_queue.put(angle)
					prev_angle = angle
		
	
"""
