# Libraries
import cv2
import math
import numpy as np
import time 
from picamera2 import Picamera2
import threading
from time import sleep 
from queue import Queue
from queue import Queue
import smbus
import board
import busio
import adafruit_vl53l0x
import RPi.GPIO as GPIO

# HAT Config
SUBADR1            = 0x02
SUBADR2            = 0x03
SUBADR3            = 0x04
MODE1              = 0x00
MODE2              = 0x01
PRESCALE           = 0xFE
LED0_ON_L          = 0x06
LED0_ON_H          = 0x07
LED0_OFF_L         = 0x08
LED0_OFF_H         = 0x09
ALLLED_ON_L        = 0xFA
ALLLED_ON_H        = 0xFB
ALLLED_OFF_L       = 0xFC
ALLLED_OFF_H       = 0xFD
  
SERVO_MOTOR_PWM3        = 6
SERVO_MOTOR_PWM4        = 7
SERVO_MOTOR_PWM5        = 8
SERVO_MOTOR_PWM6        = 9
SERVO_MOTOR_PWM7        = 10
SERVO_MOTOR_PWM8        = 11

DC_MOTOR_PWM1        = 0
DC_MOTOR_INA1        = 2
DC_MOTOR_INA2        = 1

DC_MOTOR_PWM2        = 5
DC_MOTOR_INB1        = 3
DC_MOTOR_INB2        = 4
BUTTON_PIN = 17
# Pin 9, GND 11 on the HAT

# Button Config
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Pull-up resistor enabled



i2c = busio.I2C(board.SCL, board.SDA)

kernel = np.ones((5, 5), np.uint8)   # a 5x5 square kernel




# Global Functions

class PCA9685:
	def __init__(self):
		self.i2c = smbus.SMBus(1)
		self.dev_addr = 0x7f
		self.write_reg(MODE1, 0x00)

	def write_reg(self, reg, value):
		self.i2c.write_byte_data(self.dev_addr, reg, value)

	def read_reg(self, reg):
		res = self.i2c.read_byte_data(self.dev_addr, reg)
		return res

	def setPWMFreq(self, freq):
		prescaleval = 25000000.0    # 25MHz
		prescaleval /= 4096.0       # 12-bit
		prescaleval /= float(freq)
		prescaleval -= 1.0
		prescale = math.floor(prescaleval + 0.5)

		oldmode = self.read_reg(MODE1)
		print('lodmode:',oldmode)
		newmode = (oldmode & 0x7F) | 0x10  # sleep
		self.write_reg(MODE1, newmode)        # go to sleep
		self.write_reg(PRESCALE, int(math.floor(prescale)))
		self.write_reg(MODE1, oldmode)
		time.sleep(0.005)
		self.write_reg(MODE1, oldmode | 0x80)  # 0x80

	def setPWM(self, ch, on, off):
		self.write_reg(LED0_ON_L+4*ch, on & 0xFF)
		self.write_reg(LED0_ON_H+4*ch, on >> 8)
		self.write_reg(LED0_OFF_L+4*ch, off & 0xFF)
		self.write_reg(LED0_OFF_H+4*ch, off >> 8)

	def setServoPulse(self, channel, pulse):
	   pulse = pulse*4096/20000        #PWM frequency is 50HZ,the period is 20000us=20ms
	   self.setPWM(channel, 0, int(pulse))
			  
	def Drive_time(self, channel, percent, wise, tim, pwm):
		pulse = int(15000 * percent)
		pwm.setServoPulse(channel, pulse)
		if wise == "CW":
			pwm.setServoPulse(DC_MOTOR_INA1,19999) # set INA1 H 
			pwm.setServoPulse(DC_MOTOR_INA2,0) # set INA2 L
		elif wise == "CCW":
			pwm.setServoPulse(DC_MOTOR_INA1,0) # set INA1 H 
			pwm.setServoPulse(DC_MOTOR_INA2,19999) # set INA2 L
		elif wise == "Stop":
			pwm.setServoPulse(DC_MOTOR_INA1,19999) # set INA1 H 
			pwm.setServoPulse(DC_MOTOR_INA2,19999) # set INA2 L
		elif wise == "Coast":
			pwm.setServoPulse(DC_MOTOR_INA1,0) # set INA1 H 
			pwm.setServoPulse(DC_MOTOR_INA2,0) # set INA2 L
		else:
			print("Error, invalid entry, if you have forgotten, options are: 1)CW 2)CCW 3)Stop or 4)Coast")
		time.sleep(tim)
		
	def Drive(self, channel, percent, wise, pwm):
		pulse = int(15000 * percent)
		pwm.setServoPulse(channel, pulse)
		if wise == "CW":
			pwm.setServoPulse(DC_MOTOR_INA1,19999) # set INA1 H 
			pwm.setServoPulse(DC_MOTOR_INA2,0) # set INA2 L
		elif wise == "CCW":
			pwm.setServoPulse(DC_MOTOR_INA1,0) # set INA1 H 
			pwm.setServoPulse(DC_MOTOR_INA2,19999) # set INA2 L
		elif wise == "Stop":
			pwm.setServoPulse(DC_MOTOR_INA1,19999) # set INA1 H 
			pwm.setServoPulse(DC_MOTOR_INA2,19999) # set INA2 L
		elif wise == "Coast":
			pwm.setServoPulse(DC_MOTOR_INA1,0) # set INA1 H 
			pwm.setServoPulse(DC_MOTOR_INA2,0) # set INA2 L
		else:
			print("Error, invalid entry, if you have forgotten, options are: 1)CW 2)CCW 3)Stop or 4)Coast")
		
		
		
	def MoveServo(self, channel, angle, delay=0.02):
		physical_angle = angle - 50

		physical_angle = max(-90, min(90, physical_angle))
	
		pulse = int(1500 + (physical_angle / 90.0) * 1000)

		self.setServoPulse(channel, pulse)
		time.sleep(delay)

	def SweepServo(self, channel, end_angle, step=1, delay=0.002):
		"""
		Smoothly sweeps from last known logical angle to the end_angle.
		All angles are in user input degrees and offset by -37 internally.
		"""
		end_angle = max(-37, min(40, end_angle))
	
		if not hasattr(self, '_servo_positions'):
			self._servo_positions = {}

		start_angle = self._servo_positions.get(channel, 0)

		if start_angle < end_angle:
			angles = range(start_angle, end_angle + 1, step)
		else:
			angles = range(start_angle, end_angle - 1, -step)
	
		for angle in angles:
			self.MoveServo(channel, angle, delay)
	
		self._servo_positions[channel] = end_angle
			
	def DisableServo(self, channel):
		"""
		Disables the PWM signal to the servo on the specified channel.
		This will stop holding torque and let it float freely.
		"""
		self.setPWM(channel, 0, 0)
		
	def TOF():
		distance_mm = vl53.range
		
		"""
		If we ever choose to or need to use a time of flight sensor, this is the function to utilize it.
		"""      
		
pwm = PCA9685()
pwm.setPWMFreq(50)
motor_command_queue = Queue()
servo_angle_queue = Queue()

def motor_drive():
    current_command = None
    while True:
        command = motor_command_queue.get()
        if command is None:
            break
        if command == current_command:
            continue

        if command == "drive":
            print("Drive")
            pwm.Drive(DC_MOTOR_PWM1, 0.7, "CW", pwm)

        elif command == "backtrack":
            # brief brake helps some drivers before reversing
            pwm.Drive(DC_MOTOR_PWM1, 1.0, "Stop", pwm)
            time.sleep(0.05)
            pwm.Drive(DC_MOTOR_PWM1, 1.0, "CCW", pwm)
            print("BACKTRACK...")

        elif command == "stop":
            print("Stop")
            pwm.Drive(DC_MOTOR_PWM1, 1.0, "Stop", pwm)

        current_command = command

def servo_move():
	current_angle = None
	while True:
		angle = servo_angle_queue.get()  # Wait for new command

		if angle is None:
			pwm.DisableServo(SERVO_MOTOR_PWM3)
			current_angle = None
			print("Servo disabled")
			break
		else:
			#print("ANGLE VALUE:", angle)
			pwm.SweepServo(SERVO_MOTOR_PWM3, angle)
			current_angle = angle

		time.sleep(0.005) 
		
# Helper for largest contour info inside a binary ROI-mask
def largest_contour_info(mask, roi_origin):
    # mask should already be the ROI-sized mask (not full frame)
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    if not contours:
        return 0, None, 0
    largest = max(contours, key=cv2.contourArea)
    area = int(cv2.contourArea(largest))
    x, y, w, h = cv2.boundingRect(largest)
    abs_x = x + roi_origin[0]
    abs_y = y + roi_origin[1]
    return area, (abs_x, abs_y, w, h), h



def main():
    t1 = threading.Thread(target=motor_drive, name='t1')
    t2 = threading.Thread(target=servo_move, name='t2')
    t1.start()
    t2.start()
    
    run()
    
    t1.join()
    t2.join()
    
def run():
    
    
    i = 0
    side = 0

    ang = 0

    turn_End = False

    turn_just_ended = False

    stop_Turn = False

    park_DIR = None

    line_detected = False

    # Camera Setup
    picam2 = Picamera2()
    picam2.preview_configuration.main.size =(640,480)
    picam2.preview_configuration.main.format = "RGB888"
    picam2.preview_configuration.controls.FrameRate = 30
    picam2.preview_configuration.align()
    picam2.configure("preview")
    picam2.start()

    # Define ROIs
    ROI_LEFT_WALL = [30, 250, 120, 400] 
    ROI_RIGHT_WALL = [510, 250, 600, 400]  
    ROI_MAIN = [0, 180, 640, 350]
    ROI_LINE = [200, 320, 440, 370]
    ROI_CENTER = [150, 270, 490, 330]
    ROI_BLACK = [250, 400, 390, 450]   # x1, y1 (top), x2, y2 (bottom)
    ROI_UPP1 = [230, 240, 280, 290]
    ROI_UPP2 = [400, 240, 450, 290]

    # PD Steering - Pillars
    kp = 0.040
    kd = 0.005
    
    # PD Steering - Walls
    kp_walls = 0.09
    kd_walls = 0.0004

    yKp = 0.0015 # How much it should steer relative to the y-axis of the pillar (more if closer)

    # Color Ranges
    rBlack = [np.array([0,110,112]), np.array([60, 149, 154])]
    rBlue = [np.array([0, 0, 0]), np.array([200, 255, 125])]
    rOrange = [np.array([0, 162, 176]), np.array([255,196,204])]
    rGreen = [np.array([0, 40, 0]), np.array([255, 120, 170])]
    rRed = [np.array([0,158,145]), np.array([131,196,171])]
    rMagenta = [np.array([0,166, 107]), np.array([255,196,144])]

    # Thresholds General
    line_threshold = 50
    pillar_threshold = 550

    #Turning Logic Variables
    threshold_to_start_turn = 300 
    threshold_to_end_turn = 500  

    # Angles 
    default_servo = 0
    max_turn_degree = 25

    # Obstacle Avoidance PD / Pillars + Wall
    target = 0
    Green_grav_const = 600 #This is the minimum area of an obstacle/pillar hat is necessary for it to start detecting
    Red_grac_const = 700

    # before while True:
    backtrack_active = False
    backtrack_end_time = 0.0
    backtrack_duration = 1.0   # how long the backtrack should last (s)
    turn_Deg = 8
   
    redTarget = 140 
    greenTarget = 580
    
    # ERRORS
    pillar_error = 0
    prev_pillar_error = 0 
    
    # FPS calculation setup
    prev_time = time.time()
    wall_error = 0
    prev_wall_error = 0

    # Turning 
    turn_counter = 0
    turn_Dir = None
    track_Dir = None
    line_seen = False
    three_laps = False
    TURN_DEGREE = 18
    
    line_released = True
    waiting_for_release = False
    
    angle = default_servo          # <--- prevents NameError when doing angle += / -=

    # -------- FUNCTIONS ---------
    
    def display_roi(img, ROIs, color):
        for ROI in ROIs:
            img = cv2.line(img, (ROI[0], ROI[1]), (ROI[2], ROI[1]), color, 4)
            img = cv2.line(img, (ROI[0], ROI[1]), (ROI[0], ROI[3]), color, 4)
            img = cv2.line(img, (ROI[2], ROI[3]), (ROI[2], ROI[1]), color, 4)
            img = cv2.line(img, (ROI[2], ROI[3]), (ROI[0], ROI[3]), color, 4)

        return img 

    def get_contours(ROI, lab, ranges): 

        segmented_area = lab[ROI[1]: ROI[3], ROI[0]:ROI[2]]
        
        # Protect against invalid region
        if segmented_area is None or segmented_area.size == 0:
            return []
        
        mask = cv2.inRange(segmented_area, ranges[0], ranges[1])
        kernel = np.ones((5,5), np.uint8)
        erode = cv2.erode(mask, kernel, iterations=1)
        dilate = cv2.dilate(erode, kernel)
        list_contour = cv2.findContours(dilate, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

        return list_contour
    
    def get_red_contours(ROI, lab, rRed): 

        segmented_area = lab[ROI[1]: ROI[3], ROI[0]:ROI[2]]
        
        # Protect against invalid region
        if segmented_area is None or segmented_area.size == 0:
            return []
        
        # destructure
        
        mask1 = cv2.inRange(segmented_area, rRed[0], rRed[1])
        #combined_mask = cv2.bitwise_or(mask1)
        kernel = np.ones((5,5), np.uint8)
        erode = cv2.erode(mask1, kernel, iterations=1)
        dilate = cv2.dilate(erode, kernel)
        list_contour = cv2.findContours(dilate, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

        return list_contour, dilate

    def detect_contour(contour, ROI, frame):

        maxArea = 0
        maxY = 0
        maxX = 0
        maxCnt = 0
        
        for cnt in contour: 
            try:
                area = cv2.contourArea(cnt)
                if area > 100:
                    approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
                    x, y, w, h = cv2.boundingRect(approx)
                    abs_x = x + ROI[0]
                    abs_y = y + ROI[1]
                    cv2.rectangle(frame, (abs_x, abs_y), (abs_x + w, abs_y + h ), (0,255,0), 2)  

                    if area > maxArea:
                        maxArea = area 
            except:
                print("Looking for vals")

        return maxArea  

    def filter_pillars(contours, frame, closest_pillar_distance, pillar_threshold, ROI):      
        
        """
        Filter through contours to find current pillar and returns pillar data 
        """
        
        pillars = []
        if contours == []:
            pass
        else:
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area <= pillar_threshold:
                    continue # start over again until it finds a pillar above threshold
                
                approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
                x, y, w, h = cv2.boundingRect(approx)
                
                x += ROI[0] 
                y += ROI[1] 
                
                pillar_distance = math.dist([x + w // 2, y], [320, 480]) # center top to center_bottom screen
                if pillar_distance >= closest_pillar_distance:
                    continue # try again 

                 
                cv2.rectangle(frame, (x,y), (x + w, y + h), (0,255,255), 1)
                
                pillars.append({
                    "x": x + w // 2, # Center of width
                    "y": y,
                    "h": h,
                    "w": w,
                    "area": area,
                    "distance": pillar_distance           
                })
                       
        return pillars
        


    #----- Parallel Parking Loop------

    picam2.start()
    
    pillar_detected = None # Stores the current pillar and next one if spotted
    
    # Pillar Closest Data
    closest_pillar_distance = 10000 # relatively large num, meant to be overridee
    closest_pillar_color = None
    closest_pillar_x = None
    closest_pillar_y = None
    closest_pillar_area = 0
            
    # Setup display
    frame = picam2.capture_array()
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2Lab) 
    frame = display_roi(frame,  [ROI_LEFT_WALL, ROI_RIGHT_WALL, ROI_MAIN, ROI_CENTER, ROI_LINE, ROI_BLACK, ROI_UPP1, ROI_UPP2], (255, 204, 0))
    
    y1 = 0
    y2 = frame.shape[0]
    
    # Get contours of the Line, wall, pillar
    cLeftWall = get_contours(ROI_LEFT_WALL, lab, rBlack )
    cRightWall = get_contours(ROI_RIGHT_WALL, lab, rBlack)
    
    cLW_UPP = get_contours(ROI_UPP1, lab, rBlack )
    cRW_UPP = get_contours(ROI_UPP2, lab, rBlack )
    
    
    cCenterWall = get_contours(ROI_CENTER, lab, rBlack)
    
    cListLineOrange = get_contours(ROI_LINE, lab, rOrange)
    cListLineBlue = get_contours(ROI_LINE, lab, rBlue )
    
    cListPillarGreen = get_contours(ROI_MAIN, lab, rGreen)
    cListPillarRed, dilate = get_red_contours(ROI_MAIN, lab, rRed)
    
    cListCoreRed = get_contours(ROI_CENTER, lab, rRed)
    cListCoreGreen = get_contours(ROI_CENTER, lab, rGreen)
    cListCoreBlack = get_contours(ROI_BLACK, lab, rBlack)

    # Center black: we'll create a mask and analyzeqqq largest contour
    center_seg_black = lab[ROI_BLACK[1]:ROI_BLACK[3], ROI_BLACK[0]:ROI_BLACK[2]]
    black_mask = cv2.inRange(center_seg_black, rBlack[0], rBlack[1])
    black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_OPEN, kernel, iterations=1)
    black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_CLOSE, kernel, iterations=1)

    
    # Area of Lines
    areaLineOrange = detect_contour(cListLineOrange, ROI_LINE, frame)
    areaLineBlue = detect_contour(cListLineBlue, ROI_LINE, frame)
          
    # Wall areas
    areaLeft = detect_contour(cLeftWall, ROI_LEFT_WALL, frame)
    areaRight = detect_contour(cRightWall, ROI_RIGHT_WALL, frame)
    
    areaRightUPP = detect_contour(cRW_UPP, ROI_UPP2, frame)
    areaLeftUPP = detect_contour(cLW_UPP, ROI_UPP1, frame)
    
    # CENTER FOR BACKTRACK
    areaRedCenter = detect_contour(cListCoreRed, ROI_CENTER, frame)
    areaGreenCenter = detect_contour(cListCoreGreen, ROI_CENTER, frame)
    
    # Compute largest black contour info for ROI_BLACK
    areaBlackCenter, black_bbox, black_box_h = largest_contour_info(black_mask, ROI_BLACK)
    
    
            
    # Calculate FPS
    current_time = time.time()
    fps = 1 / (current_time - prev_time)
    prev_time = current_time

    # Display FPS on frame
    cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30),
    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                       
    cv2.imshow('USB CameFd', frame)
    
    key = cv2.waitKey(1) & 0xFF
    



    if areaRight > areaLeft:
        while True:
            print("CLOCK")
            picam2.start()
        
            pillar_detected = None # Stores the current pillar and next one if spotted
        
            # Pillar Closest Data
            closest_pillar_distance = 10000 # relatively large num, meant to be overridee
            closest_pillar_color = None
            closest_pillar_x = None
            closest_pillar_y = None
            closest_pillar_area = 0
                
            # Setup display
            frame = picam2.capture_array()
            lab = cv2.cvtColor(frame, cv2.COLOR_BGR2Lab) 
            frame = display_roi(frame,  [ROI_LEFT_WALL, ROI_RIGHT_WALL, ROI_MAIN, ROI_CENTER, ROI_LINE, ROI_BLACK, ROI_UPP1, ROI_UPP2], (255, 204, 0))
        
            y1 = 0
            y2 = frame.shape[0]
        
            # Get contours of the Line, wall, pillar
            cLeftWall = get_contours(ROI_LEFT_WALL, lab, rBlack )
            cRightWall = get_contours(ROI_RIGHT_WALL, lab, rBlack)
        
            cLW_UPP = get_contours(ROI_UPP1, lab, rBlack )
            cRW_UPP = get_contours(ROI_UPP2, lab, rBlack )
        
        
            cCenterWall = get_contours(ROI_CENTER, lab, rBlack)
        
            cListLineOrange = get_contours(ROI_LINE, lab, rOrange)
            cListLineBlue = get_contours(ROI_LINE, lab, rBlue )
        
            cListPillarGreen = get_contours(ROI_MAIN, lab, rGreen)
            cListPillarRed, dilate = get_red_contours(ROI_MAIN, lab, rRed)
        
            cListCoreRed = get_contours(ROI_CENTER, lab, rRed)
            cListCoreGreen = get_contours(ROI_CENTER, lab, rGreen)
            cListCoreBlack = get_contours(ROI_BLACK, lab, rBlack)

            # Center black: we'll create a mask and analyze largest contour
            center_seg_black = lab[ROI_BLACK[1]:ROI_BLACK[3], ROI_BLACK[0]:ROI_BLACK[2]]
            black_mask = cv2.inRange(center_seg_black, rBlack[0], rBlack[1])
            black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_OPEN, kernel, iterations=1)
            black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_CLOSE, kernel, iterations=1)

        
            # Area of Lines
            areaLineOrange = detect_contour(cListLineOrange, ROI_LINE, frame)
            areaLineBlue = detect_contour(cListLineBlue, ROI_LINE, frame)
              
            # Wall areas
            areaLeft = detect_contour(cLeftWall, ROI_LEFT_WALL, frame)
            areaRight = detect_contour(cRightWall, ROI_RIGHT_WALL, frame)
            areaRightUPP = detect_contour(cRW_UPP, ROI_UPP2, frame)
            areaLeftUPP = detect_contour(cLW_UPP, ROI_UPP1, frame)
        
            # CENTER FOR BACKTRACK
            areaRedCenter = detect_contour(cListCoreRed, ROI_CENTER, frame)
            areaGreenCenter = detect_contour(cListCoreGreen, ROI_CENTER, frame)
        
            # Compute largest black contour info for ROI_BLACK
            areaBlackCenter, black_bbox, black_box_h = largest_contour_info(black_mask, ROI_BLACK)
                
            # Calculate FPS
            current_time = time.time()
            fps = 1 / (current_time - prev_time)
            prev_time = current_time

            # Display FPS on frame
            cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                           
            cv2.imshow('USB CameFd', frame)
        
            key = cv2.waitKey(1) & 0xFF
        
            # ----- RED PILLAR DETECTION ------
        
            # Stores Red Pillar Data: area, cords, and distance
            PillarsRed = filter_pillars(cListPillarRed, frame, closest_pillar_distance, pillar_threshold, ROI_MAIN)
       
            for pillar in PillarsRed:
            
                # ignore the pillar if it is not the closest (determined by distance)
                if (pillar["area"] < closest_pillar_area * 0.9) and (pillar["distance"] > closest_pillar_distance * 0.9):
                    continue
                
                # CASE A: Pillar is too far# pillar["y"] + pillar["h"]) >= 370 and 
                if pillar["x"] < 20:
                    previous_pillar = "red"
                    print("PILLAR HAS BEEN PASSED")
                    pillar_detected  = False
                    continue 
                
                # CASE B: pillar is passed
                elif (pillar["y"] + pillar["h"] ) < 155:
                    pillar_detected  = False
                    print("Pillar too far")
                    continue
            
                # CASE C: Update Pillar data
                else:
                    closest_pillar_distance = pillar["distance"]
                    closest_pillar_color = "red"
                    closest_pillar_x = pillar["x"]
                    closest_pillar_y = pillar["y"]
                    closest_pillar_area = pillar["area"]
                    pillar_detected = True

                        
        
            # ----- GREEN PILLAR DETECTION ------
        
            PillarsGreen = filter_pillars(cListPillarGreen, frame, closest_pillar_distance, pillar_threshold, ROI_MAIN)
        
            for pillar in PillarsGreen:

                # ignore the pillar if it is not the closest (determined by distance)
                if (pillar["area"] < closest_pillar_area * 0.9) and (pillar["distance"] > closest_pillar_distance * 0.9):
                    continue

                # CASE A: pillar is close and pass the target zone horizontally #pillar["y"] + pillar["h"]) >= 370 and
                if pillar["x"] > greenTarget:
                    previous_pillar = "green"
                    print("PILLAR HAS BEEN PASSED")
                    pillar_detected = False
                    continue 
                
                # CASE B: bottom_y_cord of the pillar is too far
                elif (pillar["y"] + pillar["h"] ) < 155: # Soft mark for too far (frame is 100-400 on the y)
                    print("Pillar too far")
                    pillar_detected  = False
                    continue 
            
                # CASE C: Update Pillar data
                else:
                    closest_pillar_distance = pillar["distance"]
                    closest_pillar_color = "green"
                    closest_pillar_x = pillar["x"]
                    closest_pillar_y = pillar["y"]
                    closest_pillar_area = pillar["area"]
                    pillar_detected = True
   
            # ---- TARGET SELECTION ----        
        
            if closest_pillar_color == "red" and closest_pillar_area > Red_grac_const:
                target = redTarget
                cv2.line(frame, (redTarget, y1), (redTarget, y2), (0, 0, 255), 2)
            elif closest_pillar_color == "green" and closest_pillar_area > Green_grav_const:
                target = greenTarget
                cv2.line(frame, (greenTarget, y1), (greenTarget, y2), (0, 255, 0), 2)
            else:
                # Treat as "no pillar" so wall/black logic can run
                target = 0
                
            servo_angle_queue.put(-30)
            pwm.Drive(DC_MOTOR_PWM1, 0.45, "CW", pwm)
            print("ALMOST")
            
            if areaLeftUPP >= 1 and areaLeftUPP <= 8000: 
                print("AHA")
                pwm.Drive(DC_MOTOR_PWM1, 1.0, "Stop", pwm)
                print(closest_pillar_color)
                print("A")
                print(areaLeftUPP)
             
                
 
                if closest_pillar_color == None or closest_pillar_color == "red":
                    servo_angle_queue.put(30)	 
                    pwm.Drive_time(DC_MOTOR_PWM1, 0.5, "CW", 1.0, pwm)
                    pwm.Drive(DC_MOTOR_PWM1, 1.0, "Stop", pwm)
                                    
                elif closest_pillar_color == "green":
                    servo_angle_queue.put(0)	 
                    pwm.Drive_time(DC_MOTOR_PWM1, 1.0, "CW", 1.0, pwm)
                    servo_angle_queue.put(15)	 
                    pwm.Drive_time(DC_MOTOR_PWM1, 1.0, "Cw", 1.0, pwm)

                else:
                    print("Hath, thy error aprroaches me")
                
            break
    
    elif areaLeft > areaRight:
        print("COUNTER")
        while True:
            picam2.start()
        
            pillar_detected = None # Stores the current pillar and next one if spotted
        
            # Pillar Closest Data
            closest_pillar_distance = 10000 # relatively large num, meant to be overridee
            closest_pillar_color = None
            closest_pillar_x = None
            closest_pillar_y = None
            closest_pillar_area = 0
                
            # Setup display
            frame = picam2.capture_array()
            lab = cv2.cvtColor(frame, cv2.COLOR_BGR2Lab) 
            frame = display_roi(frame,  [ROI_LEFT_WALL, ROI_RIGHT_WALL, ROI_MAIN, ROI_CENTER, ROI_LINE, ROI_BLACK, ROI_UPP1, ROI_UPP2], (255, 204, 0))
        
            y1 = 0
            y2 = frame.shape[0]
        
            # Get contours of the Line, wall, pillar
            cLeftWall = get_contours(ROI_LEFT_WALL, lab, rBlack )
            cRightWall = get_contours(ROI_RIGHT_WALL, lab, rBlack)
        
            cLW_UPP = get_contours(ROI_UPP1, lab, rBlack )
            cRW_UPP = get_contours(ROI_UPP2, lab, rBlack )
        
        
            cCenterWall = get_contours(ROI_CENTER, lab, rBlack)
        
            cListLineOrange = get_contours(ROI_LINE, lab, rOrange)
            cListLineBlue = get_contours(ROI_LINE, lab, rBlue )
        
            cListPillarGreen = get_contours(ROI_MAIN, lab, rGreen)
            cListPillarRed, dilate = get_red_contours(ROI_MAIN, lab, rRed)
        
            cListCoreRed = get_contours(ROI_CENTER, lab, rRed)
            cListCoreGreen = get_contours(ROI_CENTER, lab, rGreen)
            cListCoreBlack = get_contours(ROI_BLACK, lab, rBlack)

            # Center black: we'll create a mask and analyze largest contour
            center_seg_black = lab[ROI_BLACK[1]:ROI_BLACK[3], ROI_BLACK[0]:ROI_BLACK[2]]
            black_mask = cv2.inRange(center_seg_black, rBlack[0], rBlack[1])
            black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_OPEN, kernel, iterations=1)
            black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_CLOSE, kernel, iterations=1)

        
            # Area of Lines
            areaLineOrange = detect_contour(cListLineOrange, ROI_LINE, frame)
            areaLineBlue = detect_contour(cListLineBlue, ROI_LINE, frame)
              
            # Wall areas
            areaLeft = detect_contour(cLeftWall, ROI_LEFT_WALL, frame)
            areaRight = detect_contour(cRightWall, ROI_RIGHT_WALL, frame)
            areaRightUPP = detect_contour(cRW_UPP, ROI_UPP2, frame)
            areaLeftUPP = detect_contour(cLW_UPP, ROI_UPP1, frame)
        
            # CENTER FOR BACKTRACK
            areaRedCenter = detect_contour(cListCoreRed, ROI_CENTER, frame)
            areaGreenCenter = detect_contour(cListCoreGreen, ROI_CENTER, frame)
        
            # Compute largest black contour info for ROI_BLACK
            areaBlackCenter, black_bbox, black_box_h = largest_contour_info(black_mask, ROI_BLACK)                
                
            # Calculate FPS
            current_time = time.time()
            fps = 1 / (current_time - prev_time)
            prev_time = current_time

            # Display FPS on frame
            cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                           
            cv2.imshow('USB CameFd', frame)
        
            key = cv2.waitKey(1) & 0xFF
        
            # ----- RED PILLAR DETECTION ------
        
            PillarsRed = filter_pillars(cListPillarRed, frame, closest_pillar_distance, pillar_threshold, ROI_MAIN)
       
            for pillar in PillarsRed:
            
                # Ignore the pillar if it is not the closest (determined by distance)
                if (pillar["area"] < closest_pillar_area * 0.9) and (pillar["distance"] > closest_pillar_distance * 0.9):
                    continue
                
                # CASE A: Pillar is too far# pillar["y"] + pillar["h"]) >= 370 and 
                if pillar["x"] < 20:
                    previous_pillar = "red"
                    print("PILLAR HAS BEEN PASSED")
                    pillar_detected  = False
                    continue 
                
                # CASE B: pillar is passed
                elif (pillar["y"] + pillar["h"] ) < 155:
                    pillar_detected  = False
                    print("Pillar too far")
                    continue
            
                # CASE C: Update Pillar data
                else:
                    closest_pillar_distance = pillar["distance"]
                    closest_pillar_color = "red"
                    closest_pillar_x = pillar["x"]
                    closest_pillar_y = pillar["y"]
                    closest_pillar_area = pillar["area"]
                    pillar_detected = True                        
        
            # ----- GREEN PILLAR DETECTION ------
        
            PillarsGreen = filter_pillars(cListPillarGreen, frame, closest_pillar_distance, pillar_threshold, ROI_MAIN)
        
            for pillar in PillarsGreen:

                # ignore the pillar if it is not the closest (determined by distance)
                if (pillar["area"] < closest_pillar_area * 0.9) and (pillar["distance"] > closest_pillar_distance * 0.9):
                    continue

                # CASE A: pillar is close and pass the target zone horizontally #pillar["y"] + pillar["h"]) >= 370 and
                if pillar["x"] > greenTarget:
                    previous_pillar = "green"
                    print("PILLAR HAS BEEN PASSED")
                    pillar_detected = False
                    continue 
                
                # CASE B: bottom_y_cord of the pillar is too far
                elif (pillar["y"] + pillar["h"] ) < 155: # Soft mark for too far (frame is 100-400 on the y)
                    print("Pillar too far")
                    pillar_detected  = False
                    continue 
            
                # CASE C: Update Pillar data
                else:
                    closest_pillar_distance = pillar["distance"]
                    closest_pillar_color = "green"
                    closest_pillar_x = pillar["x"]
                    closest_pillar_y = pillar["y"]
                    closest_pillar_area = pillar["area"]
                    pillar_detected = True   
        
                       
            # ---- TARGET SELECTION ----

            if closest_pillar_color == "red" and closest_pillar_area > Red_grac_const:
                target = redTarget
                cv2.line(frame, (redTarget, y1), (redTarget, y2), (0, 0, 255), 2)
            elif closest_pillar_color == "green" and closest_pillar_area > Green_grav_const:
                target = greenTarget
                cv2.line(frame, (greenTarget, y1), (greenTarget, y2), (0, 255, 0), 2)
            else:
                # Treat as "no pillar" so wall/black logic can run
                pass
                    
            servo_angle_queue.put(30)

            # Start motor at 0.1 speed CW
            pwm.Drive(DC_MOTOR_PWM1, 0.45, "CW", pwm)
            print("ALMOST(1)")
            
            
            if areaRightUPP >= 1 and areaRightUPP <= 8000: 
                print("AHA(CW)")
                pwm.Drive(DC_MOTOR_PWM1, 1.0, "Stop", pwm)
                print(closest_pillar_color)
             
                if closest_pillar_color == None or closest_pillar_color == "green":
                    print("N")
                    servo_angle_queue.put(-30)	 
                    pwm.Drive_time(DC_MOTOR_PWM1, 0.5, "CW", 1.0, pwm)
                    pwm.Drive(DC_MOTOR_PWM1, 1.0, "Stop", pwm)
                        
                elif closest_pillar_color == "red":
                    print("R")
                    servo_angle_queue.put(0)	 
                    pwm.Drive_time(DC_MOTOR_PWM1, 1.0, "CW", 1.0, pwm)
                    servo_angle_queue.put(-30)	 
                    pwm.Drive(DC_MOTOR_PWM1, 1.0, "Stop", pwm)

                else:
                    print("Hath, thy error aprroaches me")
                
            break

    # ----- MAIN LOOP --------
    time.sleep(1)
    motor_command_queue.put("drive") 
    servo_angle_queue.put(0)
    while True:
        
        if areaLineBlue > line_threshold:
            print("DETECTING BLUE")
        if areaLineOrange > line_threshold:
            print("DETECTING ORANGE")

        
        pillar_detected = None # Stores the current pillar and next one if spotted
        
        # Pillar Closest Data
        closest_pillar_distance = 10000 # relatively large num, meant to be overridee
        closest_pillar_color = None
        closest_pillar_x = None
        closest_pillar_y = None
        closest_pillar_area = 0
                
        # Setup display
        frame = picam2.capture_array()
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2Lab) 
        frame = display_roi(frame,  [ROI_LEFT_WALL, ROI_RIGHT_WALL, ROI_MAIN, ROI_CENTER, ROI_LINE, ROI_BLACK], (255, 204, 0))
        
        y1 = 0
        y2 = frame.shape[0]
        
        # Get contours of the Line, wall, pillar
        cLeftWall = get_contours(ROI_LEFT_WALL, lab, rBlack )
        cRightWall = get_contours(ROI_RIGHT_WALL, lab, rBlack )
        cCenterWall = get_contours(ROI_CENTER, lab, rBlack)
        
        cListLineOrange = get_contours(ROI_LINE, lab, rOrange)
        cListLineBlue = get_contours(ROI_LINE, lab, rBlue )
        
        cListPillarGreen = get_contours(ROI_MAIN, lab, rGreen)
        cListPillarRed, dilate = get_red_contours(ROI_MAIN, lab, rRed)
        
        cListCoreRed = get_contours(ROI_CENTER, lab, rRed)
        cListCoreGreen = get_contours(ROI_CENTER, lab, rGreen)
        cListCoreBlack = get_contours(ROI_BLACK, lab, rBlack)

        # Center black: we'll create a mask and analyze largest contour
        center_seg_black = lab[ROI_BLACK[1]:ROI_BLACK[3], ROI_BLACK[0]:ROI_BLACK[2]]
        black_mask = cv2.inRange(center_seg_black, rBlack[0], rBlack[1])
        black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_OPEN, kernel, iterations=1)
        black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_CLOSE, kernel, iterations=1)

        
        # Area of Lines
        areaLineOrange = detect_contour(cListLineOrange, ROI_LINE, frame)
        areaLineBlue = detect_contour(cListLineBlue, ROI_LINE, frame)
              
        # Wall areas
        areaLeft = detect_contour(cLeftWall, ROI_LEFT_WALL, frame)
        areaRight = detect_contour(cRightWall, ROI_RIGHT_WALL, frame)
        
        # CENTER FOR BACKTRACK
        areaRedCenter = detect_contour(cListCoreRed, ROI_CENTER, frame)
        areaGreenCenter = detect_contour(cListCoreGreen, ROI_CENTER, frame)
        
        # Compute largest black contour info for ROI_BLACK
        areaBlackCenter, black_bbox, black_box_h = largest_contour_info(black_mask, ROI_BLACK)
        
        # ----- RED PILLAR DETECTION ------
        
        # Stores Red Pillar Data: area, cords, and distance
        PillarsRed = filter_pillars(cListPillarRed, frame, closest_pillar_distance, pillar_threshold, ROI_MAIN)
       
        for pillar in PillarsRed:
            
            # ignore the pillar if it is not the closest (determined by distance)
            if (pillar["area"] < closest_pillar_area * 0.9) and (pillar["distance"] > closest_pillar_distance * 0.9):
                continue
                
            # CASE A: Pillar is too far# pillar["y"] + pillar["h"]) >= 370 and 
            if pillar["x"] < 20:
                previous_pillar = "red"
                pillar_detected  = False
                continue 
                
            # CASE B: pillar is passed
            elif (pillar["y"] + pillar["h"] ) < 155:
                pillar_detected  = False
                continue
            
            # CASE C: Update Pillar data
            else:
                closest_pillar_distance = pillar["distance"]
                closest_pillar_color = "red"
                closest_pillar_x = pillar["x"]
                closest_pillar_y = pillar["y"]
                closest_pillar_area = pillar["area"]
                pillar_detected = True                        
        
        # ----- GREEN PILLAR DETECTION ------
        
        PillarsGreen = filter_pillars(cListPillarGreen, frame, closest_pillar_distance, pillar_threshold, ROI_MAIN)
        
        for pillar in PillarsGreen:

            # ignore the pillar if it is not the closest (determined by distance)
            if (pillar["area"] < closest_pillar_area * 0.9) and (pillar["distance"] > closest_pillar_distance * 0.9):
                continue

            # CASE A: pillar is close and pass the target zone horizontally #pillar["y"] + pillar["h"]) >= 370 and
            if pillar["x"] > greenTarget:
                previous_pillar = "green"
                #print("PILLAR HAS BEEN PASSED")
                pillar_detected = False
                continue 
                
            # CASE B: bottom_y_cord of the pillar is too far
            elif (pillar["y"] + pillar["h"] ) < 155: # Soft mark for too far (frame is 100-400 on the y)
                #print("Pillar too far")
                pillar_detected  = False
                continue 
            
            # CASE C: Update Pillar data
            else:
                closest_pillar_distance = pillar["distance"]
                closest_pillar_color = "green"
                closest_pillar_x = pillar["x"]
                closest_pillar_y = pillar["y"]
                closest_pillar_area = pillar["area"]
                pillar_detected = True        
                       
        # ---- TARGET SELECTION ----
        
        if closest_pillar_color == "red" and closest_pillar_area > Red_grac_const:
            target = redTarget
            cv2.line(frame, (redTarget, y1), (redTarget, y2), (0, 0, 255), 2)
        elif closest_pillar_color == "green" and closest_pillar_area > Green_grav_const:
            target = greenTarget
            cv2.line(frame, (greenTarget, y1), (greenTarget, y2), (0, 255, 0), 2)
        else:
            # Treat as "no pillar" so wall/black logic can run
            target = 0
        
        # ------- TURNING LOGIC ------
        
        if track_Dir is None and line_released and areaLineBlue > line_threshold and side!=2:
            side = 1
            track_Dir = "left"
            line_released = False  # Reset for this turn
            print(f"Turn {turn_counter} (LEFT)")
            print("---------------------------")
            print("---------------------------")
            print("---------------------------")
            print(track_Dir)
            
            print(track_Dir)

        elif track_Dir is None and line_released and areaLineOrange > line_threshold  and side!=1:
            side = 2
            track_Dir = "right"
            line_released = False
            print(track_Dir)
                                
        if track_Dir == "right" and areaLineOrange > line_threshold:
            if areaLineBlue < line_threshold and closest_pillar_color == "green":
                angle += (turn_Deg + 8)
                print("Rft")
            elif areaLineBlue < line_threshold and closest_pillar_color == "red":
                angle += (turn_Deg + 4)
                print("R")
            elif areaLineBlue < line_threshold and closest_pillar_color == None:
                angle += (turn_Deg + 8)
                print("R")
            elif areaLineBlue >= line_threshold:
                angle += (turn_Deg - turn_Deg)
                print("R")
                turn_End = True
                turn_counter += 1
                print(f"Turn {turn_counter} (RIGHT)")
                track_Dir = None 
                turn_just_ended = True
     
        if track_Dir == "left":
            print("LEFT TURN DETECTED")
            redTarget = 140
            angle -= 15
           
            if turn_counter == 12 and areaLineBlue >= line_threshold:
                turn_counter += 1
                print(f"Turn {turn_counter} (LEFT)")
                print(f"Turn {turn_counter} (LEFT)")
                print(f"Turn {turn_counter} (LEFT)")
                print(f"Turn {turn_counter} (LEFT)")
                print(f"Turn {turn_counter} (LEFT)")
                print("---------------------------")
           
            if areaLineBlue > line_threshold or track_Dir == "left":
                redTarget = 140
                angle -= 15
                print("SHARP LEFT")
                line_detected = True
            
        if areaLineOrange >= line_threshold and line_detected == True and turn_counter != 12: 
            redTarget = 140
            turn_End = True
            turn_counter += 1
            print(f"Turn {turn_counter} (LEFT)")
            print(f"Turn {turn_counter} (LEFT)")
            print(f"Turn {turn_counter} (LEFT)")
            print(f"Turn {turn_counter} (LEFT)")
            print(f"Turn {turn_counter} (LEFT)")
            print("---------------------------")
            track_Dir = None 
            turn_just_ended = True
            line_detected = False
            turn_just_ended = True

        # ------ PILLAR AVOIDING ------
        
        # If pillar color is spotted and we are not too close to the wall
        
        if pillar_detected == True and not (closest_pillar_color == "green" and areaLeft >= 9000) and not (closest_pillar_color == "red" and areaRight >= 9000):
            #print("PILLAR FOLLOWING")
            # Back track if needed or use pillars for pd steering
              
            # NEW (non-blocking):
            if (closest_pillar_color == "green" and areaGreenCenter > 4000) or (closest_pillar_color == "red" and areaRedCenter > 4000):
                 if not backtrack_active:
                     backtrack_active = True
                     backtrack_end_time = time.time() + backtrack_duration
                     servo_angle_queue.put(0)              # slight straighten
                     motor_command_queue.put("backtrack")  # reverse now
                     servo_angle_queue.put(0)
                        
            if closest_pillar_x is not None:
                pillar_error = abs(target - closest_pillar_x)
                # continue with PD control, etc.
                derivative_term = pillar_error - prev_pillar_error
            
                angle = int(default_servo + (kp * pillar_error) + (kd * derivative_term))
                angle = max(-TURN_DEGREE, min(angle, TURN_DEGREE))
                if closest_pillar_color == "red":
                    servo_angle_queue.put(angle)
                
                elif closest_pillar_color == "green":
                    servo_angle_queue.put(-angle)
            else:
                # no pillar detected this frame, maybe keep driving straight
                pillar_error = 0
    
        else:       
            # PD TERMS
            pillar_error = 0 
            wall_error = areaLeft - areaRight
            wall_derivative_term = wall_error - prev_wall_error
            angle = int(default_servo + (wall_error * kp_walls) + (wall_derivative_term * kd_walls))
            
            # If right turn ....
            if turn_Dir == "right":
                print("RIGHT TURN")
                prev_pillar_error = 0
                pillar_error = 0
                              
            # If left turn ...
            elif turn_Dir == "left":
                print("LEFT TURN")
                servo_angle_queue.put(-15)
                prev_pillar_error = 0
                pillar_error = 0
            
            # Else, wall follow
            else:
                print("WALL FOLLOWING VALUE:", angle)
                angle = max(-TURN_DEGREE, min(angle, TURN_DEGREE))
                servo_angle_queue.put(angle)              
                
            # ------ WALL AVOIDING / BACKTRACK (replace existing block) ------
        DEBUG_WALL = False  # set True to print debug info while tuning

    # ROI geometry
        roi_black_height = ROI_BLACK[3] - ROI_BLACK[1]
        roi_black_width  = ROI_BLACK[2] - ROI_BLACK[0]
        roi_area = float(max(1, roi_black_height * roi_black_width))  # avoid div-by-zero

    # Derived metrics from the detected black mask
        black_area = float(areaBlackCenter)         # detected black area inside ROI
        black_box_h = int(black_box_h) if black_box_h is not None else 0

    # Fractions: how much of the ROI is black / free (not black)
        black_fraction = black_area / roi_area
        free_fraction = 1.0 - black_fraction  # fraction of ROI that is NOT covered by black

    # thresholds (tweak these)
        free_frac_thresh = 0.90        # normal: backtrack when < 350 of ROI is free (i.e. mostly blocked)
        free_frac_thresh_turn = 0.90  # during turns: be stricter (allow smaller free area before backtrack)
        box_height_frac_thresh = 0.90  # require bounding box height >= 50% of ROI height (safety)
        box_height_thresh = int(roi_black_height * box_height_frac_thresh)

    # Only consider backtrack if there is no pillar being tracked
        if (not backtrack_active): #and (closest_pillar_color is None):
        # choose threshold depending on whether we are currently in a turn
            if turn_Dir in ["left", "right"]:
                trigger_free_thresh = free_frac_thresh_turn
            else:
                trigger_free_thresh = free_frac_thresh

        # Decide if we should start a backtrack:
        #  - ROI has low free fraction (mostly covered)
        #  - the black bbox is tall enough (to avoid small blobs)
            if (black_fraction >= trigger_free_thresh) and (black_box_h <= box_height_thresh):
                backtrack_active = True
                backtrack_end_time = time.time() + backtrack_duration
                servo_angle_queue.put(3)          # tiny straighten
                motor_command_queue.put("stop")   # brief stop then reverse
                time.sleep(0.05)
                motor_command_queue.put("backtrack")

    # End the backtrack when the timer expires
        if backtrack_active:
            if time.time() >= backtrack_end_time:
                #print("WALL: backtrack finished -> DRIVE")
                motor_command_queue.put("drive")
                backtrack_active = False
         
            
        if turn_just_ended:
            # ignore noise
            if (areaLineBlue < 10 and areaLineBlue > 0) and (areaLineOrange < 10 and areaLineOrange > 0):
                pass
                
            if (areaLineBlue < line_threshold) and (areaLineOrange < line_threshold):
                line_released = True
                turn_just_ended = False
                print("line Cleared")
        
        # --- RESET
        prev_pillar_error = pillar_error
        prev_wall_error = wall_error
        
          # Calculate FPS
        current_time = time.time()
        fps = 1 / (current_time - prev_time)
        prev_time = current_time

        # Display FPS on frame
        cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                           
        cv2.imshow('USB CameFd', frame)
        
        key = cv2.waitKey(1) & 0xFF
               
        if key == ord('d'):
            motor_command_queue.put("stop")
            servo_angle_queue.put(None)
            
        elif key == ord('q'):
            motor_command_queue.put("stop")
            servo_angle_queue.put(None)
            picam2.stop()
            cv2.destroyAllWindows()
              
        if turn_counter == 13:
            print("Completed 12 turns, stopping.")
            motor_command_queue.put("stop")
            break          
            
    servo_angle_queue.put(0)
    stop_Turn = False
    track_Dir = None
    line_released = True

    while True:
        print("CLOCK")
        picam2.start()

        pillar_detected = None # Stores the current pillar and next one if spotted

        # Pillar Closest Data
        closest_pillar_distance = 10000 # relatively large num, meant to be overridee
        closest_pillar_color = None
        closest_pillar_x = None
        closest_pillar_y = None
        closest_pillar_area = 0
        
        # Setup display
        frame = picam2.capture_array()
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2Lab) 
        frame = display_roi(frame,  [ROI_LEFT_WALL, ROI_RIGHT_WALL, ROI_MAIN, ROI_CENTER, ROI_LINE, ROI_BLACK, ROI_UPP1, ROI_UPP2], (255, 204, 0))

        y1 = 0
        y2 = frame.shape[0]

        # Get contours of the Line, wall, pillar
        cLeftWall = get_contours(ROI_LEFT_WALL, lab, rBlack )
        cRightWall = get_contours(ROI_RIGHT_WALL, lab, rBlack)
        cparking = get_contours(ROI_LEFT_WALL, lab, rMagenta)

        cLW_UPP = get_contours(ROI_UPP1, lab, rBlack )
        cRW_UPP = get_contours(ROI_UPP2, lab, rBlack )


        cCenterWall = get_contours(ROI_CENTER, lab, rBlack)

        cListLineOrange = get_contours(ROI_LINE, lab, rOrange)
        cListLineBlue = get_contours(ROI_LINE, lab, rBlue )

        cListPillarGreen = get_contours(ROI_MAIN, lab, rGreen)
        cListPillarRed, dilate = get_red_contours(ROI_MAIN, lab, rRed)

        cListCoreRed = get_contours(ROI_CENTER, lab, rRed)
        cListCoreGreen = get_contours(ROI_CENTER, lab, rGreen)
        cListCoreBlack = get_contours(ROI_BLACK, lab, rBlack)

        # Center black: we'll create a mask and analyze largest contour
        center_seg_black = lab[ROI_BLACK[1]:ROI_BLACK[3], ROI_BLACK[0]:ROI_BLACK[2]]
        black_mask = cv2.inRange(center_seg_black, rBlack[0], rBlack[1])
        black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_OPEN, kernel, iterations=1)
        black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_CLOSE, kernel, iterations=1)


        # Area of Lines
        areaLineOrange = detect_contour(cListLineOrange, ROI_LINE, frame)
        areaLineBlue = detect_contour(cListLineBlue, ROI_LINE, frame)
      
        # Wall areas
        areaLeft = detect_contour(cLeftWall, ROI_LEFT_WALL, frame)
        areaRight = detect_contour(cRightWall, ROI_RIGHT_WALL, frame)
        areaRightUPP = detect_contour(cRW_UPP, ROI_UPP2, frame)
        areaLeftUPP = detect_contour(cLW_UPP, ROI_UPP1, frame)
        areaLeftPARK = detect_contour(cparking, ROI_LEFT_WALL, frame)

        # CENTER FOR BACKTRACK
        areaRedCenter = detect_contour(cListCoreRed, ROI_CENTER, frame)
        areaGreenCenter = detect_contour(cListCoreGreen, ROI_CENTER, frame)

        # Compute largest black contour info for ROI_BLACK
        areaBlackCenter, black_bbox, black_box_h = largest_contour_info(black_mask, ROI_BLACK)

        # Calculate FPS
        current_time = time.time()
        fps = 1 / (current_time - prev_time)
        prev_time = current_time

        # Display FPS on frame
        cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                   
        cv2.imshow('USB CameFd', frame)

        key = cv2.waitKey(1) & 0xFF

        # ----- RED PILLAR DETECTION ------

        # Stores Red Pillar Data: area, cords, and distance
        PillarsRed = filter_pillars(cListPillarRed, frame, closest_pillar_distance, pillar_threshold, ROI_MAIN)

        for pillar in PillarsRed:
    
            # ignore the pillar if it is not the closest (determined by distance)
            if (pillar["area"] < closest_pillar_area * 0.9) and (pillar["distance"] > closest_pillar_distance * 0.9):
                continue
        
            # CASE A: Pillar is too far# pillar["y"] + pillar["h"]) >= 370 and 
            if pillar["x"] < 20:
                previous_pillar = "red"
                print("PILLAR HAS BEEN PASSED")
                pillar_detected  = False
                continue 
        
            # CASE B: pillar is passed
            elif (pillar["y"] + pillar["h"] ) < 155:
                pillar_detected  = False
                print("Pillar too far")
                continue
    
            # CASE C: Update Pillar data
            else:
                closest_pillar_distance = pillar["distance"]
                closest_pillar_color = "red"
                closest_pillar_x = pillar["x"]
                closest_pillar_y = pillar["y"]
                closest_pillar_area = pillar["area"]
                pillar_detected = True

        # ----- GREEN PILLAR DETECTION ------

        PillarsGreen = filter_pillars(cListPillarGreen, frame, closest_pillar_distance, pillar_threshold, ROI_MAIN)

        for pillar in PillarsGreen:

            # ignore the pillar if it is not the closest (determined by distance)
            if (pillar["area"] < closest_pillar_area * 0.9) and (pillar["distance"] > closest_pillar_distance * 0.9):
                continue

            # CASE A: pillar is close and pass the target zone horizontally #pillar["y"] + pillar["h"]) >= 370 and
            if pillar["x"] > greenTarget:
                previous_pillar = "green"
                print("PILLAR HAS BEEN PASSED")
                pillar_detected = False
                continue 
        
            # CASE B: bottom_y_cord of the pillar is too far
            elif (pillar["y"] + pillar["h"] ) < 155: # Soft mark for too far (frame is 100-400 on the y)
                print("Pillar too far")
                pillar_detected  = False
                continue 
    
            # CASE C: Update Pillar data
            else:
                closest_pillar_distance = pillar["distance"]
                closest_pillar_color = "green"
                closest_pillar_x = pillar["x"]
                closest_pillar_y = pillar["y"]
                closest_pillar_area = pillar["area"]
                pillar_detected = True
               
        # ---- TARGET SELECTION ----

        if closest_pillar_color == "red" and closest_pillar_area > Red_grac_const:
            target = redTarget
            cv2.line(frame, (redTarget, y1), (redTarget, y2), (0, 0, 255), 2)
        elif closest_pillar_color == "green" and closest_pillar_area > Green_grav_const:
            target = greenTarget
            cv2.line(frame, (greenTarget, y1), (greenTarget, y2), (0, 255, 0), 2)
        else:
            # Treat as "no pillar" so wall/black logic can run
            target = 0

        if track_Dir is None and line_released and not waiting_for_release:
            if areaLineBlue > line_threshold: 
                track_Dir = "CCW"
                line_released = False  # Reset for this turn
                print(track_Dir)

            elif areaLineOrange > line_threshold:
                track_Dir = "CW"
                line_released = False
                print(track_Dir)
            
        print (areaLeft)
        if track_Dir == "CCW":
            if areaLeft > 1:
                park_DIR = "LEFT"
                pwm.Drive(DC_MOTOR_PWM1, 0.45, "Stop", pwm)
                turn_End = True
                print("Turn Done)")
                track_Dir = None 
                turn_just_ended = True	
                        
                break
            elif areaLineBlue > line_threshold and not stop_Turn:
                print("Detected")
                stop_Turn = True
                pwm.Drive_time(DC_MOTOR_PWM1, 1.0, "CW", 1.5, pwm)
                servo_angle_queue.put(30)
                pwm.Drive(DC_MOTOR_PWM1, 0.6 , "CW", pwm)
            else:
                pass
                
        if track_Dir == "CW":
            if areaRight > 1:
                park_DIR = "RIGHT"
                pwm.Drive(DC_MOTOR_PWM1, 0.45, "Stop", pwm)
                turn_End = True
                print("Turn Done)")
                track_Dir = None 
                turn_just_ended = True	
                        
                break
            elif areaLineOrange > line_threshold and not stop_Turn:
                print("Detected")
                stop_Turn = True
                pwm.Drive_time(DC_MOTOR_PWM1, 1.0, "CW", 1.5, pwm)
                servo_angle_queue.put(-30)
                pwm.Drive(DC_MOTOR_PWM1, 0.6 , "CW", pwm)
            else:
                pass
                
    time.sleep(1.0)	
    servo_angle_queue.put(0)
                    
    if park_DIR == "LEFT":				
        
        pwm.Drive_time(DC_MOTOR_PWM1, 0.6, "CW", 2.0, pwm)

        pwm.Drive(DC_MOTOR_PWM1, 0.45, "Stop", pwm)	
            
        pwm.Drive_time(DC_MOTOR_PWM1, 1.0, "CCW", 0.2, pwm)
        
        pwm.Drive(DC_MOTOR_PWM1, 0.45, "Stop", pwm)	
                
        pwm.Drive_time(DC_MOTOR_PWM1, 0.6, "CCW", 1.0, pwm)
        
        pwm.Drive(DC_MOTOR_PWM1, 0.45, "Stop", pwm)	

        servo_angle_queue.put(-25)
        
        pwm.Drive_time(DC_MOTOR_PWM1, 0.6, "CCW", 1.2, pwm)

        pwm.Drive(DC_MOTOR_PWM1, 0.45, "Stop", pwm)	
        
        servo_angle_queue.put(1)

        pwm.Drive_time(DC_MOTOR_PWM1, 0.6, "CW", 5.35, pwm)

        pwm.Drive(DC_MOTOR_PWM1, 0.45, "Stop", pwm)		

        servo_angle_queue.put(-30)

        pwm.Drive_time(DC_MOTOR_PWM1, 0.6, "CCW", 2.4, pwm)	
        
        pwm.Drive(DC_MOTOR_PWM1, 0.45, "Stop", pwm)

        park_DIR = None		

while True:
    if GPIO.input(BUTTON_PIN) == GPIO.LOW:
        print("START")
        main()
