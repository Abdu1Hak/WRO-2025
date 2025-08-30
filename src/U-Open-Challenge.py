# Import Necessary Libraries and Modules
import cv2
from time import sleep 
import numpy as np
import time 
from picamera2 import Picamera2
import threading
from queue import Queue
from queue import Queue
import math
import smbus
import board
import busio
import adafruit_vl53l0x
import RPi.GPIO as GPIO


# declare the pin out
MODE1              = 0x00
PRESCALE           = 0xFE
LED0_ON_L          = 0x06
LED0_ON_H          = 0x07
LED0_OFF_L         = 0x08
LED0_OFF_H         = 0x09
SERVO_MOTOR_PWM3 = 6
DC_MOTOR_PWM1        = 0
DC_MOTOR_INA1        = 2
DC_MOTOR_INA2        = 1
BUTTON_PIN = 17
# Pin 9, GND 11 on the HAT

# Button Config
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Pull-up resistor enabled


# Declare an I2C communication protocol with the Board and Hat
i2c = busio.I2C(board.SCL, board.SDA)

# Setup the Camera with necessary configurations
picam2 = Picamera2()
picam2.preview_configuration.main.size =(640,480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.controls.FrameRate = 30
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()




# Class for the Motor / Servo Driver
class PCA9685:
    def __init__(self):
        # Opens the i2c bus on the pi
        self.i2c = smbus.SMBus(1)
        # Talks to the PCA9685 at a device address
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

    def setServoPulse(self, channel, pulse): # Given a pulse in microseconds, use it to simulate a digital high or low command
       pulse = pulse*4096/20000       
       self.setPWM(channel, 0, int(pulse))
        
    def Drive(self, channel, percent, wise, pwm):
        pulse = int(15000 * percent) # multiply the percent (between 0.0 to 1.0) with the maximum PWM pulse
        pwm.setServoPulse(channel, pulse)

        # Manipulate the directions
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
        
    def MoveServo(self, channel, angle, delay=0.02): # Converts the angle to a pulse width and moves the servo
        physical_angle = angle - 50 # Add a calibrated offset to ensure that 0 is infact center of the servo, and not -50. This is based on the servo and its mounted position.

        physical_angle = max(-90, min(90, physical_angle)) # Clamp the angle
    
        pulse = int(1500 + (physical_angle / 90.0) * 1000) # convert it to a pulse width 

        self.setServoPulse(channel, pulse) # -> Formward the pulse
        time.sleep(delay)

    def SweepServo(self, channel, end_angle, step=1, delay=0.02): # channel: PCA outpit, end angle: target angle, step:..., delay: how long to wait after each step
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
        
        self.setPWM(channel, 0, 0)
        

# Function to retreive the latest command from the queue, execute it, and update variables
def motor_drive(DC_SPEED):
    current_command = None
    while True:
        command = motor_command_queue.get()
        if command is None:
            break
        
        if command == "drive":
            print("Drive")
            pwm.Drive(DC_MOTOR_PWM1, DC_SPEED, "CW", pwm)
        if command == "backtrack":
            pwm.Drive(DC_MOTOR_PWM1, DC_SPEED, "CCW", pwm)
            print("BACKTRACK...")
        if command == "stop":
            print("Stop")
            pwm.Drive(DC_MOTOR_PWM1, 1.0, "Stop", pwm)
        
        current_command = command
    time.sleep(0.01)

# Function to retreive the latest angle from the queue, execute it, and update variables  
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

# Function to display the ROI Blocks 
def display_roi(img, ROIs, color):
    for ROI in ROIs:
        img = cv2.line(img, (ROI[0], ROI[1]), (ROI[2], ROI[1]), color, 4)
        img = cv2.line(img, (ROI[0], ROI[1]), (ROI[0], ROI[3]), color, 4)
        img = cv2.line(img, (ROI[2], ROI[3]), (ROI[2], ROI[1]), color, 4)
        img = cv2.line(img, (ROI[2], ROI[3]), (ROI[0], ROI[3]), color, 4)

    return img 

# Function to extract a list of contours present within the ROI 
def get_contours(ROI, hsv, ranges): 

    segmented_area = hsv[ROI[1]: ROI[3], ROI[0]:ROI[2]]
    
    # Protect against invalid region
    if segmented_area is None or segmented_area.size == 0:
        return []
        
    mask = cv2.inRange(segmented_area, ranges[0], ranges[1])
    kernel = np.ones((5,5), np.uint8)
    erode = cv2.erode(mask, kernel, iterations=1)
    dilate = cv2.dilate(erode, kernel)
    list_contour = cv2.findContours(dilate, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

    return list_contour

# Return the Max area of contours within the ROI detected contour block
def detect_contour(contour, ROI, frame):

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

# Main function to run the script
def run():

    # Create a class object and set the frequency
    pwm = PCA9685()
    pwm.setPWMFreq(50)

    # Create queues for angle values & motor commands
    motor_command_queue = Queue()
    servo_angle_queue = Queue()

    # VARIABLES 
    MID_SERVO = 0 #
    TURN_DEGREE = 15 # default turning angle
    TURN_DEGREE_LEFT = 17 
    MAX_DEGREE = 25 # Max turning angle -- ignored

    # Angle 
    angle = MID_SERVO
    prevAngle = angle 


    # PD STEERING
    kp = 0.005
    kd = 0.0004
    kp_left = 0.0002
    kd_left = 0.005

    # areas to find proportional and derivative 
    areaDiff = 0
    prevAreaDiff = 0 

    #Turning Logic Variables
    threshold_to_start_turn = 400 # how much the adjacent wall dissapears so that the car knows to turn in that direction
    threshold_to_end_turn = 1300  # how much the upcoming wall comes into view so the car knows to stop turning

    #Left and Right Turn bool control
    left_turn = False 
    right_turn = False 
    line_threshold = 50

    # Counter
    line_detected = False 
    counter =  0 

    # ROIs 
    ROI1 = [0, 230, 150, 320] 
    ROI2 = [490, 230, 640, 320] 
    ROI3 = [200, 300, 440, 350] 

    # FPS calculation setup
    prev_time = time.time()

    
    # Color Masks range defined for black contour (walls) and line
    rBlack = [np.array([0,0,0]), np.array([180, 255, 75])]
    rBlue = [np.array([100, 30, 30]), np.array([140, 255, 255])]
    rOrange = [np.array([10, 100, 100]), np.array([25,255,255])]


    line_released = True # Variable that waits until a new turn is acceptable
    turn_just_ended = False # Variable that becomes true when turn is completed, yet waits to update line_released until both lines are out of sight

    
    motor_command_queue.put("drive")  # roll out
    servo_angle_queue.put(angle)
    
    try:
        while True:
            
            # Capture frame - HSV conversion
            frame = picam2.capture_array()
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) 

            # Add Lines
            frame = display_roi(frame,  [ROI1, ROI2, ROI3], (255, 204, 0))

            # Get Contours
            cListLeft = get_contours(ROI1, hsv, rBlack )
            cListRight = get_contours(ROI2, hsv, rBlack )
            cListLine0 = get_contours(ROI3, hsv, rOrange)
            cListLine1 = get_contours(ROI3, hsv, rBlue )

            # Find Area of Left Contours
            areaLeft = detect_contour(cListLeft, ROI1, frame)
            areaRight = detect_contour(cListRight, ROI2, frame)
            
            # Line Area
            areaLineOrange = detect_contour(cListLine0, ROI3, frame)
            areaLineBlue = detect_contour(cListLine1, ROI3, frame)
             


            # ===== TURN DETECTION PHASE =====
            
            if line_released:
                # Check for a new turn if we are not already in a turn
                if not (left_turn or right_turn):
                    if areaLineBlue > line_threshold:
                        left_turn = True
                        line_released = False
                        print("LEFT TURN HAS STARTED")
                    if areaLineOrange > line_threshold:
                        right_turn = True 
                        line_released = False 
                        print("RIGHT TURN HAS STARTED")
            

            # ===== CALCULATE VALUES FOR STEERING =====

            areaDiff = areaLeft - areaRight # Cross track error 
            derivative_term = areaDiff - prevAreaDiff # Cross track error rate (to control)
            


            # ===== FIND STEERING ANGLE =====

            if left_turn:
                # Use left-specific PD vals
                angle = int(MID_SERVO + (kp_left * areaDiff) + (kd_left * derivative_term))
                angle = min(TURN_DEGREE_LEFT, max(angle, -TURN_DEGREE_LEFT))
                print("LEFT ANGLE", angle)
                    
            elif right_turn:
                # standard right procedure
                angle = int(MID_SERVO + (kp * areaDiff) + (kd * derivative_term))
                angle = max(-TURN_DEGREE, min(angle, TURN_DEGREE))
                print("RIGHT TURN ANGLE: ", angle)

            else:
                # Wall following mode + offset for left
                angle = int(MID_SERVO + (kp * areaDiff) + (kd * derivative_term))
                angle = max(-TURN_DEGREE, min(angle, TURN_DEGREE))
                if angle < 0:
                    angle -= 2 
                print("ANGLE", angle)


            
            # ===== TURN COMPLETED CHECK =====

            if left_turn or right_turn:
                turn_should_end = False
                # Conditions to exit a turn: respective wall reappears and subsequent line is spotted
                if left_turn and areaLeft > threshold_to_end_turn and areaLineOrange > line_threshold:
                    turn_should_end = True
                elif right_turn and areaRight > threshold_to_end_turn and areaLineBlue > line_threshold:
                    turn_should_end = True
                
                if turn_should_end:
                    # Complete the turn
                    left_turn = False
                    right_turn = False 
                    turn_just_ended = True
                    prevDiff = 0
                    counter += 1
                    print("TURN COMPLETED - COUNTER: ", counter)
            

            # ===== SERVO COMMANDS =====

            if angle != prevAngle:
                servo_angle_queue.put(angle)
                prevAngle = angle 
            

            # ===== LINE CLEARED? =====
            if turn_just_ended:
                # Wait until both lines are out of frame to prevent another turn on the same line
                if areaLineBlue < line_threshold and areaLineOrange < line_threshold:
                    line_released = True
                    turn_just_ended = False 
                    print("LINE CLEARED!! ")

            
            # Update Values 
            prevAreaDiff = areaDiff


            # ===== STOP CONDIITON CHECK =====
            if counter == 12 and abs(angle) <= 3:
                sleep(1)
                motor_command_queue.put("stop")
                servo_angle_queue.put(MID_SERVO)
            
            # Calculate FPS
            current_time = time.time()
            fps = 1 / (current_time - prev_time)
            prev_time = current_time

            # Display FPS on frame
            cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    
            cv2.imshow('USB Camera Feed', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                motor_command_queue.put("stop")
                picam2.stop()
                cv2.destroyAllWindows()
                pwm.DisableServo(SERVO_MOTOR_PWM3)

    finally:
        motor_command_queue.put("stop")
        picam2.stop()
        cv2.destroyAllWindows()
        pwm.DisableServo(SERVO_MOTOR_PWM3)

# Function to incorporate threaded functions
def main():

    DC_SPEED = 0.6
    t1 = threading.Thread(target=motor_drive(DC_SPEED), name='t1')
    t2 = threading.Thread(target=servo_move, name='t2')
    t1.start()
    t2.start()
	
    run()
    
    t1.join()
    t2.join()
	

# RUN THE SCRIPT
if GPIO.input(BUTTON_PIN) == GPIO.LOW:
	main()
    
