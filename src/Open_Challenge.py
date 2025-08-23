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

i2c = busio.I2C(board.SCL, board.SDA)

picam2 = Picamera2()
picam2.preview_configuration.main.size =(640,480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.controls.FrameRate = 30
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()



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
        #time.sleep(tim)
        
        
    def MoveServo(self, channel, angle, delay=0.02):
        physical_angle = angle - 50

        physical_angle = max(-90, min(90, physical_angle))
    
        pulse = int(1500 + (physical_angle / 90.0) * 1000)

        self.setServoPulse(channel, pulse)
        time.sleep(delay)

    def SweepServo(self, channel, end_angle, step=1, delay=0.02):
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
        
        if command == "drive":
            print("Drive")
            pwm.Drive(DC_MOTOR_PWM1, 0.6, "CW", pwm)
        if command == "backtrack":
            pwm.Drive(DC_MOTOR_PWM1, 1.0, "CCW", pwm)
            print("BACKTRACK...")
        if command == "stop":
            print("Stop")
            pwm.Drive(DC_MOTOR_PWM1, 1.0, "Stop", pwm)
        
        current_command = command
    time.sleep(0.01)
    
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

def display_roi(img, ROIs, color):
    for ROI in ROIs:
        img = cv2.line(img, (ROI[0], ROI[1]), (ROI[2], ROI[1]), color, 4)
        img = cv2.line(img, (ROI[0], ROI[1]), (ROI[0], ROI[3]), color, 4)
        img = cv2.line(img, (ROI[2], ROI[3]), (ROI[2], ROI[1]), color, 4)
        img = cv2.line(img, (ROI[2], ROI[3]), (ROI[0], ROI[3]), color, 4)

    return img 

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

def run():
    # VARIABLES 
    MID_SERVO = 0 # default angle of car and center 
    TURN_DEGREE = 15 # default turning angle
    MAX_DEGREE = 25 # Max turning angle

    # Angle 
    angle = MID_SERVO
    prevAngle = angle 
    # Motor
    DC_SPEED = 1

    # PD STEERING - subject to change via height
    kp = 0.005
    kd = 0.0004
    
    kp_left = 0.0002
    kd_left = 0.005

    # areas to find proportional and derivative 
    areaDiff = 0
    prevAreaDiff = 0 

    # Max amount we can turn either direction -> Center is 0 - Left is -90  - Right is +90
    furthestLeft = MID_SERVO - MAX_DEGREE
    furthestRight = MID_SERVO + MAX_DEGREE

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

    # IDK YET
    start = False 
    turnDir = "none"
    
    # Color Masks range defined for black contour (walls) and line
    rBlack = [np.array([0,0,0]), np.array([180, 255, 75])]
    rBlue = [np.array([100, 30, 30]), np.array([140, 255, 255])]
    rOrange = [np.array([10, 100, 100]), np.array([25,255,255])]

    turn_cooldown = 3.0  # seconds
    last_turn_end_time = 0
    line_released = True
    turn_just_ended = False
    waiting_for_release = False
    
    motor_command_queue.put("drive")  # roll out
    servo_angle_queue.put(0)
    # Capture a frame
    try:
        while True:
        
            frame = picam2.capture_array()
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) 

            # Add Lines
            frame = display_roi(frame,  [ROI1, ROI2, ROI3], (255, 204, 0))

            # Get Contours
            cListLeft = get_contours(ROI1, hsv, rBlack )
            cListRight = get_contours(ROI2, hsv, rBlack )
            cListLine0 = get_contours(ROI3, hsv, rOrange)
            cListLine1 = get_contours(ROI3, hsv, rBlue )

            # Find Area of Left Contours --> Make this into a function
            areaLeft = detect_contour(cListLeft, ROI1, frame)
            areaRight = detect_contour(cListRight, ROI2, frame)
            
            # Line
            areaLineOrange = detect_contour(cListLine0, ROI3, frame)
            areaLineBlue = detect_contour(cListLine1, ROI3, frame)
             
            if line_released and not waiting_for_release and not left_turn and not right_turn:
                if areaLineBlue > line_threshold:
                    left_turn = True
                    line_released = False
                    print("LEFT TURN STARTED")

                elif areaLineOrange > line_threshold:
                    right_turn = True
                    line_released = False
                    print("RIGHT TURN STARTED")
                    
            # find the area difference (Cross track error)
            areaDiff = areaLeft - areaRight
            derivative_term = areaDiff - prevAreaDiff # Cross track error rate (to control)
            
            angle = int(MID_SERVO + (kp * areaDiff) + (kd * derivative_term))

            if angle != prevAngle:
                
                if left_turn or right_turn:
                    if ((areaRight > threshold_to_end_turn and right_turn) and areaLineBlue > line_threshold) or (areaLeft > threshold_to_end_turn and left_turn and areaLineOrange > line_threshold):
                      #set turn variables to false as turn is over
                        left_turn = False 
                        right_turn = False
                        turnDir = "none"
                        turn_just_ended = True
                        #reset prevDiff
                        prevDiff = 0  
                        counter += 1 
                        print("COUNTER: ", counter)

                    elif left_turn: 
                        angle = int(MID_SERVO + (kp_left * areaDiff) + (kd_left * derivative_term))
                        angle = min(17, max(angle, -17))
                        print("LEFT ANGLE", angle)
                        
                    elif right_turn:
                        angle = max(-TURN_DEGREE, min(angle, TURN_DEGREE))

                    servo_angle_queue.put(angle)
                    
                else:
                    angle = max(-TURN_DEGREE, min(angle, TURN_DEGREE))
                    if angle < 0:
                        angle -= 2
                    print("ANGLE: ", angle)
                    servo_angle_queue.put(angle)
            
            if turn_just_ended:
                if areaLineBlue < line_threshold and areaLineOrange < line_threshold:
                    line_released = True       # <--- allow new turn
                    turn_just_ended = False    # reset
                    print("LINE CLEARED, READY FOR NEXT TURN")

            prevAngle = angle
            prevDiff = areaDiff

            # Stop Car Logic 
            if counter == 12 and (abs(angle) - MID_SERVO) <= 3:
                # STOP CAR
                sleep(2) 
                motor_command_queue.put("stop")
            
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

    finally:
        motor_command_queue.put("stop")
        picam2.stop()
        cv2.destroyAllWindows()
        pwm.DisableServo(SERVO_MOTOR_PWM3)

def main():
	t1 = threading.Thread(target=motor_drive, name='t1')
	t2 = threading.Thread(target=servo_move, name='t2')
	t1.start()
	t2.start()
	
	run()
	
	motor_command_queue.put("stop")
    
	
	t1.join()
	t2.join()
	

if __name__ == '__main__':
	
	main()
    
self.setPWM(SERVO_MOTOR_PWM3, 0, 0)






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
