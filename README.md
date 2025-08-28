# WRO-2025

The README file will be the central home for all written documentation. It will elaborate on all sections

Things to consider as part of the README file:

1. Pictures - introductions
2. Identify the Objective of the Game
3. Our approach - general to focused, discussions, challenges, goal - issues we faced
4. Mobility Management - motors, chassi design, mounting of all components. speed, torque, power, etc - 3D - issues we faced
5. Power and Sense Management- camera, servos, battery, hat, their usage and what they do - issues we faced
6. Obstacle Management - how they interact with components, strategy of open and obstacle, parking,  program flow, conditions, logic diagrams, code blocks, and commmented code - issues we faced during the coding
7. Assembly Instructions
8. Improvements


## 1. Introduction

Team Members & Contacts
Repo Structure
Links to Jump to sections

## 2. Objective of the Game

What is this year's challenge - obstacle and open


## 3. What is our goal?

Considering our skill set - what areas did we prioritize, beginning challenges, and our goal to work towards

## 4. Mobility Management
## 5. Power and Sense Management
## 6. Software - Open Challenge - Obstacle Challenge - Parking

Identify the Logic flow, including code snippets, and provide pictures of the VNC

### Using VNC - Connecting to the PI

   Our primary method of interfacing with the Raspberry Pi to monitor the camera feed was through RealVNC, establishing a wireless connection between the Pi and our system on the same network. By accessing the Pi’s IP address, we were able to mirror its desktop environment directly on our screen. While this approach initially introduced little overhead, performance bottlenecks quickly emerged, primarily in the form of latency and memory limitations.

   When running our open challenge code, we observed frame rates dropping to between 0–5 FPS. The root cause was the Pi’s limited 2 GB of RAM, which could not simultaneously handle intensive video processing and real-time motor control. To resolve this, we restructured execution by assigning tasks to separate CPU cores using threading: one core dedicated to motor commands, another to servo operations, and a third to managing the camera feed. This distribution of workload significantly reduced contention and stabilized performance, resulting in a consistent throughput of approximately 30 FPS. 

```python
import threading
from queue import Queue
...
motor_command_queue = Queue()
servo_angle_queue = Queue()
...
t1 = threading.Thread(target=motor_drive, name='t1')
t2 = threading.Thread(target=servo_move, name='t2')
```


### Open Challenge

#### Wall Detection
The car begins by initializing the PiCamera2, giving us a continuous video feed which will later be processed for color detection and contour tracking. Inside the main loop, we capture the frame and convert the RGB image to HSV (a color model that separates hue from brightness), which is required for accurate color masking. 

```python
frame = picam2.capture_array()
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) 
```
Instead of processing the entire screen, we focus on processing specific areas of the frame that encapsulate the left wall, right wall, and the floor/line infront of the robot. These areas save computation and drastically improve detection speed. These boxes are visualized and returned using the function ```display_roi()``` on the video feed for live confirmation. Thereafter, we begin color masking using thresholds that are defined by a lower and upper HSV range:

```python
rBlack = [np.array([0,0,0]), np.array([180, 255, 75])] # for walls
rBlue = [np.array([100, 30, 30]), np.array([140, 255, 255])] # for blue line
rOrange = [np.array([10, 100, 100]), np.array([25,255,255])] # for orange line
```

If a pixel's HSV value falls within this range, it is part of the target object. Thereafter, we extract only the ROI area from the HSV frame, and use ``` cv2.inRange() ``` to generate a binary mask where pixels within the range have a value of 255 (white) and those out of range are set to 0 (black). This will isolate the color we want in that ROI. This operation can often be noisy with small, unwanted white pixels. To fix this, we performed further morphological operations that will erode (remove tiny unwanted pixels) and dilate (expand remaining white areas to restore the object size), helping us make contours smoother and more reliable. 

```python
segmented_area = hsv[ROI[1]: ROI[3], ROI[0]:ROI[2]]
mask = cv2.inRange(segmented_area, ranges[0], ranges[1])
kernel = np.ones((5,5), np.uint8)
erode = cv2.erode(mask, kernel, iterations=1)
dilate = cv2.dilate(erode, kernel)
```

Now that we can extract the available contours, we use ``` cv2.findContours() ``` to find continuous shapes in the mask, only considering the outermost contours and compressing unnecessary points to simplify contour shapes. The result is a list of all contours that represent detected objects in that ROI. To show this detection in real time, we create a rectangular box around the contour and take the largest contour area for steering calculations. The result of all these operations is that we can successfully detect the two parallel walls and the line infront, calculate how much of the contours are visible in that area, and eventually use these values to perform corrective steering.

In the end, we segmented out these operations into distinct functions:

```python
# Segment the ROI area and return a list of contours
def get_contours(ROI, hsv_frame, color_ranges)
```

```python
# Iterate through the list of contours, create a rectangle around the detected regions in the ROI, and calculate its area 
def detect_contours(contour, ROI, frame)
```

#### PD Steering
   Our main strategy for autonomous travel was to use proportional derivative control with a wide-angle camera, a FOV of 170 degrees. In the open challenge, the only components visible to the camera were the two parallel inner and outer walls, as well as the colored lines indicating a turn. The code begins by measuring how many contours are present in the ROI (Region of Interest) for both walls, and by calculating the area of these contours, it becomes a means of determining how much of the walls are visible to the camera. By calculating the difference between the left and right walls based on the contours on either side, which is again based on the horizontal position of the car, we can determine how far we are from the desired trajectory. This area difference, also known as Cross Track Error (Ep), would be multiplied by a scaling factor called the proportional gain, which would result in a steering angle. Overall, the gain that would satisfy the requirements of our vehicle would be 0.005, and this value is subject to change. The main reason why our value is so small is that the servo's range of movement is between -25 degrees (far left) and 25 degrees (far right), and our car is relatively slow. 

```python
# Proportional and derivative gains
kp = 0.005
kd = 0.0004
```

   With proportional control, if the car only sees a single wall, it can enter the center line (center of both walls) crookedly. The result of this is that the controller will repeatedly overshoot the actual desired trajectory and not actually follow it. To correct this overshooting problem, we consider additional error measurements to update our steering command. The Derivative term  allows us to understand how fast we are moving in a perpendicular direction with respect to the desired trajectory. If the cross-track error is a low figure, or 0, it means we are perfectly following the desired trajectory. So by calculating the difference between our current area difference to the previous area difference, we get our derivative term, which likewise is subject to a derivative gain, allowing us to eventually close that overshooting gap in the instance that the car follows a zig-zag pattern. To prevent cases where our car would underdamp or overdamp, our team would continuously test different numerical figures and would eventually settle on 0.0004. This would allow our slow car to perfectly navigate the rounds smoothly without overshooting or underdamping because of this robust algorithm to determine the best angle

```python
# find the area difference (cross-track error) and cross-track error rate 
areaDiff = areaLeft - areaRight
derivative_term = areaDiff - prevAreaDiff 
MID_SERVO = 0 # default angle of car and center 

# PD Steering Algorithm            
angle = int(MID_SERVO + (kp * areaDiff) + (kd * derivative_term))

# Calculated angle is placed in the queue
servo_angle_queue.put(angle)
```

#### Turning
Similarly to the walls, we also detect the number of contours present in the orange and blue lines ahead of us. When the number of contours in the line exceeds an arbitrary threshold value, it is a condition to enter its respective turn direction. Once in a turn, we constantly examine if the wall that we turn against is above a threshold value and if the subsequent line has been detected. If these two conditions are met, we exit the turn and append our counter. The counter is responsible for calculating the number of turns.

```python
if left_turn or right_turn:
   if ((areaRight > threshold_to_end_turn and right_turn) and areaLineBlue > line_threshold) or (areaLeft > threshold_to_end_turn and left_turn and areaLineOrange > line_threshold):
   left_turn = False
   right_turn = False
   counter += 1 
```

In a right turn, we will continue using the angle value calculated based on the area difference. Ultimately, when a right turn appears, the amount of contour in the right wall will decrease until the value eventually reaches 0. This, in turn, generates an angle that resembles a sharp right turn. To prevent this angle from exceeding the steering range, we clamp the values to its turn degree. 


```python
elif right_turn:
   angle = max(-TURN_DEGREE, min(angle, TURN_DEGREE))
```

In a left turn, we noticed that the left steering was not as powerful as the right. We assumed that this was a result of a mechanical issue present within our chassis. Hence, when the robot travels clockwise, we introduced different Kp and kd values that would calculate an angle within the left turn condition.

```python
elif left_turn:
   angle = int(MID_SERVO + (kp_left * areaDiff) + (kd_left * derivative_term))
   angle = min(TURN_DEGREE, max(angle, -TURN_DEGREE))
```
 Once a turn is ended and both colored lines are out of sight, a variable called ``` line_released = True ``` is updated to mark the end of a turn and await the next turn. If the car is not in a left or right turn, the angle values would still be clamped, and to combat our left steering weakness, the negative angles would be offset by a small amount of -2 to make the car even smoother.

Once the counter has reached 12 (3 Full rounds) and the angle value is near zero (the car is straight), we wait for two seconds before turning the motor off. 

```python
# Stop Car Logic 
if counter == 12 and (abs(angle) - MID_SERVO) <= 3:
   # STOP CAR
   sleep(2) 
   motor_command_queue.put("stop")
```

### Obstacle:
Where do we pick back up from the open challenge code?
How do we avoid obstacles? and handle different cases
How do we turn the car with obstacles on the field?

### Parking:
How do we leave the parking, and how do we understand direction?
How do we enter the parking spot after 3 rounds?
How do we manage color detection between red pillars and magenta parking lot walls.



## 7. Assembly 
## 8. Improvements
   
