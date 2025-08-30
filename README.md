# WRO-2025 - Engineering Documentation ⚙️⚙️

### Team Members 

[Insert Professional Photo Here]

- Abdul Farooqi, 17, abdulfarooqi@gmail.com
- Aarav Daudia, 16, f18astro@gmail.com

### Repository Contents

File Name | Description
--- | ---
[models](https://github.com/Abdu1Hak/WRO-2025/tree/main/Models) | Folder for models used by 3D printers to produce the mechanical elements on the car
[t-photos](https://github.com/Abdu1Hak/WRO-2025/tree/main/t-photos) | Contains 2 Photos of all team members
[v-photos](https://github.com/Abdu1Hak/WRO-2025/tree/main/v-photos) | Contains 6 Photos of the vehicle
[video](https://github.com/Abdu1Hak/WRO-2025/tree/main/video) | Contains 2 Video links to the Car performing the Open and Obstacle Challenge
[schemes](https://github.com/Abdu1Hak/WRO-2025/tree/main/Schemes) | Contains several electrical schematic diagrams of the vehicle
[src](https://github.com/Abdu1Hak/WRO-2025/tree/main/srcs) | Contains the main source code for the open and obstacle challenge + more
[other](https://github.com/Abdu1Hak/WRO-2025/tree/main/Other) | Other essential files


### README Contents

- [Intro](#1-introduction)
- [Game Objective](#2-objective-of-the-game)
- [Game Strategy](#3-what-is-our-goal)
- [Mobility Management](#4-mobility-management)
- [Power and Sense Management](#5-power-and-sense-management)
- [Software](#6-software---open-challenge---obstacle-challenge---parking)
  - [Setup and Initialization](#using-vnc---connecting-to-the-pi)
  - [Begin Open Challenge](#open-challenge)
    - [Wall Detection](#wall-detection)
    - [PD Steering](#pd-steering)
    - [Turning](#turning)
  - [Obstacle Challenge](#obstacle-challenge)
    - [Pillar Detection and Maneuvering](#pillar-detection-and-maneuvering)
    - [Backtracking](#backtracking)
    - [Toggling Between Modes](#toggling-between-pillar-following-and-wall-following)
    - [Turning (Obstacle)](#turning-with-obstacles-in-the-way---wait)
  - [Parking](#parking)
 - [Flow Diagram Logic](#flow-diagram---obstacle-challenge)
- [Assembly](#7-assembly)
- [Improvements](#8-improvements)



Things to do:
1. Fix Open Challenge Code - Organize it, include comments, and restructure code logic
2. Add Vehicle Pictures and Team pictures
3. Video of Open Challenge - Annotated
   
5. Add Team pics to Read Me
6. Add Introductions and Objectives
7. Include Hardware Components in a table format or organized format
8. Finish Mobility Management - includes pictures, chassis design, motors, servo, improvements, final product -- AARAV
9. Power and Sense Management - ElectroMechanical Diagram - Power and wiring, sensors, hat, expansion board, etc.

10. Fix Obstacle Challenge Code - Organize it, .... # Sep. 1
11. Video it
12. Finish Documentation of Parking and Turning
13. SUBMIT - 6 Days Left!



Things to consider as part of the README file:

1. Pictures - introductions
2. Identify the Objective of the Game
3. Our approach - general to focused, discussions, challenges, goal - issues we faced
4. Mobility Management - motors, chassi design, mounting of all components. speed, torque, power, etc - 3D - issues we faced
5. Power and Sense Management- camera, servos, battery, hat, their usage and what they do - issues we faced
6. Obstacle Management - how they interact with components, strategy of open and obstacle, parking,  program flow, conditions, logic diagrams, code blocks, and commented code - issues we faced during the coding
7. Assembly Instructions
8. Improvements


## 1. Introduction

Team Members & Contacts
Repo Structure
Links to Jump to sections

## 2. Objective of the Game

What is this year's challenge - obstacle and open


## 3. What is our goal

Considering our skill set - what areas did we prioritize, beginning challenges, and our goal to work towards

## 4. Mobility Management

Mobility management discussion should cover how the vehicle movements are
managed. What motors are selected, how they are selected and implemented.
A brief discussion regarding the vehicle chassis design /selection can be
provided as well as the mounting of all components to the vehicle
chassis/structure. The discussion may include engineering principles such as
speed, torque, power etc. usage. Building or assembly instructions can be
provided together with 3D CAD files to 3D print parts. Be as detailed as possible.
Information on improvements is also provided.

## 5. Power and Sense Management

Power and Sense management discussion should cover the power source for
the vehicle as well as the sensors required to provide the vehicle with
information to negotiate the different challenges. The discussion can include
the reasons for selecting various sensors and how they are being used on the
vehicle together with power consumption. The discussion could include a wiring
diagram with BOM for the vehicle that includes all aspects of professional
wiring diagrams. Be as detailed as possible - provide improvements. 

## 6. Software - Open Challenge - Obstacle Challenge - Parking

### Using VNC - Connecting to the PI

   Our primary method of interfacing with the Raspberry Pi to monitor the camera feed was through RealVNC, establishing a wireless connection between the Pi and our system on the same network. By accessing the Pi’s IP address, we were able to mirror its desktop environment directly on our screen. While this approach initially introduced little overhead, performance bottlenecks quickly emerged, primarily in the form of latency and memory limitations.

   When running our open challenge code, we observed frame rates dropping to between 0 and 5 FPS. The root cause was the Pi’s limited 2 GB of RAM, which could not simultaneously handle intensive video processing and real-time motor control. To resolve this, we restructured execution by assigning tasks to separate CPU cores using threading: one core dedicated to motor commands, another to servo operations, and a third to managing the camera feed. This distribution of workload significantly reduced contention and stabilized performance, resulting in a consistent throughput of approximately 30 FPS. 

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


## Open Challenge

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

INSERT PICTURE HERE

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

## Obstacle Challenge

#### Pillar Detection and Maneuvering 

When a frame is captured and converted to HSV, the first pillar-related step is to extract color-specific contours from the forward-looking ROI. 
```python
cListPillarGreen = get_contours(ROI_MAIN, hsv, rGreen)
cListPillarRed, dilate = get_red_contours(ROI_MAIN, hsv, rRed, rRed2)
```

These raw contours are filtered into potential candidates with the function ``` filter_pillars() ``` that enforces a minimum area and computes geometric values for each pillar: center x, top y, height, and distance. This distance prefers pillars that are lower in the image and near the camera center.  Then these pillars must pass short logical filters. We ignore pillars if it is not the closest pillar (to avoid multiple pillars being detected). A pillar is marked passed if it has moved past a horizontal target zone ```python if pillar["x"] < redTarget ``` or mark it 'too far' if the bottom ```python pillar["y"] + pillar["h"]``` is above a soft distance threshold (< 155) which indicates the pillar is still far vertically. Only when these conditions are met will the dictionary below be appended as the 'closest pillar'.

```python
{
  "x": x + w // 2,
  "y": y,
  "h": h,
  "w": w,
  "area": area,
  "distance": pillar_distance
}
```
Depending on the color of the pillar candidate, the code attempts to align a target x-column to the detected pillar. This selection ensures area gating so the robot only targets clearly seen pillars. Green pillars are aligned to the far-right target so the car can pass from the left, whereas red pillars are aligned to the far-left target so the car can pass from the right of it. 

```python
if closest_pillar_color == "red" and closest_pillar_area > Red_grac_const:
    target = redTarget      # e.g. 50 (left side)
elif closest_pillar_color == "green" and closest_pillar_area > Green_grav_const:
    target = greenTarget    # e.g. 620 (right side)
else:
    target = 0              # no pillar target, fall back to wall logic
```

The actual steering is likewise PD-based and computed from the horizontal difference between the pillar center and the selected target value. The code builds an error and a derivative term as: 

```python
pillar_error = abs(target - closest_pillar_x)
derivative_term = pillar_error - prev_pillar_error
angle = int(default_servo + (kp * pillar_error) + (kd * derivative_term))
angle = max(-TURN_DEGREE, min(angle, TURN_DEGREE))
```

The sign/direction is restored when issuing a servo command by inverting the angle sign for green pillars. So the logic flow is: 


 There are safety and context checks around pillar following. The code executes the pillar following only if a candidate pillar is selected and not too close to the wall. The areaLeft and areaRight checks prevent the robot from weaving away from the pillar when the adjacent wall is already too close. Pillar following is also not executed if we are currently in a left or right turn. 

```python
if pillar_detected == True and not (closest_pillar_color == "green" and areaLeft >= 11000) and not (closest_pillar_color == "red" and areaRight >= 11000) and not (turn_Dir == "right" or turn_Dir == "left'):
    # pillar following & possible backtrack

```

#### Backtracking

If the robot is too close to a pillar where further turning could result in a collision, the code triggers a non-blocking backtrack to create enough room between the pillar and the car to successfully maneuver around it.  ```backtrack_active``` and ```backtrack_end_time``` manage a timed reverse: the loop continues sensing while the robot reverses, and when the timer expires, another block sets the drive to active (```motor_command_queue.put("drive")```). This is a practical, time-limited evasive action to avoid late collisions. 

```python
if (closest_pillar_color == "green" and areaGreenCenter > 4000) or (closest_pillar_color == "red" and areaRedCenter > 4000):
    if not backtrack_active:
        backtrack_active = True
        backtrack_end_time = time.time() + backtrack_duration
        servo_angle_queue.put(0)              # slight straighten
        motor_command_queue.put("backtrack")  # reverse now
        servo_angle_queue.put(0)

```

#### Toggling between Pillar Following and Wall Following

If no pillar is actionable (either pillar_detected is false or the safety conditions fail), the code falls back into wall following PD to center the car between the walls once again. This is specifically needed once a maneuver has been completed. In logic, Pillar following overrides wall PD when valid; otherwise, fall back to PD control steering.

```python
wall_error = areaLeft - areaRight
wall_derivative_term = wall_error - prev_wall_error
angle = int(default_servo + (wall_error * kp_walls) + (wall_derivative_term * kd_walls))
angle = max(-TURN_DEGREE, min(angle, TURN_DEGREE))
servo_angle_queue.put(angle)
```

#### Turning with Obstacles in the way - WAIT

When ``` areaLineBlue ``` or ``` areaLineOrange ``` exceeds a threshold, the robot recognizes a visible turn marker. The loop will decide on a turn when there is currently no active track direction and the system is ready to accept a new line, ``` line_released = True ```


### Parking
How do we leave the parking, and how do we understand direction?
How do we enter the parking spot after 3 rounds?
How do we manage color detection between red pillars and magenta parking lot walls?

### Flow Diagram - Obstacle Challenge

<img width="791" height="652" alt="image" src="https://github.com/user-attachments/assets/2f230ded-b8f1-44ba-b78e-60dafe4bf03d" />


## 7. Assembly 
## 8. Improvements
   
