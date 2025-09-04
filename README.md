# WRO-2025 - Engineering Documentation ⚙️⚙️⚙️

### Team Members 

<img width="713" height="671" alt="image" src="https://github.com/user-attachments/assets/b72c08db-f416-4baa-a63d-0307d5091035" />


> Team Members
- Abdul Farooqi, 17, abdulfarooqi@gmail.com
- Aarav Daudia, 16, f18astro@gmail.com

> Coach
- Vikramjeet Singh, singhvikramjeet557@gmail.com

### Repository Contents

File Name | Description
--- | ---
[Build](https://github.com/Abdu1Hak/WRO-2025/tree/cc1186081c65a94e3f9d5a899eeedb2a14111d81/Build) | Explanations and descriptions for mechanical/robot build
[Models](https://github.com/Abdu1Hak/WRO-2025/tree/main/Models) | Folder for models used by 3D printers to produce the mechanical elements on the car
[t-photos](https://github.com/Abdu1Hak/WRO-2025/tree/main/t-photos) | Contains 2 Photos of all team members
[v-photos](https://github.com/Abdu1Hak/WRO-2025/tree/main/v-photos) | Contains 6 Photos of the vehicle
[video](https://github.com/Abdu1Hak/WRO-2025/tree/main/video) | Contains 2 Video links to the Car performing the Open and Obstacle Challenge
[Schemes](https://github.com/Abdu1Hak/WRO-2025/tree/main/Schemes) | Contains several electrical schematic diagrams of the vehicle
[src](https://github.com/Abdu1Hak/WRO-2025/tree/main/srcs) | Contains the main source code for the open and obstacle challenge + more
[Other](https://github.com/Abdu1Hak/WRO-2025/tree/main/Other) | Other essential files


### README Contents

- [Game Objective](#1-objective-of-the-game)
- [Game Strategy](#2-our-strategy)
- [Mobility Management](#3-mobility-management)
- [Power and Sense Management](#4-power-and-sense-management)
- [Software](#5-software---open-challenge---obstacle-challenge---parking)
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
- [Assembly](#6-assembly)



3. Video of Open Challenge - Annotated
   
9. Finish Mobility Management - includes pictures, chassis design, motors, servo, improvements, final product -- AARAV
10. More pictures in software documentation
11. Fix Obstacle Challenge Code - Organize it
12. Video it
13. Finish Documentation of Parking and Turning
14. SUBMIT - 6 Days Left!


## 1. Objective of the Game

### The Challenge

The 2025 WRO Future Engineers challenge is a Time Attack race. The car must complete its laps as fast as possible while following strict driving rules. The challenge focuses on building a fully autonomous self-driving car capable of navigating randomized environments, obeying traffic rules, and performing parallel parking correctly at the finish.

The vehicle must complete three full laps around the track in each attempt. During the run, the car cannot collide with or move the traffic signs (pillars) placed on the field.

Goal: Design and program a self-driving car that completes both the Open Challenge and the Obstacle Challenge.

### Open Challenge

- Complete three laps around the track without touching the walls.
- Each side of the track is randomized to be either 100 cm or 60 cm.
- The driving direction (clockwise or counterclockwise) is also randomized at the start of the run.

### Obstacle Challenge

- Complete three laps around the track while avoiding traffic signs (pillars).
  - Red Pillar -> Right Side | Green Pillar - Left Side
- The driving direction (clockwise or counterclockwise) is randomized.
- After 3 laps are completed, the car must parallel park into the parking zone


## 2. Our Strategy

Throughout the development of our robot, we identified specific areas to prioritize based on our team’s skills and the requirements of the challenge. Our team consisted of two members: Aarav, who specialized in CAD design, mechanical iteration, and chassis optimization, and Abdul, who focused on game logic, coding, and system integration. Together, this allowed us to balance robust mechanical design with awesome software implementation.

For the software framework, we selected Python due to its accessibility, extensive library support, and compatibility with computer vision tools. Our design philosophy was simplicity and effectiveness: how could we best utilize a peripheral to interpret the playing field and execute tasks autonomously? While the drivetrain relied on a DC motor for propulsion and a servo for steering, the central sensing device we used was the camera. Unlike single-purpose sensors, the camera offered environmental data, allowing for object detection, color recognition, and spatial awareness. We intentionally prioritized vision-based sensing over other alternatives, confident that it would provide the most direct path to consistent performance.

The computational platform was the Raspberry Pi, chosen over microcontrollers such as Arduino or ESP boards. This decision was driven by the Pi’s higher processing capabilities, making it better suited for handling the intensive operations required by real-time image processing. Since our system heavily depended on computer vision, the Pi provided the necessary balance of performance and scalability.

To structure development, we adopted a staged approach: **research** → **test** → **refine** → **iterate**. Numerous design ideas were discussed, but we initially focused on building a minimal, working prototype before exploring optimizations. The combination of the Raspberry Pi and OpenCV meant rapid prototyping, allowing us to quickly get set up with color detection, contour tracking, and geometric calculations for navigation. This iterative process created opportunities for improvements and informed our overall strategy.

This documentation serves as a consolidated record of our engineering process, highlighting technical decisions, challenges encountered, and opportunities for improvement.


## 3. Mobility Management

PLEASE DO THIS ----------------------------------------------

## 4. Power and Sense Management

The Robot is composed of a minimal amount of sensors, motors, and a main processing unit, the Raspberry Pi 4 Model B. 

### Pinout of the HAT
<img width="973" height="618" alt="image" src="https://github.com/user-attachments/assets/1ba73d24-60bf-4041-91fa-448d421dbbee" />

### Custom-Made Circuit Scheme of the Robot
<img width="1060" height="480" alt="image" src="https://github.com/user-attachments/assets/865e0d0e-f861-4994-a4d1-5dc5745edfb6" />

### Power Heirarchy

* Battery ```12V - 3000mA```
  * Pi HAT Regulator
    * Raspberry Pi 4 Model B ```6V 3000mA```
      * RPi Camera ```5V 200mA```
    * Motor ```6-12V - 650-2200mA```
    * Servo ```6V - 1000mA```
    * Button ```negligable```

### Connections
* Laptop
  * ```VNC``` using RealVNC into Pi
    * Raspberry Pi
      * HAT
      * Motor
      * Servo
      * Camera
      * Button

More information on the Hardware Components can be found in [Build](https://github.com/Abdu1Hak/WRO-2025/tree/cc1186081c65a94e3f9d5a899eeedb2a14111d81/Build) 


### Management

Our vehicle is powered by a 12V lithium-ion power bank (TalentCell YB1203000-USB), chosen for its fair weight ```190 grams```, current output of up to ```3A``` and operating temperature range (-20C to 60C). This battery was ideal because it delivered adequate current to run the Pi, motor, servo, and others without overheating. The 12V rail is sent to the motor driver (TB6612FNG), which directly controls the brushed DC gearmotors, and also to the servo controller board (PCA9685), which features its own onboard 5V regulator - both these items are built into the HAT. This dual-stage regulation allows our system to consolidate power delivery to all moving parts while reducing wire complexity. One battery saves weight and improves reliability in extended runs. 

The RPi camera is our primary vision sensor, drawing approximately 200mA at 5V. It connects via MIPI CSI, ensuring low latency and reduced CPU overhead compared to a USB camera. Initially, we invested a lot of time and tools into testing out USB cameras, such as this [module](https://www.amazon.ca/OV2643-Camera-Module-Autofocus-Board/dp/B07QGZCF8N/ref=sr_1_5?sr=8-5) and this other [module](https://www.amazon.ca/Kano-Headphones-Bluetooth-Buildable-Booming/dp/B08KSBSZTG/ref=pd_ci_mcx_di_int_sccai_cn_d_sccl_2_3/145-3181089-8444629?psc=1). However, a comparative analysis showed that using non-usb camera would significantly reduce the CPU usage. While streaming a live feed on the USB cameras, we noticed perpetual falls and freezes in fps and it was becoming difficult to update data variables at the speed we had intended for. We opted for this camera over alternatives such as the LIDAR or ultrasonic sensors because it provides more context with less power than many discrete sensors. Through LAB color-space processing, the camera would identify walls, lanes, and pillars based on color and shape.

For manual input, we used a pull-button start mechanism on GPIO 17 configured with an internal pull-up resistor. 

### Camera Mount with a rotatable surface
<img width="526" height="406" alt="image" src="https://github.com/user-attachments/assets/7ea2144a-3e43-4e14-89fe-05befa14b130" /><img width="363" height="260" alt="image" src="https://github.com/user-attachments/assets/d47d7aab-bf35-47d0-a5a6-14ca4e4bee32" />

### Improvements

Firstly, the start button could benefit from a more rigid base, as the current design allows unintended flexing. Secondly, incorporating an additional sensor alongside the camera during parallel parking could have improved overall accuracy and provided more robust positional feedback. Finally, due to network transmission and the high computational load imposed by real-time camera processing, the streamed video feed was not consistently smooth. Occasional frame freezes could pose a significant challenge, as the robot relies on continuously updated visual data to accurately determine its position and orientation.



## 5. Software - Open Challenge - Obstacle Challenge - Parking

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

### Installing Libraries

``` sudo apt install smbus, board, busio, adafruit_vl53l0x```
- Relevant Libraries to interact with the Hat (Expansion Board) and Motor

``` sudo apt install python3-picamera2 ```
- Library to open the camera

``` sudo apt install cv2, math, numpy, time ```
- Relevant libraries to perform operations using the camera

``` pip install threading ```
-  Thread the core processor 

### Interacting with Motors and Servos

As outlined before, a Motor and Servo Driver Hat is mounted on the Raspberry Pi 40-pin header and offloads two jobs: generating PWM signals (using the PCA9685 chip) and driving the DC Motor (using the TB6612 H-Bridge). The Pi communicates with the PCA9685 using I2C, creating up to 16 PWM outputs. We set a default frequency (typically 50Hz for servos), which the PCA converts to a 12-bit count and outputs that duty cycle on the channel.

Using a class provided by the HAT itself, we call distinct functions from the class to execute our motor and servo commands. 
We create a class instance and set the frequency to 50:
```python
pwm = PCA9685()
pwm.setPWMFreq(50) 
```

We swiftly move the servo to the targeted angle using two functions:

```MoveServo```: This function takes a target angle that is calculated based on the car's surroundings, and converts it into a corresponding pulse width signal used to move the servo to that position. It first applies a calibrated offset to account for the servo's physical mounting so that the input angle correctly represents the center. The angle is thereafter clamped within the servo's safe ranges, converted to the PWM, and sent to the servo controller. 

```SweepServo```: This function allows the servo to smoothly reach the desired angle in small increments for a gradual sweep. It generates a sequence of intermediate angles between the start and target, and calls ```MoveServo``` for each step. After completing the sweep, it updates the servo's last position for future reference. 

``` python

def MoveServo(self, channel, angle, delay=0.02):

    # Add a calibrated offset to ensure that 0 is in fact the center of the servo, and not -50. This is based on the servo's position and its mounting.
    physical_angle = angle - 50
    physical_angle = max(-90, min(90, physical_angle)) 
    pulse = int(1500 + (physical_angle / 90.0) * 1000) # convert it to a pulse width 
    self.setServoPulse(channel, pulse) 
    time.sleep(delay)

def SweepServo(self, channel, end_angle, step=1, delay=0.02): 

    end_angle = max(-30, min(30, end_angle))
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
```

The ```Drive``` function controls the DC motor's speed and direction using PWM signals. The percent parameter sets the motor's speed by multiplying it by the maximum PWM pulse, while the wise parameter sets the rotation direction or stopping command. Depending on the value of wise, two pins  (INA1 and INA2) are set to High or Low to achieve the intended result.

```python
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
```

In a right turn, we will continue using the angle value calculated based on the area difference. Ultimately, when a right turn appears, the amount of contour in the right wall will decrease until the value eventually reaches 0. This, in turn, generates an angle that resembles a sharp right turn. To prevent this angle from exceeding the steering range, we clamp the values to its turn degree. 


```python
elif right_turn:
    # standard right procedure
    angle = int(MID_SERVO + (kp * areaDiff) + (kd * derivative_term))
    angle = max(-TURN_DEGREE, min(angle, TURN_DEGREE))
```

In a left turn, we noticed that the left steering was not as powerful as the right. We assumed that this was a result of a mechanical issue present within our chassis. Hence, when the robot travels clockwise, we introduced different Kp and kd values that would calculate an angle within the left turn condition.

```python
if left_turn:
  # Use left-specific PD vals
  angle = int(MID_SERVO + (kp_left * areaDiff) + (kd_left * derivative_term))
  angle = min(TURN_DEGREE_LEFT, max(angle, -TURN_DEGREE_LEFT))
  print("LEFT ANGLE", angle)
```
If the car is not in a left or right turn, the angle values would still be clamped, and to combat our left steering weakness, the negative angles would be offset by a small amount of -2 to make the car even smoother.

To prevent the same line from the subsequent line from being detected after a turn has been performed, we wait until both lines are out of sight and update a variable ``` line_released = True ```

```python
if turn_just_ended:
    # Wait until both lines are out of frame to prevent another turn on the same line
    if areaLineBlue < line_threshold and areaLineOrange < line_threshold:
        line_released = True
        turn_just_ended = False 
        print("LINE CLEARED!! ")
```

Once the counter has reached 12 (3 Full rounds) and the angle value is near zero (the car is straight), we wait for two seconds before turning the motor off.

```python
# Stop Car Logic 
if counter == 12 and abs(angle) <= 3:
    sleep(1)
    motor_command_queue.put("stop")
    servo_angle_queue.put(MID_SERVO)
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

#### Turning with Obstacles in the way

##### 1. Detecting when a turn should begin:

At the start, the robot scans the game mat to decide whether or not it needs to turn left or right. This is done by checking which line color appears intially (blue = left, orange = right):


```python
if track_Dir is None and line_released and not waiting_for_release:
    if areaLineBlue > line_threshold: 
        track_Dir = "left"
        line_released = False  # Reset for this turn
        print(track_Dir)

    elif areaLineOrange > line_threshold:
        track_Dir = "right"
        line_released = False
        print(track_Dir)
```


``` track_Dir``` keeps track of the current turn direction.

If the robot detects an area of the blue line larger than our line threshold, or minimum area to protect against accidental detection, (areaLineBlue > line_threshold), it sets the turn direction to left and vice versa for the orange line (areaLineOrange > line_threshold).

``` line_released``` ensures that the robot does not accidentally trigger multiple turns from the same line.

Essentially, this section of code is comparable to a decision gate, asking the question?: “Do I start a left turn, right turn, or keep going straight?”

##### 2. Executing the turn

Once the robot knows the direction (track_Dir is “left” or “right”), it needs to rotate by a certain angle to commence and complete the turn.

Code for right turn:

```python
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
```

While turning right, the robot adjusts its steering angle (``` angle```) based on:

  If blue line is not detected as well as a green pillar, it turns slightly more than normal (turn_Deg + 8).

  If blue line is not detected sees a red pillar, it turns a little less (turn_Deg + 4).

  If blue line is not detected and there is no pillar, it assumes a default turn (turn_Deg + 8).

Once the blue line is sensed (```python areaLineBlue >= line_threshold```), the turn is complete.

The same structure applies for left turns, just mirrored:

```python
if track_Dir == "left" and areaLineBlue > line_threshold:
    if areaLineOrange <= line_threshold and closest_pillar_color == "green":
        kp = 0.034
        greenTarget = 550
        angle -= (turn_Deg - 4)
        print("Left")
    elif areaLineOrange < line_threshold and closest_pillar_color == "red":
        ang = angle - 15
        angle = ang
        print("Left")
    elif areaLineOrange < line_threshold and closest_pillar_color == None:
        angle -= (turn_Deg - 8)
        print("Left")
    elif areaLineOrange >= line_threshold:
        kp = 0.045
        greenTarget = 600
        turn_End = True
        turn_counter += 1
        print(f"Turn {turn_counter} (LEFT)")
        track_Dir = None 
        turn_just_ended = True
```

##### 3. Ending the turn and counting progress

The last step is marking when a turn is finished.
This occurs when the opposing line comes back into view (e.g., for a right turn, the blue line reappears).

At that point:

  The robot resets ``` track_Dir = None``` so robot ready to detect the next turn.

  ``` turn_counter``` is incremented to keep track of how many turns have been made and to mark when to run the parallel parking code.

  ``` turn_just_ended``` is set to prevent double-counting.

Summary in plain words

  The robot detects a line color (blue or orange), telling it which way to turn.

  It then executes the turn by adjusting its steering angle. The exact adjustment depends on the colour of pillar detected or no pillar (default turn).

  When the opposing line color comes into view, it knows the turn is finished and subsequently resets for the next turn and adds to the turn counter.

  This ensures the robot can reliably follow a path with alternating left/right turns while adapting to obstacles (pillars).


#### Parking
##### Exiting Parking Spot


How do we enter the parking spot after 3 rounds?
How do we manage color detection between red pillars and magenta parking lot walls?

#### Flow Diagram - Obstacle Challenge

<img width="791" height="652" alt="image" src="https://github.com/user-attachments/assets/2f230ded-b8f1-44ba-b78e-60dafe4bf03d" />


#### 6. Assembly 

   
