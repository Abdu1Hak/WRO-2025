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

### Using VNC

Our primary method of interfacing with the Raspberry Pi to monitor the camera feed was through RealVNC, establishing a wireless connection between the Pi and our system on the same network. By accessing the Pi’s IP address, we were able to mirror its desktop environment directly on our screen. While this approach initially introduced little overhead, performance bottlenecks quickly emerged, primarily in the form of latency and memory limitations.

When running our open challenge code, we observed frame rates dropping to between 0–5 FPS. The root cause was the Pi’s limited 2 GB of RAM, which could not simultaneously handle intensive video processing and real-time motor control. To resolve this, we restructured execution by assigning tasks to separate CPU cores using threading: one core dedicated to motor commands, another to servo operations, and a third to managing the camera feed. This distribution of workload significantly reduced contention and stabilized performance, resulting in a consistent throughput of approximately 30 FPS. 


### Open Challenge

Our main strategy for autonomous travel was to use proportional derivative control with a wide-angle camera, a FOV of 170 degrees. In the open challenge, the only components visible to the camera were the two parallel inner and outer walls, as well as the colored lines indicating a turn. The code begins by measuring how many contours are present in the ROI (Region of Interest) for both walls, and by calculating the area of these contours, it becomes a means of determining how much of the walls are visible to the camera. By calculating the difference between the left and right walls based on the contours on either side, which is again based on the horizontal position of the car, we can determine how far we are from the desired trajectory. This area difference, also known as Cross Track Error (Ep), would be multiplied by a scaling factor called the proportional gain, which would result in a steering angle. Overall, the gain that would satisfy the requirements of our vehicle would be 0.005, and this value is subject to change. The main reason why our value is so small is that the servo's range of movement is between -25 degrees (far left) and 25 degrees (far right), and our car is relatively slow. 

With proportional control, if the car only sees a single wall, it can enter the center line (center of both walls) crookedly. The result of this is that the controller will repeatedly overshoot the actual desired trajectory and not actually follow it. To correct this overshooting problem, we consider additional error measurements to update our steering command. The Derivative term  allows us to understand how fast we are moving in a perpendicular direction with respect to the desired trajectory. If the cross-track error is a low figure, or 0, it means we are perfectly following the desired trajectory. So by calculating the difference between our current area difference to the previous area difference, we get our derivative term, which likewise is subject to a derivative gain, allowing us to eventually close that overshooting gap in the instance that the car follows a zig-zag pattern. To prevent cases where our car would underdamp or overdamp, our team would continuously test different numerical figures and would eventually settle on 0.0004. This would allow our slow car to perfectly navigate the rounds smoothly without overshooting or underdamping because of this robust algorithm to determine the best angle

How does the car steer without hitting the walls?
How do we manage turns?
When do we end a round?

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
   
