# Electrical


This section discusses the various hardware components that we utilize to complete the challenge as well as the reasonings about why we selected these parts.

## Parts Explanation

| Part Name | Explanation | Image | Datasheet |
| ----------- | ----------- | ----------- | ----------- |
| Raspberry Pi 4 Model B Quad Core 64 Bit WiFi Bluetooth (8GB)  | We selected the Raspbeery Pi 4 to be the brains of our robot for a myriad of reasons. First, the Raspbeery Pi is well known, respected, and reliable brand that already has multiple accessories designed for it as well as multiple libraries available, making our lives simpler and easier. The Raspbeeru Pi 4 is extremely powerful with excellent image processing, this allows us to use solely a Raspberry Pi 4 camera where other teams using Arduinos or ESPs are forced to use a mosaic os ensors including a camera, multiple Time of Flight sensors, colour sensors, as well as gyro sensors, which gives us a huge advantage in streamlining our design and code. The Raspberry Pi 4 contains 20 digital input/output pins allowing us to control various devices. | <img width="250" height="453" alt="image" src="https://github.com/user-attachments/assets/e6d0b9e9-11a2-4c07-b357-e797b0dc4428" />
 |[Datasheet](https://drive.google.com/file/d/15USfiaweyP4RiKymB9xjHh8MpDuqv5D5/view?usp=drive_link) |
| XICOOLEE Motor and Servo Driver HAT for Raspberry Pi Zero/Zero W/Zero WH/2B/3B/3B+/4B, Adopting Dual Chips PCA9685+TB6612 | The Seengreat Motor Driver HAT is a Raspberry Pi add-on board using a PCA9685 + TB6612FNG solution. It supports control of 2 DC motors and up to 6 servos simultaneously, with I²C-based 12-bit PWM (4096 steps) for smooth motion control. The TB6612FNG dual H-bridge provides up to 1A continuous current per channel (2–3A peak), with forward, reverse, brake, and stop modes. Designed for Pi models from Zero to 4B, it simplifies motor and servo control while reducing CPU overhead. | <img width="709" height="346" alt="image" src="https://github.com/user-attachments/assets/2f971bcb-b333-4e48-b9b1-eb45aabc7259" />
 | [Motor Driver Info]()  [Schematic](Motor%20And%20Servo%20Driver%20HAT.pdf) |
| Raspberry Pi Camera Module, Fisheye Lens, Wider Field of View] | As we are only using a camera to guide the robot through both challenges, we believed it would be important to invest a reliable and functional camera that would act as the backbone for our robot. We selected a camera with an extremely wide field of view along the x-axis so we are able to adequately view both the inner and outer walls for the open challenge and additionally to easily detect obstacles no matter the robots orientation on the game mat. we also selected a camera that had a high image quality at [insert megapixels] so we can prevent accidental misidentifiaction of our surroundings. This is the Raspberry Pi Camera Module with fisheye lens, 5 MP OV5647 sensor, adjustable focus, wide 200° field of view, supports 1080p resolution. Compatible with all Raspberry Pi boards. | ![image](https://drive.google.com/uc?id=1V1Nqks-wj--PqYVI9ksZawQXJftz3UOQ) | [[Camera] Info](https://www.amazon.ca/Raspberry-pi-Camera-Raspberry-Pi-megapixel-Fisheye/dp/B078MN1KRM?utm_source=chatgpt.com) |
| YFROBOT Metal Gearmotor GA25 12V with 360 CPR Encoder photoelectric speed measurement |25 mm brushed DC gearmotor with integrated quadrature encoder, 12 V rated, 4 mm D-shaft. Gearbox uses helical first stage for reduced noise and improved efficiency. | <img width="509" height="412" alt="image" src="https://github.com/user-attachments/assets/5597ac5b-e783-4651-9267-1d826d0e197c" />
 | [DC Motor Info](https://yfrobot.com/en-ca/products/metal-gearmotor-ga25-12v-with-360-cpr) |
| Miuzei MG90S 9G Micro Servo Motor |Compact 9 g metal-gear micro servo designed for robotics and RC applications. It offers a good balance of torque and speed in a lightweight package, making it well-suited for steering and small mechanical actuation.| <img width="189" height="180" alt="image" src="https://github.com/user-attachments/assets/01554dc0-91d5-4727-8bd5-2837a9f85232" />
| [Servo Info](https://www.amazon.ca/Miuzei-Geared-Helicopter-Arduino-Project/dp/B0CP98TZJ2?th=1) |
| 12V lithium-ion power bank (TalentCell YB1203000-USB) | Our vehicle is powered by a 12V lithium-ion power bank (TalentCell YB1203000-USB), chosen for its fair weight 190 grams, current output of up to 3A and operating temperature range (-20C to 60C). This battery was ideal because it delivered adequate current to run the Pi, motor, servo, and others without overheating. The 12V rail is sent to the motor driver (TB6612FNG), which directly controls the brushed DC gearmotors, and also to the servo controller board (PCA9685), which features its own onboard 5V regulator | <img width="292" height="198" alt="image" src="https://github.com/user-attachments/assets/21449a97-ff57-442c-b2dc-10486294dc1e" />
 | [Battery Info](https://talentcell.com/lithium-ion-battery/12v/yb1203000-usb.html) |


## Parts Bill Of Materials (BOM)
| Part Name | Quantity | Price |
| ----------| -------- | -------- |
| Raspberry Pi 4 Model B Quad Core 64 Bit WiFi Bluetooth (2GB) | 1 | $166.99 |
| XICOOLEE Motor and Servo Driver HAT for Raspberry Pi Zero/Zero W/Zero WH/2B/3B/3B+/4B, Adopting Dual Chips PCA9685+TB6612 | 1 | $ 25.30 |
| Raspberry Pi Camera Module | 1 | $54.80 |
| YFROBOT Metal Gearmotor | 1 | $34.00 |
| Miuzei MG90S 9G Micro Servo Motor | 1 | $8.99 | 
| 12V lithium-ion power bank (TalentCell YB1203000-USB | 1 | $42.99 |
| | Total Price | $333.07 |
