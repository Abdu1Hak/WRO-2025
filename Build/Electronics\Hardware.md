# Electrical


This section discusses the various hardware components that we utilize to complete the challenge as well as the reasonings about why we selected these parts.

## Parts Explanation

| Part Name | Explanation | Image | Datasheet |
| ----------- | ----------- | ----------- | ----------- |
| Raspberry Pi 4 Model B Quad Core 64 Bit WiFi Bluetooth (2GB)  | We selected the Raspbeery Pi 4 to be the brains of our robot for a myriad of reasons. First, the Raspbeery Pi is well known, respected, and reliable brand that already has multiple accessories designed for it as well as multiple libraries available, making our lives simpler and easier. The Raspbeeru Pi 4 is extremely powerful with excellent image processing, this allows us to use solely a Raspberry Pi 4 camera where other teams using Arduinos or ESPs are forced to use a mosaic os ensors including a camera, multiple Time of Flight sensors, colour sensors, as well as gyro sensors, which gives us a huge advantage in streamlining our design and code. The Raspberry Pi 4 contains 20 digital input/output pins allowing us to control various devices. | ![image]([https://drive.google.com/file/d/1jcntYn8FrEI_t9AaL071cwJKFTuHKwKT/view?usp=sharing]) | [Raspberry Pi 4 Information](https://drive.google.com/file/d/15USfiaweyP4RiKymB9xjHh8MpDuqv5D5/view?usp=drive_link) |
| XICOOLEE Motor and Servo Driver HAT for Raspberry Pi Zero/Zero W/Zero WH/2B/3B/3B+/4B, Adopting Dual Chips PCA9685+TB6612 | Without using a motor and servo Driver HAT, attempting to control our motor and servo would be a nightmare, between the immense wire management to the unneseccarily complex code, it does not make sense not to use one. | ![image]) | [Motor Driver Info]()  [Schematic](Motor%20And%20Servo%20Driver%20HAT.pdf) |
| [Insert Camera Name] | As we are only using a camera to guide the robot through both challenges, we believed it would be important to invest a reliable and functional camera that would act as the backbone for our robot. We selected a camera with an extremely wide field of view along the x-axis so we are able to adequately view both the inner and outer walls for the open challenge and additionally to easily detect obstacles no matter the robots orientation on the game mat. we also selected a camera that had a high image quality at [insert megapixels] so we can prevent accidental misidentifiaction of our surroundings. | ![image](https://drive.google.com/uc?id=1V1Nqks-wj--PqYVI9ksZawQXJftz3UOQ) | [[Camera] Info](https://docs.pixycam.com/wiki/doku.php?id=wiki:v2:start) |
| [Insert Motor Name], rpm | This motor was selected as it provides a more than enough speed and power at 360 rpm and [insert number]ft-lb of torque while still being accurate enoguh to give as precise movements. | ![image](https://drive.google.com/uc?id=1eZ35c58Pk-_ApFsSc1q7nTVgRgd4GYDa) | [DC Motor Info](https://drive.google.com/file/d/1ovx4JvY0TAlGeaWuEGIg9i7RbAIlBQif/view?usp=drive_link) |
| [Insert Servo Name] | Similar to this: *In order to steer our vehicle, we use a servo motor with 2.0 Kilogram-force centimetres of torque and a speed of 0.10 sec/60°. Since steering will require a reasonably large amount of torque, this motor provides more than enough for our application. Furthermore, this servo motor can turn quickly, which allows us to make incredibly sharp turns.* | ![image](https://drive.google.com/uc?id=1PW3WRTPNTWx5we4OpXFKie0twWL3cDdZ) | [Servo Info](https://m.media-amazon.com/images/I/61ZU3A84tYS._AC_SL1000_.jpg) |
| [Insert Battery Name] |  | ![image]() | [Battery Info](https://m.media-amazon.com/images/I/61ZU3A84tYS._AC_SL1000_.jpg) |


## Parts Bill Of Materials (BOM)
| Part Name | Quantity |
| ----------| -------- |
| 1.5V Rechargeable Lithium Ion Battery | 4 |
| DC 6V Micro Gear Box Speed Reduction Motor, 300RPM | 2 |

| Raspberry Pi 4 Model B Quad Core 64 Bit WiFi Bluetooth (2GB) | 1 |
| ES08MA Metal Analog Servo | 1 |

| XICOOLEE Motor and Servo Driver HAT for Raspberry Pi Zero/Zero W/Zero WH/2B/3B/3B+/4B, Adopting Dual Chips PCA9685+TB6612 | 1 |

| PixyCam Pixy 2.1 Robot Vision Image Sensor | 1 |
