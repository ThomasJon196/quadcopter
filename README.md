# Quadcopter
## Hardware & Software

This repo contains a noobie quadcopter-build and implementation based on:
- Arduino Uno
- MPU6050 Gyroscope/Accelerometer module
- nRF24L01 Wireless communication module
- Power distribution board (XT60 male)
- Electronic speed controllers : [Turnigy MultiStar 30A BLHeli-S Rev16 V3 ESC 2~4S](https://hobbyking.com/de_de/blheli-s-30a.html)
- Motors : 2205 2300kV (brushless) 2xCW/2xCCW
- Props 5045
- [3S Lipo Battery (XT60 female)](https://www.ampow.com/products/ovonic-50c-11-1-v-2200mah-3s1p-deans-lipo-battery?variant=34830436728988&utm_source=google&utm_medium=cpc&utm_campaign=Google+Shopping&currency=USD&gclid=EAIaIQobChMI6KPlmZeW9QIVgY9oCR33IQNXEAYYASABEgLOMPD_BwE)

## Next steps

- Include safety condition for disarming drone (if-Case for angle)
- Read/Write HC-05 (Bluetooth Module) for PID-Tuning (maximal baud-rate for HC-05?)
- PID Tuning **Fix quadcopter along 1 axis e.g. only pitch or roll** -> example:https://www.youtube.com/watch?v=yvame7QLWbo
  - Add a proportional control to improve the rise time
  - Add a derivative control to reduce the overshoot
  - Add an integral control to reduce the steady-state error
- Copyright-rules for code from other repositories?!
- Joystick dead-zone might be a problem.
---
### ESC calibration

To have properly working ESC's/motors the ESC's have to be calibrated.
This is done by applying the minimum(1000us) and maximum(2000us) PWM-signal(pulse-width-modulation) to the ESC's while they are connected to a battery.\
[ESC_calibration file](code/ESC_calibration/ESC_calibration.ino) can be used for that.\
more about pulse-width-modulation: https://www.allaboutcircuits.com/textbook/semiconductors/chpt-11/pulse-width-modulation/
more about electronic speed controller calibration: https://ardupilot.org/copter/docs/esc-calibration.html\

To arm the ESC's, connect the battery with minimal throttle speed (1000us).

### Arduino Baud-Rate
 
- Sets the datarate in bits/s for the serial communication.
- A to small baud-rate might slow down the loop(), if many Serial.prints are executed. Buffer hast to be reserved for each character. If the baud-rate is slow it takes longer to send one.
- Remove all unneccecery Serial.prints from loop(), or safe them otherweise to read later.


### IMU - MP6050 (+ Magnetometer)
Usefull websites:
- https://www.teachmemicro.com/orientation-arduino-mpu6050/ <- explanation.
- https://lastminuteengineers.com/mpu6050-accel-gyro-arduino-tutorial/ <- basic accel./gyro readings (angles still need to be calculated)
- https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050 <- Complete library for MPU readings! (but how to include magnetometer?)


- angular rate system implementation?
- PWM frequency for ESC's via analogWrite() rather than Servo.h 490Hz/50Hz -> might not be responsive enough
- digital low pass filter for MPU6050 20Hz?

- Gyroscope + Accelerometer
- I2C - Communication
- DMP (digital motion processor) - programming to include magnetometer (future work)

- (Interrupt Pin signales board when new data is available.

### Flight-Control for self stabilizing quadcopter

Similar projects: https://github.com/rohanverma94/The-Open-Copter/blob/master/quadcopter-documentation/quadcopter-arduino.pdf\\
https://reefwing.medium.com/how-to-write-your-own-flight-controller-software-part-1-ac08b6ecc01e\\
https://github.com/lobodol/drone-flight-controller

#### Flight controller procedure/features
- SETUP
  - Configure DMP(Digital Motion Processor) of MPU6050
  - Calibrate Gyro (on drone startup, else wrong offsets) **TODO**
- LOOP(AIR-Routine)   
  - Arm(AUX1) & Disarm(AUX2) drone. 
  - read reviever data \[throttle, yaw, pitch, roll\] from nRF24L01-module
  - read current angles \[pitch, roll, yaw\] from MPU6050
  - Adjust ESC pulse (currently via Servo.h)
  - (integrate battery voltage) **TODO**

---
## Drone Build

#### Arduino

#### Frame

#### Motors, Electronic Speed Controllers & Propellors

#### Inertial Measurement Unit

#### Communication

https://github.com/akarsh98/DIY-Radio-Controller-for-Drone-Arduino-Based-Quadcopter

#### Power Distribution

#### Battery

Lipo Guide: https://rogershobbycenter.com/lipoguide\
[Why storage voltage is only important for long lasting (years) batteries.](https://www.propwashed.com/lipo-storage-voltage/)







|Question|Answer|
|---|---|
|Why?           | Figure out how interesting robotics might be and learn to build drone.|
|When?          | starting november 2021|
|Importance     | Below job search and preperation for master |
|How much time? | 3d / week |
|Finished       | Self stabilizing drone works.|

















# Markdown Cheat Sheet

Thanks for visiting [The Markdown Guide](https://www.markdownguide.org)!

This Markdown cheat sheet provides a quick overview of all the Markdown syntax elements. It can’t cover every edge case, so if you need more information about any of these elements, refer to the reference guides for [basic syntax](https://www.markdownguide.org/basic-syntax) and [extended syntax](https://www.markdownguide.org/extended-syntax).

## Basic Syntax

These are the elements outlined in John Gruber’s original design document. All Markdown applications support these elements.

### Heading

# H1
## H2
### H3

### Bold

**bold text**

### Italic

*italicized text*

### Blockquote

> blockquote

### Ordered List

1. First item
2. Second item
3. Third item

### Unordered List

- First item
- Second item
- Third item

### Code

`code`

### Horizontal Rule

---

### Link

[Markdown Guide](https://www.markdownguide.org)

### Image

![alt text](https://www.markdownguide.org/assets/images/tux.png)

## Extended Syntax

These elements extend the basic syntax by adding additional features. Not all Markdown applications support these elements.

### Table

| Syntax | Description |
| ----------- | ----------- |
| Header | Title |
| Paragraph | Text |

### Fenced Code Block

```
{
  "firstName": "John",
  "lastName": "Smith",
  "age": 25
}
```

### Footnote

Here's a sentence with a footnote. [^1]

[^1]: This is the footnote.

### Heading ID

### My Great Heading {#custom-id}

### Definition List

term
: definition

### Strikethrough

~~The world is flat.~~

### Task List

- [x] Write the press release
- [ ] Update the website
- [ ] Contact the media

### Emoji

That is so funny! :joy:

(See also [Copying and Pasting Emoji](https://www.markdownguide.org/extended-syntax/#copying-and-pasting-emoji))

### Highlight

I need to highlight these ==very important words==.

### Subscript

H~2~O

### Superscript

X^2^
