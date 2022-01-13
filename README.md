# Quadcopter
## Hardware & Software

This repo contains a noobie quadcopter-build and implementation based on:
- Arduino Uno
- MPU6050 Gyroscope/Accelerometer module
- nRF24L01 Wireless communication module
- Power distribution board (XT60 male)
- Electronic speed controllers : [Turnigy MultiStar 30A BLHeli-S Rev16 V3 ESC 2~4S](https://hobbyking.com/de_de/blheli-s-30a.html), [Racerstar](https://de.banggood.com/4X-Racerstar-Racing-Edition-2205-BR2205-2300KV-2-4S-Brushless-Motor-For-QAV250-ZMR250-RC-Drone-FPV-Racing-p-1066837.html?cur_warehouse=CN)
- Motors : 2205 2300kV (brushless) 2xCW/2xCCW [Turnigy](https://hobbyking.com/de_de/brushless-motor-d2205-2300kv-cw.html); [RaceStar]
- Props 5045
- [3S Lipo Battery (XT60 female)](https://www.ampow.com/products/ovonic-50c-11-1-v-2200mah-3s1p-deans-lipo-battery?variant=34830436728988&utm_source=google&utm_medium=cpc&utm_campaign=Google+Shopping&currency=USD&gclid=EAIaIQobChMI6KPlmZeW9QIVgY9oCR33IQNXEAYYASABEgLOMPD_BwE)

## Next steps
- Checkout another interseting similar project.
- Tune PID parameters.[LINK](https://www.technik-consulting.eu/en/optimizing/drone_PID-optimizing.html)
- Refactor code in setup() function of flight controller: Rewrite mpu_setup
- Understand Transmitter AutoAcknoledge, DataRate... other settings
- Make prototypeboard to place gyro in the center of geometry/mass.
- Make condition to start mount motors dependent on two signals(joysticks) to reduce the probability of an error.
- Servo.h only works with 50Hz, Need higher refresh rate for a stable drone: https://forum.arduino.cc/t/adjusting-pwm-frequency/45204
  - Checkout:https://forum.arduino.cc/t/how-can-i-change-the-frequency-of-servo-library/148099/3,\ 
  - But writing directly to the outputpins and simulating the pwm pulse doesnt work yet (applaySpeed())
- Copyright-rules for code from other repositories?!
- Joystick dead-zone might be a problem.
- Install battery mount.
- Measure batteries voltage drop.
---
### ESC calibration

To have properly working ESC's/motors the ESC's have to be calibrated.
This is done by applying the minimum(1000us) and maximum(2000us) PWM-signal(pulse-width-modulation) to the ESC's while they are connected to a battery.\
[ESC_calibration file](code/ESC_calibration/ESC_calibration.ino) can be used for that.\
more about pulse-width-modulation: https://www.allaboutcircuits.com/textbook/semiconductors/chpt-11/pulse-width-modulation/
more about electronic speed controller calibration: https://ardupilot.org/copter/docs/esc-calibration.html\

To arm the ESC's, connect the battery with minimal throttle speed (1000us).



### IMU - MP6050 (+ Magnetometer)
Usefull websites:
- https://www.teachmemicro.com/orientation-arduino-mpu6050/ <- explanation.
- https://lastminuteengineers.com/mpu6050-accel-gyro-arduino-tutorial/ <- basic accel./gyro readings (angles still need to be calculated)
- https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050 <- Complete library for MPU readings! (but how to include magnetometer?)


- angular rate system implementation?
- PWM frequency for ESC's via analogWrite() rather than Servo.h 490Hz/50Hz -> might not be responsive enough
- digital low pass filter for MPU6050 20Hz?
- I2C - Communication
- DMP (digital motion processor) - programming to include magnetometer (future work)
- Interrupt Pin signales board when new data is available.

### Flight-Control for self stabilizing quadcopter

Similar projects: https://github.com/rohanverma94/The-Open-Copter/blob/master/quadcopter-documentation/quadcopter-arduino.pdf \
https://github.com/lobodol/drone-flight-controller

Features that effect minimal loop time/refresh rate of flight controller: [more info](https://oscarliang.com/best-looptime-flight-controller/)
- ESC's refresh rate. When max pulse is 2ms loop cant refresh motor speed faster.
- Gyro refresh rate & low pass filter(introduces delay)


#### Flight controller procedure/features
- SETUP
  - Configure DMP(Digital Motion Processor) of MPU6050
  - Calibrate Gyro (on drone startup, else wrong offsets)
- LOOP(AIR-Routine)   
  - Arm(AUX1) & Disarm(AUX2) drone. 
  - read reviever data \[throttle, yaw, pitch, roll\] from nRF24L01-module
  - calc current angles \[pitch, roll, yaw\] from MPU6050
  - calc PID-Terms for each angle
  - Adjust ESC pulse (currently via Servo.h) based in PID-terms
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
