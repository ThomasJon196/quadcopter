# Quadcopter
## Build & Software

This repo contains a noobie quadcopter-build and implementation based on:
- Arduino Uno
- MPU6050 Gyroscope/Accelerometer module
- nRF24L01 Wireless communication module
- Power distribution board (XT60 male)
- Electronic speed controllers : Turnigy MultiStar 30A BLHeli-S Rev16 V3 ESC 2~4S 
- Motors : 2205 2300kV (brushless) 2xCW/2xCCW
- Props 5045
- 3S Lipo Battery (XT60 female)

## Next steps

- Calibrate gyro on startup
- Read/Write HC-05 (Bluetooth Module) for PID-Tuning
- PID Tuning **Fix quadcopter along 1 axis e.g. only pitch or roll** -> example:https://www.youtube.com/watch?v=yvame7QLWbo
  - Add a proportional control to improve the rise time
  - Add a derivative control to reduce the overshoot
  - Add an integral control to reduce the steady-state error

### IMU - MP6050 (+ Magnetometer)
Usefull websites:
- https://www.teachmemicro.com/orientation-arduino-mpu6050/ <- explanation.
- https://lastminuteengineers.com/mpu6050-accel-gyro-arduino-tutorial/ <- basic accel./gyro readings (angles still need to be calculated)
- https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050 <- Complete library for MPU readings! (but how to include magnetometer?)

- Gyroscope + Accelerometer
- I2C - Communication
- DMP (digital motion processor) - programming to include magnetometer (future work)

- (Interrupt Pin signales board when new data is available.

### Flight-Control for self stabilizing quadcopter

Similar project: https://github.com/rohanverma94/The-Open-Copter/blob/master/quadcopter-documentation/quadcopter-arduino.pdf
https://reefwing.medium.com/how-to-write-your-own-flight-controller-software-part-1-ac08b6ecc01e

#### Flight controller procedure
- SETUP
  - Configure DMP(Digital Motion Processor) of MPU6050
  - Calibrate Gyro (on drone startup, else wrong offsets) **TODO**
  - Setup ESC's/Motors with PWM(Pulse width modulation) signal. (e.g. https://ardupilot.org/copter/docs/esc-calibration.html)
- LOOP(AIR-Routine)   
  - read reviever data (nRF24L01-module)
  - read current angle from IMU (intertial measurement unit)
  - calculate pid values for pitch, roll, yaw
  - Adjust ESC pulse
  - (integrate battery voltage)



|Question|Answer|
|---|---|
|Why?           | Figure out how interesting robotics might be and learn to build drone. (70/30)|
|When?          | 2021 |
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
