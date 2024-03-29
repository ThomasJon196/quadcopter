
15.01.2022 - Changed Servo.h library frequency from 50Hz -> 250Hz


## Problems i came across:

|Problem|Solution|
|---|---|
|All for motors spin in the same direction, resulting in constant yaw movement due to torque. | Mount 2 clockwise and 2 counter-clockwise motors.|
|Is the timedelta required for d-term calculation? |No. Should be constant most of the time. |
|Drone has loses controll right after start and drifts in one direction. | - Might be transmitter signal which sends a slight drift. -> disabled|




### Serial port busy
- When Arduino expects Serial input, one can not write code to it.-> RESTART

### Input and Output with Pulse width modulation (PWM)
- Input [source](http://www.benripley.com/diy/arduino/three-ways-to-read-a-pwm-signal-with-arduino/)
- Output 

### Motors
- To counter the torque on the drone 2 clockwise and 2 counter-clockwise motors are required. Else the drone will sping around the yaw-axis.

### PID-Controller
What is a PID-Controller and how is it used?
- A PID-Controller adjusts the controlling mechanism of a system, to converge to a desired system-state.\
- In this case:
  - System: Quadcopter
  - Controlling mechanism: Motors/Electronic Speed Controller
  - System-State: Orientation of quadcopter (yaw, pitch, roll)-angles
- The PID-C calculates how the motor speed has to be changed, to reach a desired (yaw, pitch, roll)-angle-orientation of the quadcopter. For hovering that would be (0, 0, 0). (or (x, 0, 0) since hovering can be achieved without fixing the vertical-yaw-axis)

PID explained:

- P(roportional)
- I(ntegral)
- D(erivative) [source](https://oscarliang.com/excessive-d-gain-cause-oscillations-motor-overheat/)

PID-Tuning: [source](https://oscarliang.com/quadcopter-pid-explained-tuning/)


Is the time_delta term required for the derivative-term?
- Nope. Should be ~constant.


### Gyroscope
- How does slight shaking influence Gyro calibration?
Should cancel itself out.


### Communication

- The Transmitter&Receiver regulary lose signal : 
  - The cause may be current surges/drops. This this can be solved by soldering a capacitor to the nRF24 modules VSS & ground pin.
  - Test with receiver_raw.ino works fine. Must be a problem in the flight controller code.
  - timer variables were not set properly for lost signal identification.


### LiPo Battery

How to handle Lipo batteries? 
[Battery handling guide](https://www.robotshop.com/media/files/pdf/hyperion-g5-50c-3s-1100mah-lipo-battery-User-Guide.pdf)
[Lipo charger manual](https://cdn.sparkfun.com/assets/c/d/8/1/5/16793-SkyRC_IMAX_B6_V2_Balance_Charger_-_Discharger_Instruction_Manual_EN_V1.0.pdf)


### Arduino Baud-Rate
 
- Sets the datarate in bits/s for the serial communication.
- A to small baud-rate might slow down the loop(), if many Serial.prints are executed. Buffer hast to be reserved for each character. If the baud-rate is slow it takes longer to send one.
- Remove all unneccecery Serial.prints from loop(), or safe them otherweise to read later.


### License

If you find software that doesn’t have a license, that generally means you have no permission from the creators of the software to use, modify, or share the software. Although a code host such as GitHub may allow you to view and fork the code, this does not imply that you are permitted to use, modify, or share the software for any purpose.

Your options:

Ask the maintainers nicely to add a license. Unless the software includes strong indications to the contrary, lack of a license is probably an oversight. If the software is hosted on a site like GitHub, open an issue requesting a license and include a link to this site, or if you’re bold and it’s fairly obvious what license is most appropriate, open a pull request to add a license.

Don’t use the software. Find or create an alternative that is under an open source license.

Negotiate a private license. Bring your lawyer.
