## Problems i came across:

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


### License

If you find software that doesn’t have a license, that generally means you have no permission from the creators of the software to use, modify, or share the software. Although a code host such as GitHub may allow you to view and fork the code, this does not imply that you are permitted to use, modify, or share the software for any purpose.

Your options:

Ask the maintainers nicely to add a license. Unless the software includes strong indications to the contrary, lack of a license is probably an oversight. If the software is hosted on a site like GitHub, open an issue requesting a license and include a link to this site, or if you’re bold and it’s fairly obvious what license is most appropriate, open a pull request to add a license.

Don’t use the software. Find or create an alternative that is under an open source license.

Negotiate a private license. Bring your lawyer.
