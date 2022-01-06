Problems i came across:

- To counter the torque on the drone 2 clockwise and 2 counter-clockwise motors are required. Else the drone will sping around the yaw-axis.
- The Transmitter&Receiver regulary lose signal : 
  - The cause may be current surges/drops. This this can be solved by soldering a capacitor to the nRF24 modules VSS & ground pin.
  - Test with receiver_raw.ino works fine. Must be a problem in the flight controller code.
