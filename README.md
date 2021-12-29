# Quadcopter
## Build & Software


|Question|Answer|
|---|---|
|Why?           | Figure out how interesting robotics might be and learn to build drone. (70/30)|
|When?          | 2021 |
|Importance     | Below job search and preperation for master |
|How much time? | 3d / week |
|Finished       | Self stabilizing drone works.|


## Next steps

### IMU - MP6050 (+ Magnetometer)
- https://www.teachmemicro.com/orientation-arduino-mpu6050/ <- explanation.

- Gyroscope + Accelerometer
- I2C - Communication




- (Interrupt Pin signales board when new data is available.

### Flight-Control for self stabilizing quadcopter

Similar project: https://github.com/rohanverma94/The-Open-Copter/blob/master/quadcopter-documentation/quadcopter-arduino.pdf

- Write minimal simulation
- Which sensors are neccecary to stabilize drone?
- How to write the PID-Controller?


#### Flight controller procedure
- Calibrate Gyro
- LOOP
  - Start-Routine: start motors
  - Stop-Routine:  turn of motors
  - Air-Routine:   
    - measure current angle
    - calculate pid values for pitch, roll, yaw
    - Adjust ESC pulse
    - (integrate battery voltage)



