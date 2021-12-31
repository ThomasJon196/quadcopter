# Quadcopter
## Build & Software

This repo contains a noobie quadcopter-build and implementation based on:
- Arduino Uno
- MPU6050 Gyroscope/Accelerometer module
- nRF24L01 Wireless communication module
- Electronic speed controllers :
- Motors
- Power distribution board (XT60 male)
- 3S Lipo Battery (XT60 female)

## Next steps

- Calibrate gyro on startup
- Read/Write HC-05 (Bluetooth Module) for PID-Tuning



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


