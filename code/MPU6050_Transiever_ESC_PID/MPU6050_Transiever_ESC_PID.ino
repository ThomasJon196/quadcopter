
// ================================================================
// ===                     MPU - CONFIG                         ===
// ================================================================

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif


MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 17 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ================================================================
// ===                     ESC - SETUP                          ===
// ================================================================
#include <Servo.h>
int throttle_input, yaw_input, pitch_input, roll_input;

byte servoPin_FrontRight = 3;        // ESC Signal pin
byte servoPin_RearRight = 4;    // ESC Signal pin
byte servoPin_RearLeft = 5;      // ESC Signal pin
byte servoPin_FrontLeft = 6;        // ESC Signal pin
Servo servo_FrontLeft, servo_RearRight, servo_FrontRight, servo_RearLeft;
bool motor_on_flag = false;

// ================================================================
// ===                     Reciever - SETUP                     ===
// ================================================================
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
const uint64_t pipeIn = 0xE8E8F0F0E1LL; //Remember that this code is the same as in the transmitter
RF24 radio(9, 10); 
unsigned long lastRecvTime = 0;

//We could use up to 32 channels
struct MyData {
  byte throttle; //We define each byte of data input, in this case just 6 channels
  byte yaw;
  byte pitch;
  byte roll;
  byte AUX1;
  byte AUX2;
};
MyData data;



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // Set ESC pins
    servo_FrontRight.attach(servoPin_FrontRight);
    servo_RearRight.attach(servoPin_RearRight);
    servo_RearLeft.attach(servoPin_RearLeft);
    servo_FrontLeft.attach(servoPin_FrontLeft);

    // Set Reciever
    resetData();
    radio.begin();
    radio.setAutoAck(false);
    radio.setDataRate(RF24_250KBPS);
    radio.openReadingPipe(1,pipeIn);
    //we start the radio comunication
    radio.startListening();

    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    //while (Serial.available() && Serial.read()); // empty buffer
    //while (!Serial.available());                 // wait for data
    //Serial.println(F("\nTurn pitch all the way up: "));
    //recvData();
//    while (!(data.pitch > 230)){
//      recvData();// wait for Max pitch signal
//    }
    //while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(-93);
    mpu.setYGyroOffset(115);
    mpu.setZGyroOffset(381);
    mpu.setZAccelOffset(773); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    
    motor_on_flag = true;
}

// Variables for PID control (have to be tuned)
double yaw_error, pitch_error, roll_error; 
double yaw_adjust, pitch_adjust, roll_adjust;
int last_time, current_time, time_delta;
int Kp_YPR[] = {2,2,2};
int Ki_YPR[] = {0.1,0.1,0.1};
int Kd_YPR[] = {10,10,10};
double last_reading[3];
float yaw_PID[3], pitch_PID[3], roll_PID[3];
float integrated_error_yaw;
float esc_1, esc_2, esc_3, esc_4;
float throttle;
bool motor_cal_flag = true;

void loop() {
    // Recieve from nRF24L01
    recvData();
    unsigned long now = millis();
    //Here we check if we've lost signal, if we did we reset the values 
    if ( now - lastRecvTime > 1000 ) {
    // Signal lost?
    resetData();
    }
    throttle_input = map(data.throttle, 127, 255, 1000, 1900);
    yaw_input      = map(data.yaw, 0, 255, -127, 127);
    pitch_input    = map(data.pitch, 0, 255, -127, 127);
    roll_input     = -map(data.roll, 0, 255, -127, 127); // Roll input inverted

    if(pitch_input > 115 and throttle_input < 900){ // Turn of motor if pitch all the way up and throttle all the way down
      motor_on_flag = false;
    }
    
  
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO

    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//            Serial.print("ypr\t");
//            Serial.print(ypr[0] * 180/M_PI);
//            Serial.print("\t");
//            Serial.print(ypr[1] * 180/M_PI);
//            Serial.print("\t");
//            Serial.println(ypr[2] * 180/M_PI);
        #endif
    
    
    
    // PID control
    current_time = millis();
    time_delta   = current_time - last_time;
    last_time = current_time;
    yaw_error = yaw_input - ypr[0] * 180/M_PI;
    pitch_error = pitch_input - ypr[1] * 180/M_PI;
    roll_error = roll_input - ypr[2] * 180/M_PI;


    // Calc each P,I,D-value for yaw, pitch, roll
    yaw_PID[0] = Kp_YPR[0] * yaw_error;
    integrated_error_yaw += yaw_error; 
    yaw_PID[1] = Ki_YPR[0] * integrated_error_yaw;
    yaw_PID[2] = Kd_YPR[0] * (last_reading[0] - yaw_error);
    pitch_PID[0] = Kp_YPR[1] * pitch_error;
    pitch_PID[1] = Ki_YPR[1] * (last_reading[1] + pitch_error);
    pitch_PID[2] = Kd_YPR[1] * (last_reading[1] - pitch_error)/time_delta;
    roll_PID[0] = Kp_YPR[2] * roll_error;
    roll_PID[1] = Ki_YPR[2] * (last_reading[2] + roll_error);
    roll_PID[2] = Kd_YPR[2] * (last_reading[2] - roll_error)/time_delta;
    
    yaw_adjust = (yaw_PID[0] + yaw_PID[1] + yaw_PID[2]);
    pitch_adjust = (pitch_PID[0] + pitch_PID[1] + pitch_PID[2]);
    roll_adjust = (roll_PID[0] + roll_PID[1] + roll_PID[2]);
        
    last_reading[0] = yaw_error;
    last_reading[1] = pitch_error;
    last_reading[2] = roll_error;

    throttle = throttle_input; // Hover speed. Have to be adjusted (Or automatically calculated)
    if(throttle > 1700){
      throttle = 1700;
    }
    if(throttle < 1150){
      throttle = 1150;
    }

    
    esc_1 = throttle - pitch_adjust + roll_adjust - yaw_adjust; //Calculate the pulse for esc 1 (front-right - CCW)
    esc_2 = throttle + pitch_adjust + roll_adjust + yaw_adjust; //Calculate the pulse for esc 2 (rear-right - CW)
    esc_3 = throttle + pitch_adjust - roll_adjust - yaw_adjust; //Calculate the pulse for esc 3 (rear-left - CCW)
    esc_4 = throttle - pitch_adjust - roll_adjust + yaw_adjust; //Calculate the pulse for esc 4 (front-left - CW)
    if(esc_1 > 1800){
      esc_1 = 1800;
    }else if(esc_1 < 1150){
      esc_1 = 1150;
    }
    if(esc_2 > 1800){
      esc_2 = 1800;
    }else if(esc_2 < 1150){
      esc_2 = 1150;
    }
    if(esc_3 > 1800){
      esc_3 = 1800;
    }else if(esc_3 < 1150){
      esc_3 = 1150;
    }
    if(esc_4 > 1800){
      esc_4 = 1800;
    }else if(esc_4 < 1150){
      esc_4 = 1150;
    }
    
    
    Serial.print("esc_1\t");
    Serial.print(esc_1);
    Serial.print("esc_2\t");
    Serial.print(esc_2);
    Serial.print("esc_3\t");
    Serial.print(esc_3);
    Serial.print("esc_4\t");
    Serial.print(esc_4);
    Serial.print("throttle\t");
    Serial.println(throttle_input);
  

//    Serial.print("ypr\t");
//    Serial.print(yaw_adjust);
//    Serial.print("\t");
//    Serial.print(pitch_adjust);
//    Serial.print("\t");
//    Serial.println(roll_adjust);
        

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
    
    if(motor_on_flag and !motor_cal_flag){
      servo_FrontRight.write(esc_1); 
      servo_RearRight.write(esc_2);  
      servo_RearLeft.write(esc_3);   
      servo_FrontLeft.write(esc_4);  
    }else if(motor_cal_flag){
      servo_FrontRight.write(throttle_input); 
      servo_RearRight.write(throttle_input);  
      servo_RearLeft.write(throttle_input);   
      servo_FrontLeft.write(throttle_input);  
    }else{
      servo_FrontRight.write(1000); 
      servo_RearRight.write(1000);  
      servo_RearLeft.write(1000);   
      servo_FrontLeft.write(1000);
      Serial.print("Motors stopped!");
      delay(1000);
    }
    if(roll_input > 115){ // Stop motor calibration after turning roll input to the right?
      motor_cal_flag = false;
      Serial.print("Motor calibration finished.");
      delay(2000);
      recvData();
      delay(100); // Ugly but works (Else the motors get a increased roll signal.)
    }
        
    
}
