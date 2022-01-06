
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

bool pitch_roll_inverted = true;

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
unsigned long loop_timer, now, period, difference, temp_time;


// ================================================================
// ===                     Reciever - SETUP                     ===
// ================================================================
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
//#include "receiver.h"
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
MyData receiver_data;


// ================================================================
// ===                     Bluetooth - SETUP                    ===
// ================================================================
//#include <SoftwareSerial.h>

//SoftwareSerial mySerial(0, 1); // RX, TX

//int ledpin=13;

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


// ================================================================
// ===               PID Control Variables                      ===
// ================================================================
// Variables for PID control (have to be tuned)
double yaw_error, pitch_error, roll_error; 
double yaw_adjust, pitch_adjust, roll_adjust;
int last_time, current_time, time_delta;
//float Kp_YPR[] = {2,2,2};
//float Ki_YPR[] = {0.001,0.001,0.001};
//float Kd_YPR[] = {10,10,10};

float Kpid_Y[] = {0, 0, 0};
float Kpid_P[] = {0, 0, 0};
float Kpid_R[] = {0, 0, 0};

float last_reading[3];
float yaw_PID[3], pitch_PID[3], roll_PID[3];
float integrated_error_yaw;
float integrated_error_pitch;
float integrated_error_roll;
float esc_1, esc_2, esc_3, esc_4;
float throttle;
int AUX_2_counter;


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // Set ESC pins
    servo_FrontRight.attach(servoPin_FrontRight);
    servo_RearRight.attach(servoPin_RearRight);
    servo_RearLeft.attach(servoPin_RearLeft);
    servo_FrontLeft.attach(servoPin_FrontLeft);

    esc_1 = 1000;
    esc_2 = 1000;
    esc_3 = 1000;
    esc_4 = 1000;
    
    servo_FrontRight.writeMicroseconds(esc_1); 
    servo_RearRight.writeMicroseconds(esc_2);  
    servo_RearLeft.writeMicroseconds(esc_3);   
    servo_FrontLeft.writeMicroseconds(esc_4);

    // Set Reciever
    resetData(receiver_data);
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

   
    // initialize bluetooth serial communication
    Serial.begin(230400);
    Serial.println("Hallo vom Bluetooth Modul");

    
    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    //pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    //while (Serial.available() && Serial.read()); // empty buffer
    //while (!Serial.available());                 // wait for data
    //Serial.println(F("\nTurn pitch all the way up: "));
    recvData(receiver_data);
    while (!(receiver_data.pitch > 230)){
      recvData(receiver_data);// wait for Max pitch signal
    }
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


    period = 4000;
    loop_timer = micros();
}



void read_gyro()
{
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 

      #ifdef OUTPUT_READABLE_YAWPITCHROLL
          // display Euler angles in degrees
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      #endif
  }
}

void loop() {
    unsigned long now = millis();
    // Recieve from nRF24L01
    recvData(receiver_data);
    
    //Here we check if we've lost signal, if we did we reset the values 
    if ( now - lastRecvTime > 1000 ) {
    // Signal lost?
      resetData(receiver_data);
      Serial.println("Lost Transmitter Signal");
    }
    throttle_input = map(receiver_data.throttle, 127, 255, 1150, 1600);
    yaw_input      = map(receiver_data.yaw, 0, 255, -127, 127);
    pitch_input    = map(receiver_data.pitch, 0, 255, -127, 127);
    roll_input     = -map(receiver_data.roll, 0, 255, -127, 127); // Roll input inverted

    if(pitch_input > 115 and throttle_input < 900){ // Turn of motor if pitch all the way up and throttle all the way down
      motor_on_flag = false;
    }
    

    if(receiver_data.AUX2 == 1){
      motor_on_flag = false;
    }
    if(receiver_data.AUX1 == 1){
      motor_on_flag = true;
    }
    
    
    // GYRO
    read_gyro();

    // If quadcopter is > +/- 30° on either pitch or role axis stop motors
    if(abs(ypr[1] * 180/M_PI) > 45 or abs(ypr[2] * 180/M_PI) > 45){
      motor_on_flag = false;
      Serial.println("Too steep angle, motors stopped.");
    }
    
    // PID control    
    current_time = millis();
    time_delta   = current_time - last_time;
    last_time = current_time;
    yaw_error = yaw_input - ypr[0] * 180/M_PI;

    if(!pitch_roll_inverted){
      pitch_error = pitch_input - ypr[1] * 180/M_PI;
      roll_error = roll_input - ypr[2] * 180/M_PI;
    }else{
      pitch_error = pitch_input - ypr[2] * 180/M_PI;
      roll_error = roll_input - ypr[1] * 180/M_PI;
    }

    // Calc each P,I,D-value for yaw, pitch, roll
    yaw_adjust = calc_pid(Kpid_Y, yaw_error, integrated_error_yaw, last_reading[0], time_delta);
    pitch_adjust = calc_pid(Kpid_P, pitch_error, integrated_error_pitch, last_reading[1], time_delta);
    roll_adjust = calc_pid(Kpid_R, roll_error, integrated_error_roll, last_reading[2], time_delta);  
    
        
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

    
    esc_1 = throttle - pitch_adjust - roll_adjust - yaw_adjust; //Calculate the pulse for esc 1 (front-right - CCW)
    esc_2 = throttle + pitch_adjust - roll_adjust + yaw_adjust; //Calculate the pulse for esc 2 (rear-right - CW)
    esc_3 = throttle + pitch_adjust + roll_adjust - yaw_adjust; //Calculate the pulse for esc 3 (rear-left - CCW)
    esc_4 = throttle - pitch_adjust + roll_adjust + yaw_adjust; //Calculate the pulse for esc 4 (front-left - CW)
    if(esc_1 > 1800){
      esc_1 = 1800;
    }else if(esc_1 < 1050){
      esc_1 = 1050;
    }
    if(esc_2 > 1800){
      esc_2 = 1800;
    }else if(esc_2 < 1050){
      esc_2 = 1050;
    }
    if(esc_3 > 1800){
      esc_3 = 1800;
    }else if(esc_3 < 1050){
      esc_3 = 1050;
    }
    if(esc_4 > 1800){
      esc_4 = 1800;
    }else if(esc_4 < 1050){
      esc_4 = 1050;
    }

           

    
    
    if(motor_on_flag){
//      esc_1 = 1030;
//      esc_2 = 1030;
//      esc_3 = 1030;
//      esc_4 = 1030;
      servo_FrontRight.writeMicroseconds(esc_1); 
      servo_RearRight.writeMicroseconds(esc_2);  
      servo_RearLeft.writeMicroseconds(esc_3);   
      servo_FrontLeft.writeMicroseconds(esc_4);

      // blink LED to indicate activity
      blinkState = !blinkState;
      digitalWrite(LED_PIN, blinkState);
    
      
      //applyMotorSpeed();  
    }else{
      esc_1 = 1000;
      esc_2 = 1000;
      esc_3 = 1000;
      esc_4 = 1000;
      
      servo_FrontRight.writeMicroseconds(esc_1); 
      servo_RearRight.writeMicroseconds(esc_2);  
      servo_RearLeft.writeMicroseconds(esc_3);   
      servo_FrontLeft.writeMicroseconds(esc_4);
      //applyMotorSpeed();  
      Serial.print("Motors stopped!");
    }
    
    print_info();

    temp_time = micros();
    difference = temp_time - loop_timer;
    loop_timer = temp_time;

   
        
    //Serial.print("Loop Time: ");
    //Serial.println(difference);
}


void print_info(){
    Serial.print("FR\t");
    Serial.print(esc_1);
    Serial.print("RR\t");
    Serial.print(esc_2);
    Serial.print("RL\t");
    Serial.print(esc_3);
    Serial.print("FL\t");
    Serial.print(esc_4);
    Serial.print("throttle\t");
    Serial.println(throttle_input);
}

/*
void applyMotorSpeed() {
    // Refresh rate is 250Hz: send ESC pulses every 4000µs
    while ((now = micros()) - loop_timer < period);

    // Update loop timer
    loop_timer = now;

    // Set pins #3 #4 #5 #6 HIGH
    PORTD |= B01111000;

    // Wait until all pins #4 #5 #6 #7 are LOW
    while (PORTD >= 8) {
        now        = micros();
        difference = now - loop_timer;

        if (difference >= esc_1) PORTD &= B11110111; // Set pin #3 LOW
        if (difference >= esc_2) PORTD &= B11101111; // Set pin #4 LOW
        if (difference >= esc_3) PORTD &= B11011111; // Set pin #5 LOW
        if (difference >= esc_4) PORTD &= B10111111; // Set pin #6 LOW
    }
}
*/
