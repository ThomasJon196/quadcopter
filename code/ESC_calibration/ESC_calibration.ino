/*
 * ESC calibration : Setting up MAX/MIN signal the ESC will recieve.
 */
#include <Servo.h>

#define MAX_SIGNAL 2000
#define MIN_SIGNAL 1000
#define MOTOR_PIN_1 3
#define MOTOR_PIN_2 4
#define MOTOR_PIN_3 5
#define MOTOR_PIN_4 6

Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

int CURRENT_SIGNAL = 1000;

void setup() {
  
  Serial.begin(9600);
  delay(1500);
  Serial.println("ESC calibration routine initializes...");
  
  motor1.attach(MOTOR_PIN_1);
  motor2.attach(MOTOR_PIN_2);
  motor3.attach(MOTOR_PIN_3);
  motor4.attach(MOTOR_PIN_4);

  delay(1000);
  Serial.println("ESC's attached.");


  Serial.print("Sending maximum output: (");
  Serial.print(MAX_SIGNAL);
  Serial.print(" micro_second pulse)");
  Serial.print("\n");
  Serial.println("Apply power source to ESC's and wait 2 seconds before pressing Enter");

  // Send MAX_SIGNAL output
  motor1.writeMicroseconds(MAX_SIGNAL);
  motor2.writeMicroseconds(MAX_SIGNAL);
  motor3.writeMicroseconds(MAX_SIGNAL);
  motor4.writeMicroseconds(MAX_SIGNAL);


  // Wait for input
  while (!Serial.available());
  Serial.read();

  // Send MIN_SIGNAL output
  Serial.println("\n");
  Serial.println("\n");
  Serial.print("Sending minimum output: (");
  Serial.print(MIN_SIGNAL);
  Serial.print(" micro-second pulse)");
  Serial.print("\n");
  motor1.writeMicroseconds(MIN_SIGNAL);
  motor2.writeMicroseconds(MIN_SIGNAL);
  motor3.writeMicroseconds(MIN_SIGNAL);
  motor4.writeMicroseconds(MIN_SIGNAL);
  Serial.println("The ESC is calibrated");
  Serial.println("---------------------");

  
  Serial.println("You can now enter values between 1000 and 2000");
  Serial.println("to adjust the motor speed");
  Serial.println("Enter 1000 to turn off the motors");
}

void loop() {
   
  if (Serial.available() > 0)
  {
    int CURRENT_SIGNAL = Serial.parseInt();
    if (CURRENT_SIGNAL > 999 and CURRENT_SIGNAL < 2001)
    {
      motor1.writeMicroseconds(CURRENT_SIGNAL);
      motor2.writeMicroseconds(CURRENT_SIGNAL);
      motor3.writeMicroseconds(CURRENT_SIGNAL);
      motor4.writeMicroseconds(CURRENT_SIGNAL);
    }else{
      Serial.print("Signal must be between [1000, 2000]");  
    }
  }
}
 
