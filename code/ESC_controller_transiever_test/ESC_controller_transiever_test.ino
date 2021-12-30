/* Test code for the Radio control transmitter
 *  Install the NRF24 library to your IDE
 * Upload this code to the Arduino UNO
 * Connect a NRF24 module to it:
 
    Module // Arduino UNO
    
    GND    ->   GND
    Vcc    ->   3.3V
    CE     ->   D9
    CSN    ->   D10
    CLK    ->   D13
    MOSI   ->   D11
    MISO   ->   D12
*/

// Servo setup
#include <Servo.h>
int throttle_input, yaw_input, pitch_input, roll_input;

byte servoPin_FrontRight = 3;        // ESC Signal pin
byte servoPin_RearRight = 4;    // ESC Signal pin
byte servoPin_RearLeft = 5;      // ESC Signal pin
byte servoPin_FrontLeft = 6;        // ESC Signal pin
Servo servo_FrontLeft, servo_RearRight, servo_FrontRight, servo_RearLeft;


// Reciever setup (reciever.ino)
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






/**************************************************/

void setup()
{
  servo_FrontRight.attach(servoPin_FrontRight);
  servo_RearRight.attach(servoPin_RearRight);
  servo_RearLeft.attach(servoPin_RearLeft);
  servo_FrontLeft.attach(servoPin_FrontLeft);
  
  Serial.begin(9600); //Set the speed to 9600 bauds if you want.
                        //You should always have the same speed selected in the serial monitor
  resetData();
  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  
  radio.openReadingPipe(1,pipeIn);
  //we start the radio comunication
  radio.startListening();

}

/**************************************************/



/**************************************************/

void loop()
{
  recvData();
  unsigned long now = millis();
  //Here we check if we've lost signal, if we did we reset the values 
  if ( now - lastRecvTime > 1000 ) {
  // Signal lost?
  resetData();
  }
  
  throttle_input = map(data.throttle, 127, 255, 1000, 1900);
  yaw_input      = data.yaw;
  pitch_input    = data.pitch;
  roll_input     = data.roll;


  servo_FrontRight.write(throttle_input); // Direct write to esc
  servo_RearRight.write(throttle_input); // Direct write to esc
  servo_RearLeft.write(throttle_input); // Direct write to esc
  servo_FrontLeft.write(throttle_input); // Direct write to esc
  //value = analogRead(PIN_POTI);
  //if(Serial.available() > 1){
    //value = Serial.parseInt();
  //}
  
  //Serial.print("Analog-Val: ");Serial.println(analog_val);
  Serial.print("Throttle_input: ");Serial.print(data.throttle);Serial.print(", ESC_input");Serial.println(throttle_input);


}

/**************************************************/
