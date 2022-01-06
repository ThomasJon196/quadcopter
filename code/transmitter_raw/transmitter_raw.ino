#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

/*
The receiver needs the same unique code
*/
  
const uint64_t pipeOut = 0xE8E8F0F0E1LL; //IMPORTANT: The same as in the receiver

RF24 radio(9, 10); // select  CSN  pin

// The sizeof this struct should not exceed 32 bytes
// This gives us up to 32 8 bits channals
struct MyData {
  byte throttle;
  byte yaw;
  byte roll;
  byte pitch;   
  byte AUX1;
  byte AUX2;
};

MyData data;

void init_Data() 
{
  // Initial values
    
  data.throttle = 0;
  data.yaw = 127;
  data.pitch = 127;
  data.roll = 127;
  data.AUX1 = 0;
  data.AUX2 = 0;
}

void setup()
{
  //Start everything up
  Serial.begin(9600);
  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(pipeOut);
  init_Data();
}

/**************************************************/

// Returns a corrected value for a joystick position that takes into account
// the values of the outer extents and the middle of the joystick range.
int mapJoystickValues(int val, int lower, int middle, int upper, bool reverse)
{
  val = constrain(val, lower, upper);
  if ( val < middle )
    val = map(val, lower, middle, 0, 128);
  else
    val = map(val, middle, upper, 128, 255);
  return ( reverse ? 255 - val : val );
  return val;
}

void loop()
{
  // Min,Middle,Max,(invert) for each joystick.
  // Measure the real values with print(analogRead()). Differ for each joystick.
  data.throttle = mapJoystickValues( analogRead(A3), 1, 524, 1018, false );
  data.yaw      = mapJoystickValues( analogRead(A2),  8, 505, 1018, false );
  data.pitch    = mapJoystickValues( analogRead(A1), 4, 490, 1018, false );
  data.roll     = mapJoystickValues( analogRead(A0), 3, 515, 1019, false );
  data.AUX1     = digitalRead(3); 
  data.AUX2     = digitalRead(4);

//  Print analogRead(.) for calibration...
//  Serial.print("Throttle: ");
//  Serial.print(analogRead(A3));
//  Serial.print(" ");
//  Serial.print("yaw: ");
//  Serial.print(analogRead(A2));
//  Serial.print(" ");
//  Serial.print("pitch: ");
//  Serial.print(analogRead(A1));
//  Serial.print(" ");
//  Serial.print("roll: ");
//  Serial.print(analogRead(A0));
//  Serial.print(" ");
  

//   Serial.print("Throttle: ");
//   Serial.print(data.throttle);
//   Serial.print(" ");
//   Serial.print("yaw: ");
//   Serial.print(data.yaw);
//   Serial.print(" ");
//   Serial.print("pitch: ");
//   Serial.print(data.pitch);
//   Serial.print(" ");
//   Serial.print("roll: ");
//   Serial.print(data.roll);
//   Serial.print(" ");

//   Serial.print("\n");
  
  radio.write(&data, sizeof(MyData));

  
}
