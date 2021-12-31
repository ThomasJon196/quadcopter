// HC-05 Bluetooth Transmitter
// Sending data to another bluetooth device.


#include <SoftwareSerial.h>

// Other pins should be used if RX,TX connections cant be removed when uploading new code.
// Have to be done because RX, TX are used as Serial communication when uploading new code.
SoftwareSerial mySerial(0, 1); // RX, TX 
//int ledpin=13;


void setup() {
  mySerial.begin(9600);
  mySerial.println("Hello I am another bluetooth module.");
  pinMode(ledpin,OUTPUT);
}

void loop() {
    // Simple counter
    digitalWrite(ledpin, 1);
    mySerial.print(i);
    mySerial.println(" von Bluetooth");
    digitalWrite(ledpin, 0);  
    i++;
    delay(1000);
  
}
