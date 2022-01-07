
void recvData(MyData &receiver_data)
{
  while ( radio.available() ) {
    radio.read(&receiver_data, sizeof(MyData));
    lastRecvTime = millis(); //here we receive the data
  }
}

void resetData(MyData &receiver_data)
{
  //We define the initial value of each data input
  //Joysticks will be in the middle position so 127 is the middle from 254
  receiver_data.throttle = 0;
  receiver_data.yaw = 127;
  receiver_data.pitch = 127;
  receiver_data.roll = 127;
  receiver_data.AUX1 = 0;
  receiver_data.AUX2 = 0;

}

void read_receiver(){

  // Recieve from nRF24L01
    recvData(receiver_data);
    //Here we check if we've lost signal, if we did we reset the values 
    if ( now - lastRecvTime > 1000 ) {
    // Signal lost?
      resetData(receiver_data);
      Serial.println("Lost Transmitter Signal");
    }

    
    
    throttle_input = map(receiver_data.throttle, 127, 255, MIN_THROTTLE, MAX_THROTTLE);
    yaw_input      = map(receiver_data.yaw, 0, 255, -127, 127);
    pitch_input    = map(receiver_data.pitch, 0, 255, -127, 127);
    roll_input     = -map(receiver_data.roll, 0, 255, -127, 127); // Roll input inverted


    if(throttle_input > MAX_THROTTLE){
      throttle = MAX_THROTTLE;
    }
    if(throttle_input < MIN_THROTTLE){
      throttle = MIN_THROTTLE;
    }
}
