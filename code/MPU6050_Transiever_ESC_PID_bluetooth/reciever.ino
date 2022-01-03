
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
