
void recvData()
{
  while ( radio.available() ) {
    radio.read(&data, sizeof(MyData));
    lastRecvTime = millis(); //here we receive the data
  }
}

void resetData()
{
  //We define the initial value of each data input
  //Joysticks will be in the middle position so 127 is the middle from 254
  data.throttle = 0;
  data.yaw = 127;
  data.pitch = 127;
  data.roll = 127;
  data.AUX1 = 0;
  data.AUX2 = 0;

}
