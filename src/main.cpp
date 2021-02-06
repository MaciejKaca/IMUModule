#include <Arduino.h>
#include <../include/IMU.h> 

IMU imu;

void setup()
{
  Serial.begin(9600);
  imu.receiveInitialMessage();
}

void loop()
{
  imu.updateIMUandSend();
}