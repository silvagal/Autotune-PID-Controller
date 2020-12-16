#ifndef TiltSensor_h
#define TiltSensor_h
#include <Arduino.h>
#include "Kalman.h"
#include "I2C.h"

class TiltSensor{
public:
  TiltSensor();
  double getXAxis(long unsigned &temp);
  void begin();

private:
  int16_t accX, accY, accZ;
  int16_t tempRaw;
  int16_t gyroX, gyroY, gyroZ;

  float accXangle; //, accYangle; // Angle calculate using the accelerometer
  float gyroXangle;//, gyroYangle; // Angle calculate using the gyro
  float kalAngleX; //, kalAngleY; // Calculate the angle using a Kalman filter

  uint8_t i2cData[14]; 
  double gyroXrate;
  Kalman kalmanX; 
};

#endif
