#ifndef Motor_h
#define Motor_h

#include <Arduino.h>
#include <Servo.h>
#include "Parameters.h" 

class Motor{
  public:
    Motor();
    void startMotor();
    void set(int pidGain);
    void initialPosition(int initTime);
  
  private:
    Servo motor1;
    Servo motor2;
};

#endif
