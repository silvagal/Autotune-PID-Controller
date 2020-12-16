#include "Motor.h"

Motor::Motor() {
}

void Motor::set(int pidGain){
  int motor1Speed = constrain(NORMAL_SPEED - pidGain, MIN_SPEED,MAX_SPEED); 
  int motor2Speed = constrain(NORMAL_SPEED + pidGain, MIN_SPEED,MAX_SPEED);
  this->motor1.writeMicroseconds(motor1Speed);
  this->motor2.writeMicroseconds(motor2Speed);
}

void Motor::startMotor(){
  this->motor1.attach(MOTOR1_PIN);
  this->motor2.attach(MOTOR2_PIN);
  delay(1000);
  this->motor1.writeMicroseconds(MIN_SIGNAL);
  this->motor2.writeMicroseconds(MIN_SIGNAL);
  delay(4000);
}

void Motor::initialPosition(int initTime){
  this->motor1.writeMicroseconds(1080);
  this->motor2.writeMicroseconds(1150);
  delay(initTime);
}
