#include "PID.h"

PID::PID() {
  this->kp = 1;
  this->ki = 0;
  this->kd = 0;
  this->contEstable = 0;
  this->lastError = 0.0;
}

int PID::control(float angle) {
  float pTerm = 0.0, iTerm = 0.0, dTerm = 0.0;
  float error = 0.0;
  int pid = 0;
  error = SETPOINT - angle;
  pTerm = this->kp * error;
  iTerm += this->ki * error;
//  iTerm = constrain(iTerm, -10, 10);
  if(error > -5 && error < 5)
    iTerm += abs(this->ki * error);
  dTerm = this->kd * (error - this->lastError);
  lastError = error;
  pid = pTerm + iTerm + dTerm;
  pid = constrain(pid, -PID_GAIN_LIMIT, PID_GAIN_LIMIT);
  return pid;
}

void PID::robotBalance(Motor *m, TiltSensor *s) {
    long unsigned temp = micros();    
    double currentAngle = s->getXAxis(temp);
    this->contEstable += abs((SETPOINT - currentAngle) / (MAX_ANGLE - SETPOINT));
//    Serial.print(currentAngle);
//    Serial.println();
    int speed = control(currentAngle);
    m->set(speed);    
}

void PID::setPID(float p, float i, float d) {
  this->kp = p;
  this->ki = i;
  this->kd = d;
}

void PID::setContEstable(int c) {
  this->contEstable = c;
}

float PID::getContEstable() {
  return this->contEstable;
}
