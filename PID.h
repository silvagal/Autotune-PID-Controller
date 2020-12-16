#ifndef PID_h
#define PID_h

#include <Arduino.h>
#include "Motor.h"
#include "TiltSensor.h"



class PID{
  public:
    PID();
    void setPID(float p, float i, float d);
    void setContEstable(int c);    
    void robotBalance(Motor *m, TiltSensor *s);
    int control(float angle);   
    float getContEstable();
    
  private:
    float kp;
    float ki;
    float kd;
    float contEstable;
    float lastError;
};

#endif
