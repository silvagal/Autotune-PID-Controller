#include "Individual.h"
#include <Arduino.h>
Individual::Individual(float p, float i, float d) {
  setPID(p, i, d);
}

void Individual::setKp(float p) {
  if (p >= 0.3 && p <= 7.0) {
    kp = p;
  }else if(p > 7.0){
    kp = 7.0;
  }else if(p < 0.30){
    kp = 0.30;
  }
}

void Individual::setKi(float i) {
  if (i>=0 && i<=7.0) {
    ki = i;
  }else if(i>7.0){
    ki = 7.0;
  }else if(i<0.0){
    ki =0.0;
  }
}

void Individual::setKd(float d) {
  if (d>=0 && d<=7.0) {
    kd = d;
  }else if(d>7.0){
    kd = 7.0;
  }else if(d<0.0){
    kd =0.0;
  }
}

void Individual::setPID(float p, float i, float d) {
  setKp(p);
  setKi(i);
  setKd(d);
}

void Individual::setScore(float s) {
  score = s;
}


float const Individual::getScore() {
  return score;
}

float const Individual::getKp() {
  return kp;
}

float const Individual::getKi() {
  return ki;
}

float const Individual::getKd() {
  return kd;
}
