#include "Particle.h"

Particle::Particle(float p, float i, float d) {
  setPID(p, i, d);
}

void Particle::setCurrentKp(float p) {
  if (p >= 0.0 && p <= 7.0) {
    kp = p;
  }else if(p > 7.0){
    kp = 7.0;
  }else if(p < 0.0){
    kp = 0.0;
  }
}

void Particle::setCurrentKi(float i) {
  if (i>=0 && i<=7.0) {
    ki = i;
  }else if(i>7.0){
    ki = 7.0;
  }else if(i<0.0){
    ki =0.0;
  }
}

void Particle::setCurrentKd(float d) {
  if (d>=0 && d<=7.0) {
    kd = d;
  }else if(d>7.0){
    kd = 7.0;
  }else if(d<0.0){
    kd =0.0;
  }
}

void Particle::setPbestKp(float p) {
  if (p >= 0.0 && p <= 7.0) {
    best_kp = p;
  }else if(p > 7.0){
    best_kp = 7.0;
  }else if(p < 0.0){
    best_kp = 0.0;
  }
}

void Particle::setPbestKi(float i) {
  if (i>=0 && i<=7.0) {
    best_ki = i;
  }else if(i>7.0){
    best_ki = 7.0;
  }else if(i<0.0){
    best_ki =0.0;
  }
}

void Particle::setPbestKd(float d) {
  if (d>=0 && d<=7.0) {
    best_kd = d;
  }else if(d>7.0){
    best_kd = 7.0;
  }else if(d<0.0){
    best_kd =0.0;
  }
}

void Particle::setPID(float p, float i, float d) {
  setCurrentKp(p);
  setCurrentKi(i);
  setCurrentKd(d);
}

void Particle::setPbestPID(float p, float i, float d) {
  setPbestKp(p);
  setPbestKi(i);
  setPbestKd(d);
}

void Particle::setPreviousVelocityKp(float vel){
  pVelp = vel;
}

void Particle::setPreviousVelocityKi(float vel){
  pVeli = vel;
}

void Particle::setPreviousVelocityKd(float vel){
  pVeld = vel;
}

void Particle::setCurrentFitness(float s){
  Fitness = s;
}

void Particle::setAllVelocity(float p, float i, float d) {
  setPreviousVelocityKp(p);
  setPreviousVelocityKi(i);
  setPreviousVelocityKd(d);
}

void Particle::setPbestFitness(float pb) {
  pBest_Fitness = pb;
}

float Particle::getPbestFitness() {
  return pBest_Fitness;
}

float Particle::getCurrentKp() {
  return kp;
}

float Particle::getCurrentKi() {
  return ki;
}

float Particle::getCurrentKd() {
  return kd;
}

float Particle::getPbestKp() {
  return best_kp;
}

float Particle::getPbestKi() {
  return best_ki;
}

float Particle::getPbestKd() {
  return best_kd;
}


float Particle::getPreviousVelocityKp(){
  return pVelp;
}

float Particle::getPreviousVelocityKi(){
  return pVeli;
}

float Particle::getPreviousVelocityKd(){
  return pVeld;
}

float Particle::getCurrentFitness(){
  return Fitness;
}
