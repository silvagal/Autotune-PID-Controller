#include "TiltSensor.h"
#include "PID.h"
#include "Motor.h"
#include "Parameters.h"
#include "VNS.h"
#include "GA.h"
#include "PSO.h"

void printInfo();
float getRandomSeed();

TiltSensor sensor;
PID pid;
Motor m;
VNS *vns;
GA *ga;
PSO *pso;

void setup() {
  Serial.begin(BAUDRATE);   
  sensor.begin();
  m.startMotor();
}

void loop() {
  for(int i = 0; i < 5; i++){
    int randomseed = getRandomSeed();
    printInfo();    
    vns = new VNS();
    vns -> run(&m,&pid,&sensor);
  }

  for(int i = 0; i < 5; i++){
    int randomseed = getRandomSeed();  
    ga = new GA(); 
    ga->run(&m, &pid, &sensor);
    ga->deleteIndividual();
  }  

  for(int i = 0; i < 5; i++){
    int randomseed = getRandomSeed();
    pso = new PSO();
    pso->run(&m, &pid, &sensor);
    pso->deleteParticle();
  }
  
//  while (true) {
//    pid.robotBalance(&m, &sensor, &pid);
//  }
}

void printInfo(){
    Serial.print("Estado");
    Serial.print(";");
    Serial.print("Iteration");
    Serial.print(";");
    Serial.print("Iteration of local search");
    Serial.print(";");
    Serial.print("Kp");
    Serial.print(";");
    Serial.print("Ki");
    Serial.print(";");
    Serial.print("Kd");
    Serial.print(";");
    Serial.print("Score");
    Serial.println();
}

float getRandomSeed(){
    int randomseed = analogRead(A0);
    randomSeed(randomseed);
    Serial.print("\n\nRandom seed");
    Serial.print(";");
    Serial.println(randomseed);
    Serial.print("\n\n");
    return randomseed;
}
