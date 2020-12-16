#include "VNS.h"
#include <stdio.h>
#include <stdlib.h>

  int neighborhood [7][3] = { 1, 0, 0,
                             0, 1, 0,
                             0, 0, 1,
                             1, 1, 0,
                             1, 0, 1,
                             0, 1, 1,
                             1, 1, 1};
                             
VNS::VNS() {
  this->neighborhoodRadious = 0.4;

}

Individual VNS::run( Motor *m, PID *p, TiltSensor *s) {
  generateSolution(m, p, s);
  printInTxt("Best generated solution", 0, 0);
  int k = 0;
  int i = 0;
  while(k < 7 && i < NUMBER_OF_ITERATIONS){     
    localSearch(m, p, s, i, &k);
    i++;
    Serial.print("k;");
    Serial.print(k);
    Serial.print("nr; ");
    Serial.println(neighborhoodRadious);
  }
  printInTxt("VNS done", 0, 0);
  p->setPID(solution->getKp(), solution->getKi(), solution->getKd());
  return *solution;
}

void VNS::generateSolution(Motor *m, PID *p, TiltSensor *s){
  float kp = 0.0, ki = 0.0, kd = 0.0;
  float currentScore = 0.0, bestScore = 0.0;
  this->solution = new Individual(0.0, 0.0, 0.0);
  for (int i = 0; i < NUMBER_OF_SOLUTIONS; i++) {
    getPID(&kp, &ki, &kd);
    this->solution->setPID(randomFloat(1.0, 7.0), randomFloat(0.0, 7.0), randomFloat(0.0, 7.0));
    this->solution->setScore(evaluate(m, p, s));
    currentScore = solution->getScore();
    printInTxt("Generating solution", 0, i);
    if (currentScore > bestScore){
      bestScore = currentScore;
      this->solution->setScore(currentScore);
    }else{
      this->solution->setPID(kp, ki, kd);
      this->solution->setScore(bestScore);
    }
  }
}


float VNS::evaluate(Motor *m, PID *p, TiltSensor *s) {
  m->initialPosition(2000);
  p->setPID(solution->getKp(), solution->getKi(), solution->getKd());
  int estabilizationTime = 0;
  
  while (estabilizationTime < MAX_ESTABILIZATION_TIME) {
    p->robotBalance(m, s);
    estabilizationTime++;
  }
  
  float score = (float)(abs(p->getContEstable()))/(float)((MAX_ESTABILIZATION_TIME/100.00f));
  score = 100 - score;
  score = constrain(score,0,100);

  p->setContEstable(0);
  return score;
}

void VNS::localSearch(Motor *m, PID *p, TiltSensor *s, int iter, int *k){  //first improvement
  float kp = 0.00,ki = 0.00, kd = 0.00, currentScore = 0.00, solutionScore = 0.00;
  int flag = 0, cont = 0;
  getPID(&kp, &ki, &kd);
  solutionScore = this->solution->getScore();
//  Serial.print("Kp: ");
//  Serial.print(kp);
//  Serial.print("  Ki: ");
//  Serial.print(ki);
//  Serial.print("  Kd: ");
//  Serial.print(kd);
//  Serial.print("  Score: ");
//  Serial.println(solutionScore);
  changeNeighborhood(k);
  while(cont < NUMBER_OF_SEARCHES){  
    changeNeighborhood(k);
    currentScore = evaluate(m, p, s);
    this->solution->setScore(currentScore);
    printInTxt("Local Search", iter, cont);
    if(currentScore > solutionScore) {
      printInTxt("First Update", iter, cont);
      this->solution->setScore(currentScore);
      (*k) = 0;
      this->neighborhoodRadious /= 2; 
      flag = 0;      
      break;
    }else{
      flag = 1;   
    }
    cont++;
  }
  if(flag){
    this->solution->setPID(kp, ki, kd);
    this->solution->setScore(solutionScore);
    (*k)++;
    printInTxt("Not Updated", iter, 0);
    this->neighborhoodRadious = 0.4;
  }
}

float VNS::randomFloat(float minf, float maxf) {
  float a = random(0, 99);
  float b = random(1, 99);
  float c = random(0, 9);

  float randomFloat = ((a * b) / 10) + (c / 100);

  randomFloat = minf + randomFloat * (maxf - minf) / 980.19;
  return randomFloat;
}

int VNS::randomInt(){
  int output = MIN + (rand() % static_cast<int>(MAX - MIN + 1));
  if(output % 2 == 0)
    return 1;
  else
    return -1;
}

void VNS::printInTxt(String type, int iteration, int numberLocalSearch) {
    Serial.print(type);
    Serial.print(";");
    Serial.print(iteration);
    Serial.print(";");
    Serial.print(numberLocalSearch);
    Serial.print(";");
    Serial.print(this->solution->getKp());
    Serial.print(";");
    Serial.print(this->solution->getKi());
    Serial.print(";");
    Serial.print(this->solution->getKd());
    Serial.print(";");
    Serial.print(this->solution->getScore());
    Serial.println();
}

void VNS::getPID(float *kp, float *ki, float *kd){
  *kp = this->solution->getKp();
  *ki = this->solution->getKi();
  *kd = this->solution->getKd();
}

void VNS::changeNeighborhood(int *k){
  this->solution->setKp(solution->getKp() + neighborhood[*k][0] * randomInt() * randomFloat(0,this->neighborhoodRadious));
  this->solution->setKi(solution->getKi() + neighborhood[*k][1] * randomInt() * randomFloat(0,this->neighborhoodRadious));
  this->solution->setKd(solution->getKd() + neighborhood[*k][2] * randomInt() * randomFloat(0,this->neighborhoodRadious));
}
