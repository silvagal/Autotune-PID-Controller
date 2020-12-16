#ifndef VNS_h
#define VNS_h

#include "Individual.h"
#include "Parameters.h" 
#include "PID.h"
#include "Motor.h" 

class VNS{
  public:
    VNS();   
    Individual run(Motor *m, PID *p, TiltSensor *s);
    void localSearch(Motor *m, PID *p, TiltSensor *s, int iter, int *k);
    int randomInt();
    float evaluate(Motor *m, PID *p, TiltSensor *s);
    float randomFloat(float minf, float maxf);
    void generateSolution(Motor *m, PID *p, TiltSensor *s);
    void getPID(float *kp, float *ki, float *kd);
    void printInTxt(String type, int iteration, int numberLocalSearch);
    void changeNeighborhood(int *k);
    
  private:
    Individual *solution;
    float neighborhoodRadious;
};

#endif
