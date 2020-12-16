#ifndef PSO_H
#define PSO_H
    
#include "Particle.h"
#include "Parameters.h"  
#include "PID.h"
#include "Motor.h" 

#define min 0
#define max 10

class PSO{
  public:
    PSO();         
    float evaluate(Particle *a,  Motor *m, PID *p, TiltSensor *s);
    float randomDouble(float minf, float maxf);
    void pBest();
    void getVelocity(int gBest_index, Particle *a);
    void updateParticles(int gBest_index, Motor *m, PID *p, TiltSensor *s, int cont);
    void printInTxt(Particle *solution2, String type, int iteration, int numberLocalSearch);
    void printInTxtGbest(Particle *solution2, String type, int iteration, int numberLocalSearch);
    void run( Motor *m, PID *p, TiltSensor *s); 
    void deleteParticle();
    int gBest();
    int worstParticle();
    int randomInt();
    
  private:
    int gBest_index;
    float velp, veli, veld;
    float ponderacao;
    Particle *vetParticle[NUMBER_OF_PARTICLES*2];  
    Particle *swap;    
};

#endif
