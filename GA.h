#ifndef GA_H
#define GA_H
     
#include "Individual.h"
#include "PID.h"
#include "Motor.h"
#include "Parameters.h"   

class GA{
  public:
    GA();   
    void deleteIndividual(); 
    void printIndividual(int index);
    void printScore(int index);
    void crossover(Individual *a, Individual *b, Individual *result);
    void mutate(Individual *a, Individual *m);
    float evaluate(Individual *a, Motor *m, PID *p, TiltSensor *s);
    Individual run( Motor *m, PID *p, TiltSensor *s);
    void evolution( Motor *m, PID *p, TiltSensor *s);
    float randomDouble(float minf, float maxf);
    int torneio(int minId, int maxId);
  private:
    Individual *vetIndividual[NUMBER_OF_INDIVIDUALS * 2];
    Individual *swap;      
    float mutateProbability;
    float crossoverProbability; 
};

#endif
