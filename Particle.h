#ifndef Particle_H
#define Particle_H   

class Particle {
  public:
    Particle(float p, float i, float d);
    
    void setCurrentKp(float p);
    void setCurrentKd(float d);
    void setCurrentKi(float i);
    void setPbestKp(float p);
    void setPbestKd(float d);
    void setPbestKi(float i);    
    void setPID(float p, float i, float d);
    void setPbestPID(float p, float i, float d);    
    void setPbestFitness(float pb);
    void setPreviousVelocityKp(float vel);
    void setPreviousVelocityKi(float vel);
    void setPreviousVelocityKd(float vel);
    void setCurrentFitness(float s);
    void setAllVelocity(float p, float i, float d);

    float getPbestFitness();
    float getCurrentKp();
    float getCurrentKd();
    float getCurrentKi();
    float getPbestKp();
    float getPbestKd();
    float getPbestKi();    
    float getPreviousVelocityKp();
    float getPreviousVelocityKi();
    float getPreviousVelocityKd();
    float getCurrentFitness();

  private:
    float kp;
    float ki;
    float kd;
    float best_kp;
    float best_ki;
    float best_kd;
    float pBest_Fitness;
    float pVelp;
    float pVeli;
    float pVeld;
    float Fitness;
};

#endif
