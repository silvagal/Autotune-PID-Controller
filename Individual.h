#ifndef INDIVIDUAL_H
#define INDIVIDUAL_H   

class Individual {
  public:
    Individual(float p, float i, float d);
    
    void setKp(float p);
    void setKd(float d);
    void setKi(float i);
    void setPID(float p, float i, float d);
    void setScore(float s);

    float const getScore();
    float const getKp();
    float const getKd();
    float const getKi();

  private:
    float kp;
    float ki;
    float kd;
    float score;
};

#endif
