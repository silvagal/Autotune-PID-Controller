#include "PSO.h"

   int cont = 0;                             
//   int geracao=0, individuo=0;
//   float kpp, kii, kdd, score;   
                             
PSO::PSO() {
  gBest_index = 0;
  velp = 0.0; veli = 0.0; veld = 0.0;
  for (int i = 0; i < NUMBER_OF_PARTICLES; i++) {                                                                               /////////////////////////////////////////
    vetParticle[i] = new Particle(randomDouble(0.00, 7.00), randomDouble(0.00, 7.00), randomDouble(0.00, 7.00));                //                                     //
    vetParticle[i]->setPbestPID(vetParticle[i]->getCurrentKp(),vetParticle[i]->getCurrentKi(), vetParticle[i]->getCurrentKd()); ////cria e inicializa as partículas    // 
    vetParticle[i]->setAllVelocity(0,0,0);                                                                                      //                                     //      
    vetParticle[i]->setPbestFitness(0);                                                                                         /////////////////////////////////////////  
  }
  Particle *swap = NULL;
}

void PSO::run( Motor *m, PID *p, TiltSensor *s) {
  for(int j = 0; j <  NUMBER_OF_PARTICLES ; j++){                                                                                                                                                                                                                                                                                //                                  //
    vetParticle[j]->setCurrentFitness(evaluate(vetParticle[j],m,p,s));
    printInTxt(vetParticle[j], "PSO", 0,j);                                                                                                                                            
  }
  for(int i = 0; i < NUMBER_OF_GENERATIONS; i++){
    double last = millis();                        //Inicializa a contagem para verificar o tempo total de execução de uma geração                        
    pBest();  
    gBest_index = gBest(); 
    printInTxtGbest(vetParticle[gBest_index], "Gbest", i+1,0);                                   
    ponderacao = MAX_INERTIA - i*((MAX_INERTIA - MIN_INERTIA)/NUMBER_OF_GENERATIONS); //formula de ponderação, retirada do artigo do Eberhart
    updateParticles(gBest_index, m, p, s, i+1);                   
    float execution_time = (millis() - last)/1000; 
    cont++;
}
    
    p->setPID(vetParticle[gBest_index]->getPbestKp(), vetParticle[gBest_index]->getPbestKi(), 
                                                        vetParticle[gBest_index]->getPbestKd());
    printInTxtGbest(vetParticle[gBest_index], "PSO-VNS done", 0, 0);                                                    
}

int PSO::worstParticle(){
    int loser = 0;
    for(int i = 0; i < NUMBER_OF_PARTICLES; i++){
      if(i != loser)            
        if(vetParticle[i]->getCurrentFitness() < vetParticle[loser]->getCurrentFitness())
           loser = i;                
    } 
    return loser;  
}

float PSO::evaluate(Particle *a, Motor *m, PID *p, TiltSensor *s) {
  m->initialPosition(2000);
  p->setPID(a->getCurrentKp(), a->getCurrentKi(), a->getCurrentKd());
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

int PSO::gBest(){
    int winner = 0;
    for(int i = 1; i < NUMBER_OF_PARTICLES; i++){       
      if(i != winner && vetParticle[i]->getPbestFitness() > vetParticle[winner]->getPbestFitness())
         winner = i;                
    } 
    return winner;
}

void PSO::pBest(){
  for(int i = 0; i < NUMBER_OF_PARTICLES; i++){
    if(vetParticle[i]->getCurrentFitness() > vetParticle[i]->getPbestFitness()){        
        vetParticle[i]->setPbestPID(vetParticle[i]->getCurrentKp(), vetParticle[i]->getCurrentKi(), vetParticle[i]->getCurrentKd());  
        vetParticle[i]->setPbestFitness(vetParticle[i]->getCurrentFitness());              
    }
  }  
}

void PSO::getVelocity(int gBest_index, Particle *a){

    velp = a->getPreviousVelocityKp()*ponderacao + 2*randomDouble(0,1)*    //////////////////////////////////////////////////////////////
          (a->getPbestKp() - a->getCurrentKp()) + 2*randomDouble(0,1)*     //                                                            //
          (vetParticle[gBest_index]->getPbestKp()- a->getCurrentKp());   //                                                            //
                                                                           //                                                            //
    veli = a->getPreviousVelocityKi()*ponderacao + 2*randomDouble(0,1)*    //                                                            //    
          (a->getPbestKi() - a->getCurrentKi()) + 2*randomDouble(0,1)*     //  //equação de velocidade extraída do artigo do Eberhart    //
          (vetParticle[gBest_index]->getPbestKi()- a->getCurrentKi());   //                                                            //                     
                                                                           //                                                            //
    veld = a->getPreviousVelocityKd()*ponderacao + 2*randomDouble(0,1)*    //                                                            //
          (a->getPbestKd() - a->getCurrentKd()) + 2*randomDouble(0,1)*     //                                                            //
          (vetParticle[gBest_index]->getPbestKd()- a->getCurrentKd());   //////////////////////////////////////////////////////////////

    
    velp = constrain(velp, MIN_VELOCITY, MAX_VELOCITY);             //////////////////////////////////////////
    veli = constrain(veli, MIN_VELOCITY, MAX_VELOCITY);             // //limita a velocidade das partículas //
    veld = constrain(veld, MIN_VELOCITY, MAX_VELOCITY);             //////////////////////////////////////////

    a->setAllVelocity(velp, veli, veld);                     
}

void PSO::updateParticles(int gBest_index, Motor *m, PID *p, TiltSensor *s, int cont){
    for(int i = 0; i < NUMBER_OF_PARTICLES; i++){
          getVelocity(gBest_index, vetParticle[i]);                                      
          vetParticle[i]->setCurrentKp(vetParticle[i]->getCurrentKp() + velp);          
          vetParticle[i]->setCurrentKi(vetParticle[i]->getCurrentKi() + veli);           
          vetParticle[i]->setCurrentKd(vetParticle[i]->getCurrentKd() + veld);                                                                      
          vetParticle[i]->setCurrentFitness(evaluate(vetParticle[i],m,p,s)); 
          printInTxt(vetParticle[i], "Updating particles", cont, i);
          
          vetParticle[i]->setPbestPID(vetParticle[i]->getPbestKp(),vetParticle[i]->getPbestKi(), vetParticle[i]->getPbestKd());                               
          vetParticle[i]->setAllVelocity(vetParticle[i]->getPreviousVelocityKp(), vetParticle[i]->getPreviousVelocityKi(),vetParticle[i]->getPreviousVelocityKd());                                                                                                                                                                                                                       
          vetParticle[i]->setPbestFitness(vetParticle[i]->getPbestFitness());                                                                                         
    }
}              

float PSO::randomDouble(float minf, float maxf) {
  float a = random(0, 99);
  float b = random(1, 99);
  float c = random(0, 9);

  float randomFloat = ((a * b) / 10) + (c / 100);

  randomFloat = minf + randomFloat * (maxf - minf) / 980.19;
  return randomFloat;
}

void PSO::printInTxt(Particle *solution2, String type, int iteration, int numberLocalSearch) {
    Serial.print(type);
    Serial.print(";");
    Serial.print(iteration);
    Serial.print(";");
    Serial.print(numberLocalSearch);
    Serial.print(";");
    Serial.print(solution2->getCurrentKp());
    Serial.print(";");
    Serial.print(solution2->getCurrentKi());
    Serial.print(";");
    Serial.print(solution2->getCurrentKd());
    Serial.print(";");
    Serial.print(solution2->getCurrentFitness());
    Serial.println();
}

void PSO::printInTxtGbest(Particle *solution2, String type, int iteration, int numberLocalSearch) {
    Serial.print(type);
    Serial.print(";");
    Serial.print(iteration);
    Serial.print(";");
    Serial.print(numberLocalSearch);
    Serial.print(";");
    Serial.print(solution2->getPbestKp());
    Serial.print(";");
    Serial.print(solution2->getPbestKi());
    Serial.print(";");
    Serial.print(solution2->getPbestKd());
    Serial.print(";");
    Serial.print(solution2->getPbestFitness());
    Serial.println();
}

int PSO::randomInt(){
  int output = min + (rand() % static_cast<int>(max - min + 1));
  if(output % 2 == 0)
    return 1;
  else
    return -1;
}

void PSO::deleteParticle(){
  for(int i = 0; i < NUMBER_OF_PARTICLES; i++)
    delete(vetParticle[i]);
}
