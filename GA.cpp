#include "GA.h"

GA::GA() {
  Serial.println("Random individuals: ");
  for (int i = 0; i < NUMBER_OF_INDIVIDUALS; i++) {
    vetIndividual[i] = new Individual(randomDouble(0.30, 7.00), randomDouble(0.00, 7.00), randomDouble(0.00, 7.00));
    Serial.print(i);
    Serial.print(";");
    printIndividual(i);
    Serial.println();
    //delay(1000);
  }
  for(int i = NUMBER_OF_INDIVIDUALS; i < NUMBER_OF_INDIVIDUALS*2; i++) {
    vetIndividual[i] = new Individual(0,0,0);
  }
    Individual *swap = NULL;
    Serial.println();
  }

void GA::crossover(Individual *a, Individual *b, Individual *s) {
  float crossover_factor = randomDouble(0.00, 1.00);
  
  float p = (crossover_factor * a->getKp()) + ((1 - crossover_factor) * b->getKp());
  float i = (crossover_factor * a->getKi()) + ((1 - crossover_factor) * b->getKi());
  float d = (crossover_factor * a->getKd()) + ((1 - crossover_factor) * b->getKd());
  s->setPID(p, i, d);
}

void GA::mutate(Individual *a, Individual *m) {
  float mutation = randomDouble(0.00, 1.00);
  int s = random(0, 99);
  if (s % 2 == 0) {
    m->setKp(a->getKp() + mutation);
    m->setKi(a->getKi() + mutation);
    m->setKd(a->getKd() + mutation);
  } else {
      m->setKp(a->getKp() - mutation);
      m->setKi(a->getKi() - mutation);
      m->setKd(a->getKd() - mutation);
    }
}

float GA::evaluate(Individual *a, Motor *m, PID *p, TiltSensor *s) {
  m->initialPosition(2000);
  p->setPID(a->getKp(), a->getKi(), a->getKd());
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

Individual GA::run( Motor *m, PID *p, TiltSensor *s) {
  Serial.println("#RUN#");
  for (int j = 0; j < NUMBER_OF_INDIVIDUALS; j++) {
    Serial.print(0);   //Generation
    Serial.print(";");
    Serial.print(j);   //Individual
    Serial.print(";");
    printIndividual(j);
    vetIndividual[j]->setScore(evaluate(vetIndividual[j], m, p, s));
    printScore(j);
  }
  
  for (int i = 0; i < NUMBER_OF_GENERATIONS; i++) {
    double last = millis();
    evolution(m, p, s); 
    float execution_time = (millis() - last)/1000;
    float score_medio=0.0,kp_medio=0.0,kd_medio=0.0,ki_medio=0.0;
    for(int i=0; i < NUMBER_OF_INDIVIDUALS; i++){
       score_medio+=vetIndividual[i]->getScore();
       kp_medio+=vetIndividual[i]->getKp();
       ki_medio+=vetIndividual[i]->getKi();
       kd_medio+=vetIndividual[i]->getKd();
    }
    Serial.print("Media_geração");
    Serial.print(";");
    Serial.print(kp_medio/NUMBER_OF_INDIVIDUALS,2); 
    Serial.print(";");
    Serial.print(ki_medio/NUMBER_OF_INDIVIDUALS,2); 
    Serial.print(";");
    Serial.print(kd_medio/NUMBER_OF_INDIVIDUALS,2);
    Serial.print(";");
    Serial.print(score_medio/NUMBER_OF_INDIVIDUALS,2);  
    Serial.print(";");
    Serial.print(execution_time,2);    
    Serial.println("s"); 

    // Nova geração
    for (int j = 0; j < NUMBER_OF_INDIVIDUALS; j++) {
      Serial.print(i+1);   //Generation
      Serial.print(";");
      Serial.print(j);   //Individual
      Serial.print(";");
      printIndividual(j);
      printScore(j);
    }
  }

  Serial.println("Best Individual:");
  printIndividual(0);
  printScore(0);

  p->setPID(vetIndividual[0]->getKp(), vetIndividual[0]->getKi(), vetIndividual[0]->getKd());
  return *vetIndividual[0];
}


void GA::evolution( Motor *m, PID *p, TiltSensor *s) {
  //Torneio
  //Cruzamento
  //Mutação
  //Seleção dos melhores indivíduos
  int ind = 0, outroInd = 0, probability = 0;
  for (int i = 0; i < NUMBER_OF_INDIVIDUALS; i ++) {
    // Escolher operação genética
    Serial.print("Novo individuo");
    Serial.print(";");
    Serial.print(i);
    Serial.print(";");
    do{
      ind = torneio(0, NUMBER_OF_INDIVIDUALS - 1);
      outroInd = torneio(0, NUMBER_OF_INDIVIDUALS - 1);
//      Serial.println("T/ravou");
      while(ind == outroInd)
        outroInd = torneio(0, NUMBER_OF_INDIVIDUALS - 1);
      probability = random(1, 100);   
    }while(probability > CROSSOVER_PROBABILITY * 100);  
    crossover(vetIndividual[ind], vetIndividual[outroInd], vetIndividual[NUMBER_OF_INDIVIDUALS+i]);       
    printIndividual(NUMBER_OF_INDIVIDUALS + i);
    probability = random(1, 100);
    if (probability <= MUTATION_PROBABILITY * 100)
      mutate(vetIndividual[NUMBER_OF_INDIVIDUALS+i], vetIndividual[NUMBER_OF_INDIVIDUALS+i]); 
    vetIndividual[NUMBER_OF_INDIVIDUALS + i]->setScore(evaluate(vetIndividual[NUMBER_OF_INDIVIDUALS + i], m, p, s));
    printScore(NUMBER_OF_INDIVIDUALS + i);
  }

  // # Seleção #
  // Ordena o vetor de individuos pelo Score maior para o menor (Bubble Sort) <- Substituir por um melhor depois
  for (int i = 0 ; i < (TAM - 1); i++) {
    for (int j = 0 ; j < (TAM - (i + 1)); j++) {
      if ((vetIndividual[j]->getScore()) < (vetIndividual[j + 1]->getScore())) {
        swap = vetIndividual[j];
        vetIndividual[j] = vetIndividual[j + 1];
        vetIndividual[j + 1] = swap;
      }
    }
  }
  swap = NULL;
  
//  Serial.print("\n\n");
//  Serial.print(NUMBER_OF_INDIVIDUALS);
//  Serial.println(" MELHORES INDIVÍDUOS: ");
//  for (int i = 0 ; i < NUMBER_OF_INDIVIDUALS; i++) {
//    Serial.print(i);
//    Serial.print("\t");
//    printIndividual(i) ;
//    printScore(i);
//  }
  for(int i = NUMBER_OF_INDIVIDUALS; i < TAM; i++){
    vetIndividual[i]->setPID(0,0,0);
    vetIndividual[i]->setScore(0);
  }
}

int GA::torneio(int minId, int maxId) {
    // Torneio 1
    int ind1Pos = random(minId, maxId);
    int ind2Pos = random(minId, maxId);
    while(ind1Pos == ind2Pos)
      ind2Pos = random(minId, maxId);
      
    // Torneio - pega o melhor individuo
    if (vetIndividual[ind1Pos]->getScore() > vetIndividual[ind2Pos]->getScore()) {
      return ind1Pos;
    } else {
      return ind2Pos;
    }
}

float GA::randomDouble(float minf, float maxf) {
  float a = random(0, 99);
  float b = random(1, 99);
  float c = random(0, 9);
  float randomFloat = ((a * b) / 10) + (c / 100);
  randomFloat = minf + randomFloat * (maxf - minf) / 980.19;
  return randomFloat;
}

void GA::printIndividual(int index) {
  Serial.print(vetIndividual[index]->getKp(),2);
  Serial.print(";");
  Serial.print(vetIndividual[index]->getKi(),2);
  Serial.print(";");
  Serial.print(vetIndividual[index]->getKd(),2);
}

void GA::printScore(int index){
  Serial.print(";");
  Serial.println(vetIndividual[index]->getScore(),2);
//  Serial.println("%");
}

void GA::deleteIndividual(){
  for(int i = 0; i < TAM; i++)
    delete(vetIndividual[i]);
}
