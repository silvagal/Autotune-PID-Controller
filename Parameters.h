#ifndef PARAMETERS_h
#define PARAMETERS_h

  #include <Wire.h>
  #include <Servo.h>
  
  #define BAUDRATE 250000
  #define NUMBER_OF_TESTS 15
  #define MAX_ESTABILIZATION_TIME 6000
  
  /*--------------------------- Motor -----------------------------*/
  
  #define MAX_SIGNAL 2000
  #define MIN_SIGNAL 1000
  #define NORMAL_SPEED 1170
  #define MAX_SPEED 1270
  #define MIN_SPEED 1070
  #define MOTOR1_PIN 6
  #define MOTOR2_PIN 7
  
  /*---------------------------- PID ------------------------------*/
  
  #define MAX_ANGLE 220
  #define MIN_ANGLE 140
  #define SETPOINT 180 
  #define PID_GAIN_LIMIT 100
  
  /*-------------------------- VNS ----------------------------------*/
  #define NUMBER_OF_SOLUTIONS 6
  #define NUMBER_OF_SEARCHES 15
  #define NUMBER_OF_ITERATIONS 25

  /*--------------------- RandomInt ----------------------------------*/
  #define MIN 0
  #define MAX 10

  /*---------------- Particle Swarm Optimization ------------------*/

  #define NUMBER_OF_PARTICLES 6
  
  #define NUMBER_OF_GENERATIONS 15
  
  #define MAX_VELOCITY  0.4
  #define MIN_VELOCITY -0.4 
  
  #define MAX_INERTIA 0.9 // Shi e Eberhart relataram bons resultados com esses valores
  #define MIN_INERTIA 0.4

  /*---------------- Genetic Algorithm ------------------*/
  #define NUMBER_OF_INDIVIDUALS 25
  #define NUMBER_OF_GENERATIONS 14
  
  #define TIME_OF_PERTURBATION 200
  #define FORCE_OF_PERTURBATION 220
  
  #define TAM NUMBER_OF_INDIVIDUALS * 2
  #define CROSSOVER_PROBABILITY  0.9
  #define MUTATION_PROBABILITY  0.1
 
#endif
