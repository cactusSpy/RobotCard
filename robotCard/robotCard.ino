/*
   Author: Alexis.m
   realesed 05/02/2023
   This code is used for servo gripper with 6 pwm for 7 servo motor (2 parallel)
   To use this code on Arduino board, please make sure you're using a suitable power!
*/

#include <Servo.h>
#include <Arduino.h>
#include "scanHandler.h"

#define DEMO   1 //si 1 alors test les mouvements aatention consommation d'energie élevé
#define FALSE  0
#define TRUE   1

#define SOCLE           0
#define PINCE_ROTATE    1
#define PINCE_LOCKING   2
#define BRAS_INF        3
#define COUDE           4
#define BRAS_SUP        5

Servo moteur_socle;  // create servo object to control a servo until 12 for arduino Uno
Servo moteur_pince_rotate;
Servo moteur_pince_locking;
Servo moteur_bras_inf;//there are 2 motor in parallel for this
Servo moteur_coude;
Servo moteur_bras_sup;

int flagMotor;//permet d'attentre la fin d'un mouvement
unsigned long timeEndOP;//permet d'attentre la fin d'un mouvement
int error = 0;
//unsigned long time;
unsigned long currentTime;
unsigned long previousTime = 0;
extern volatile unsigned long timer0_millis;/*A VOIR*/
unsigned long new_value = 1000;

typedef enum {POWERON, COUCOU, START, SCAN, TAKE, GIVE} ROBOT;
ROBOT EC_ROBOT, EF_ROBOT;

int motorInit(void);
int setMotorPilo(int motorName, unsigned int pos, unsigned int timeStartMs, unsigned long timeStopMs);

int robotGoCoucou(void);
int robotGoHome(void);
int robotGoDeck(void);
int robotGoGive(void);

int robotMaintient(int);

int robotGoPos(int temps, int deg);

int motorCalcul_timeStep(Servo motor, int motorName);
int motorCalcul_timeStep(int motorName);

void setMillis(unsigned long new_millis);


struct motor {
  unsigned int pos;
  unsigned long timeStartMs;
  unsigned long timeStopMs;
  unsigned int  pin;
  unsigned int currentPos;
  unsigned long previousTime;
  unsigned long timeStep;
};
typedef struct motor motor;// alias
motor motorPilot[6];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.print("moniteur test ");

  scanInit();//initialisation du capteur ultrasons
  motorInit();/* chargement des paramètre servo moteur PIN*/

  Serial.println(F("robot START"));
  robotMaintient(TRUE);/*Active l'auto maintient*/
  
  flagMotor=99;//acun mouvement
  EC_ROBOT=POWERON;

}      

void loop()
{

  switch (EC_ROBOT) {

     case POWERON:
   
      if(flagMotor==99){
        flagMotor=0;//mouvement en cours
        Serial.println(F("robot GO coucou"));
        robotGoHome();     
      }
      else if(flagMotor ==1)//mouvement terminer
      {
         Serial.println(F("coucou position OK"));
         robotMaintient(true); 
         Serial.println(F("go home"));
         flagMotor=99; //mouvement suivant
         EF_ROBOT = COUCOU;  
      }
      break;

     case COUCOU:
   
      if(flagMotor==99){
        flagMotor=0;//mouvement en cours
        Serial.println(F("robot GO coucou"));
        robotGoCoucou();     
      }
      else if(flagMotor ==1)//mouvement terminer
      {
         Serial.println(F("coucou position OK"));
         robotMaintient(true); 
         Serial.println(F("go home"));
         flagMotor=99; //mouvement suivant
         EF_ROBOT = START;  
      }
      break;

    case START:
   
      if(flagMotor==99){
        flagMotor=0;//mouvement en cours
        Serial.println(F("robot GO HOME"));
        robotGoHome();
        Serial.println(F("Waiting robot at home position"));         
      }
      else if(flagMotor ==1)//mouvement terminer
      {
         Serial.println(F("home position OK"));
         robotMaintient(false); 
         Serial.println(F("Waiting for client"));
         flagMotor=99; //mouvement suivant
         EF_ROBOT = SCAN;  
      }
      break;

      case SCAN:
      if (scanPresence()) {
        Serial.println(F("Client detecté!"));
        Serial.println(F("Robot take a card"));
        robotMaintient(true); 
        EF_ROBOT = TAKE;
      }
      break;

    case TAKE:
      if(flagMotor==99){
        flagMotor=0;//mouvement en cours
        Serial.println(F("robot take card"));
        robotGoDeck();
        }
      else if(flagMotor ==1)//mouvement terminer
        {
          Serial.println(F("take position OK"));
          //robotMaintient(false); 
          flagMotor=99; //mouvement suivant
          EF_ROBOT = GIVE;
        }  
      break;

    case GIVE:
      if(flagMotor==99){
        flagMotor=0;//mouvement en cours
      Serial.println(F("Robot Give the card"));
      robotGoGive();
      }
      else if(flagMotor ==1)//mouvement terminer
        {
          flagMotor=99; //mouvement suivant
          EF_ROBOT = START;
        }
      break;
    default:
      Serial.println(F("ERROR"));
      EF_ROBOT = START;
      break;
  }
  EC_ROBOT = EF_ROBOT;//actualisation de la MAE & des servo moteurs
  //actualisation des servo moteur
   robotGOPos(moteur_socle , SOCLE);
   robotGOPos(moteur_pince_rotate , PINCE_ROTATE);
   robotGOPos(moteur_pince_locking , PINCE_LOCKING);
   robotGOPos(moteur_bras_inf , BRAS_INF);
   robotGOPos(moteur_coude , COUDE);
   robotGOPos(moteur_bras_sup , BRAS_SUP);
   if(millis() >=  timeEndOP && flagMotor==0 )flagMotor=1;/* calcul la fin de mouvement*/

}

int robotGoHome(void)
{
  motorPilot[SOCLE].pos = 0;
  motorPilot[PINCE_ROTATE].pos = 0;
  motorPilot[PINCE_LOCKING].pos = 200;
  motorPilot[BRAS_INF].pos = 150;
  motorPilot[COUDE].pos = 0;
  motorPilot[BRAS_SUP].pos = 60;

  setMillis(0);
  currentTime = millis();
  timeEndOP= currentTime+ 8000; //accorde temps pour terminer les déplacement vers HOME
  motorPilot[SOCLE].timeStartMs = currentTime;
  motorPilot[PINCE_ROTATE].timeStartMs = currentTime;
  motorPilot[PINCE_LOCKING].timeStartMs = 4000 + currentTime;
  motorPilot[BRAS_INF].timeStartMs = 4000 + currentTime;
  motorPilot[COUDE].timeStartMs = 3000 + currentTime;
  motorPilot[BRAS_SUP].timeStartMs = 5000 + currentTime;

  motorPilot[SOCLE].timeStopMs = 4500 + currentTime;
  motorPilot[PINCE_ROTATE].timeStopMs = 2800 + currentTime;
  motorPilot[PINCE_LOCKING].timeStopMs = 5900 + currentTime;
  motorPilot[BRAS_INF].timeStopMs = 5200 + currentTime;
  motorPilot[COUDE].timeStopMs = 4800 + currentTime;
  motorPilot[BRAS_SUP].timeStopMs = 7000 + currentTime;

  motorPilot[SOCLE].previousTime = currentTime;
  motorPilot[PINCE_ROTATE].previousTime = currentTime;
  motorPilot[PINCE_LOCKING].previousTime = currentTime;
  motorPilot[BRAS_INF].previousTime = currentTime;
  motorPilot[COUDE].previousTime = currentTime;
  motorPilot[BRAS_SUP].previousTime = currentTime;

//calcul du delai timeStep pour chaque moteur
  motorCalcul_timeStep(moteur_socle, SOCLE);
  motorCalcul_timeStep(moteur_pince_rotate, PINCE_ROTATE);
  motorCalcul_timeStep(moteur_pince_locking, PINCE_LOCKING);
  motorCalcul_timeStep(moteur_bras_inf, BRAS_INF);
  motorCalcul_timeStep(moteur_coude, COUDE);
  motorCalcul_timeStep(moteur_bras_sup, BRAS_SUP);

  return 0;
}

int robotGoCoucou(void)
{
  motorPilot[SOCLE].pos = 140;
  motorPilot[PINCE_ROTATE].pos = 0;
  motorPilot[PINCE_LOCKING].pos = 350;
  motorPilot[BRAS_INF].pos = 80;
  motorPilot[COUDE].pos = 100;
  motorPilot[BRAS_SUP].pos = 130;

  setMillis(0);
  currentTime = millis();
  timeEndOP= currentTime+ 8000; //accorde temps pour terminer les déplacement vers HOME
  motorPilot[SOCLE].timeStartMs = currentTime;
  motorPilot[PINCE_ROTATE].timeStartMs = currentTime;
  motorPilot[PINCE_LOCKING].timeStartMs = 4000 + currentTime;
  motorPilot[BRAS_INF].timeStartMs = 0 + currentTime;
  motorPilot[COUDE].timeStartMs = 2000 + currentTime;
  motorPilot[BRAS_SUP].timeStartMs = 3000 + currentTime;

  motorPilot[SOCLE].timeStopMs = 3500 + currentTime;
  motorPilot[PINCE_ROTATE].timeStopMs = 3000 + currentTime;
  motorPilot[PINCE_LOCKING].timeStopMs = 7000 + currentTime;
  motorPilot[BRAS_INF].timeStopMs = 4000 + currentTime;
  motorPilot[COUDE].timeStopMs = 5000 + currentTime;
  motorPilot[BRAS_SUP].timeStopMs = 5500 + currentTime;

  motorPilot[SOCLE].previousTime = currentTime;
  motorPilot[PINCE_ROTATE].previousTime = currentTime;
  motorPilot[PINCE_LOCKING].previousTime = currentTime;
  motorPilot[BRAS_INF].previousTime = currentTime;
  motorPilot[COUDE].previousTime = currentTime;
  motorPilot[BRAS_SUP].previousTime = currentTime;

//calcul du delai timeStep pour chaque moteur
  motorCalcul_timeStep(moteur_socle, SOCLE);
  motorCalcul_timeStep(moteur_pince_rotate, PINCE_ROTATE);
  motorCalcul_timeStep(moteur_pince_locking, PINCE_LOCKING);
  motorCalcul_timeStep(moteur_bras_inf, BRAS_INF);
  motorCalcul_timeStep(moteur_coude, COUDE);
  motorCalcul_timeStep(moteur_bras_sup, BRAS_SUP);

  return 0;
}

int robotGoDeck(void)
{
  motorPilot[SOCLE].pos = 80;
  motorPilot[PINCE_ROTATE].pos = 0;
  motorPilot[PINCE_LOCKING].pos = 389;
  motorPilot[BRAS_INF].pos = 45;
  motorPilot[COUDE].pos = 60;
  motorPilot[BRAS_SUP].pos = 80;

  setMillis(0);
  currentTime = millis();
  timeEndOP= currentTime+ 10000; //accorde temps pour terminer les déplacement vers HOME
  motorPilot[SOCLE].timeStartMs = currentTime;
  motorPilot[PINCE_ROTATE].timeStartMs = currentTime;
  motorPilot[PINCE_LOCKING].timeStartMs = 4000 + currentTime;
  motorPilot[BRAS_INF].timeStartMs = 2000 + currentTime;
  motorPilot[COUDE].timeStartMs = 3000 + currentTime;
  motorPilot[BRAS_SUP].timeStartMs = 8000 + currentTime;

  motorPilot[SOCLE].timeStopMs = 1000 + currentTime;
  motorPilot[PINCE_ROTATE].timeStopMs = 3000 + currentTime;
  motorPilot[PINCE_LOCKING].timeStopMs = 7000 + currentTime;
  motorPilot[BRAS_INF].timeStopMs = 7000 + currentTime;
  motorPilot[COUDE].timeStopMs = 5500 + currentTime;
  motorPilot[BRAS_SUP].timeStopMs = 9000 + currentTime;

  motorPilot[SOCLE].previousTime = currentTime;
  motorPilot[PINCE_ROTATE].previousTime = currentTime;
  motorPilot[PINCE_LOCKING].previousTime = currentTime;
  motorPilot[BRAS_INF].previousTime = currentTime;
  motorPilot[COUDE].previousTime = currentTime;
  motorPilot[BRAS_SUP].previousTime = currentTime;

//calcul du delai timeStep pour chaque moteur
  motorCalcul_timeStep(moteur_socle, SOCLE);
  motorCalcul_timeStep(moteur_pince_rotate, PINCE_ROTATE);
  motorCalcul_timeStep(moteur_pince_locking, PINCE_LOCKING);
  motorCalcul_timeStep(moteur_bras_inf, BRAS_INF);
  motorCalcul_timeStep(moteur_coude, COUDE);
  motorCalcul_timeStep(moteur_bras_sup, BRAS_SUP);  
  return 0;
}

int robotGoGive(void)
{
 motorPilot[SOCLE].pos = 120;
  motorPilot[PINCE_ROTATE].pos = 0;
  motorPilot[PINCE_LOCKING].pos = 300;
  motorPilot[BRAS_INF].pos = 65;
  motorPilot[COUDE].pos = 80;
  motorPilot[BRAS_SUP].pos = 170;

  setMillis(0);
  currentTime = millis();
  timeEndOP= currentTime+ 15000; //accorde temps pour terminer les déplacement vers HOME
  motorPilot[SOCLE].timeStartMs = 5000+ currentTime;
  motorPilot[PINCE_ROTATE].timeStartMs = currentTime;
  motorPilot[PINCE_LOCKING].timeStartMs = 2000 + currentTime;
  motorPilot[BRAS_INF].timeStartMs = 1000 + currentTime;
  motorPilot[COUDE].timeStartMs = 2000 + currentTime;
  motorPilot[BRAS_SUP].timeStartMs = 6000 + currentTime;

  motorPilot[SOCLE].timeStopMs = 70001 + currentTime;
  motorPilot[PINCE_ROTATE].timeStopMs = 2000 + currentTime;
  motorPilot[PINCE_LOCKING].timeStopMs = 4000 + currentTime;
  motorPilot[BRAS_INF].timeStopMs = 4000 + currentTime;
  motorPilot[COUDE].timeStopMs = 5000 + currentTime;
  motorPilot[BRAS_SUP].timeStopMs = 7500 + currentTime;

  motorPilot[SOCLE].previousTime = currentTime;
  motorPilot[PINCE_ROTATE].previousTime = currentTime;
  motorPilot[PINCE_LOCKING].previousTime = currentTime;
  motorPilot[BRAS_INF].previousTime = currentTime;
  motorPilot[COUDE].previousTime = currentTime;
  motorPilot[BRAS_SUP].previousTime = currentTime;

//calcul du delai timeStep pour chaque moteur
  motorCalcul_timeStep(moteur_socle, SOCLE);
  motorCalcul_timeStep(moteur_pince_rotate, PINCE_ROTATE);
  motorCalcul_timeStep(moteur_pince_locking, PINCE_LOCKING);
  motorCalcul_timeStep(moteur_bras_inf, BRAS_INF);
  motorCalcul_timeStep(moteur_coude, COUDE);
  motorCalcul_timeStep(moteur_bras_sup, BRAS_SUP);  
  return 0;
}

int robotDemo(void)
{
  return 0;
}

int robotMaintient(int state)
{
  if (!state)
  {
    moteur_socle.detach();
    moteur_pince_rotate.detach();
    moteur_pince_locking.detach();
    moteur_bras_inf.detach();
    moteur_coude.detach();
    moteur_bras_sup.detach();
  }
  else {
    moteur_socle.attach(motorPilot[SOCLE].pin);
    moteur_pince_rotate.attach(motorPilot[PINCE_ROTATE].pin);
    moteur_pince_locking.attach(motorPilot[PINCE_LOCKING].pin);
    moteur_bras_inf.attach(motorPilot[BRAS_INF].pin);
    moteur_coude.attach(motorPilot[COUDE].pin);
    moteur_bras_sup.attach(motorPilot[BRAS_SUP].pin);
  }

}

int motorCalcul_timeStep(Servo motor, int motorName)
{
  Serial.print("currentPos : ");
  Serial.println(motor.read());//debogage

  Serial.print("angleStop : ");
  Serial.println(motorPilot[motorName].pos);//debogage

  Serial.print("timeStop : ");
  Serial.println(motorPilot[motorName].timeStopMs);//debogage
  unsigned int currentPos = motor.read();

  if (currentPos > motorPilot[motorName].pos)
  {
    motorPilot[motorName].timeStep = ((motorPilot[motorName].timeStopMs) / ((motor.read() - motorPilot[motorName].pos )));
  }
  else
  {
    motorPilot[motorName].timeStep = ((motorPilot[motorName].timeStopMs) / ((motorPilot[motorName].pos - motor.read())));
  }
  Serial.print("timeStep : ");
  Serial.println(motorPilot[motorName].timeStep);//debogage
  return 1;
}

int robotGOPos(Servo moteur, int motorName)
{
  motorPilot[motorName].currentPos = moteur.read();
  int angleStop  = motorPilot[motorName].pos;
  int i = motorPilot[motorName].currentPos;
  int timeStop = motorPilot[motorName].timeStopMs;
  //Serial.print("timeStop :"); Serial.println(timeStop);
  unsigned long t = motorPilot[motorName].timeStep;
  if (i != angleStop)
  {
    if ((angleStop <= 360) && (angleStop >= 0))
    {
      // if ((timeStop - previousTime) <= 0)
      // {
      //   moteur.write(angleStop);
      // }
      //else
      //{
      //Serial.println("test1");
      currentTime = millis();
      if ((currentTime - motorPilot[motorName].previousTime) > motorPilot[motorName].timeStep)
      {

        Serial.print("AngleStop :"); Serial.println(angleStop);
        Serial.print("currentPos :"); Serial.println(i);
        if (i < angleStop)
        {
          Serial.println("incremente");
          moteur.write(i + 1);
          motorPilot[motorName].previousTime = currentTime;
        }
        else if (i > angleStop)
        {
          Serial.println("Decremente");
          moteur.write(i - 1);
          motorPilot[motorName].previousTime = currentTime;
        }
      }
      //}
    }
  }
}


int motorInit()
{
  //start the config for all the servos motors
  motorPilot[SOCLE].pin = 10;//ok
  motorPilot[PINCE_ROTATE].pin = 9;// ok
  motorPilot[PINCE_LOCKING].pin = 4;//ok
  motorPilot[BRAS_INF].pin = 11;// ok
  motorPilot[COUDE].pin = 3;//OK
  motorPilot[BRAS_SUP].pin = 6;//OK

  pinMode(11, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(3, OUTPUT);

  //robotMaintient(TRUE); appeler si test en dehors du main
  return 0;
}

int setMotorPilot(int motorName, unsigned int pos, unsigned int timeStartMs, unsigned long timeStopMs)
{
  motorPilot[motorName].pos = pos;
  motorPilot[motorName].timeStartMs = timeStartMs;
  motorPilot[motorName].timeStopMs = timeStopMs;
  return 0;
}

void setMillis(unsigned long new_millis) {
  uint8_t oldSREG = SREG;
  cli();
  timer0_millis = new_millis;
  SREG = oldSREG;
}
