#define DISABLE_PCINT_MULTI_SERVICE

#include "PinChangeInt.h"

boolean senseFrontRigth, senseFrontLeft, senseLeft, senseRigth, senseBack, searching;
boolean endLineInterrupt = true;
unsigned long frontRigthTime, frontLeftTime, leftTime, rigthTime, backTime;
const long minTimeFind = 100;

const int SignalPin = 0;
const int ReciverPin = 1;

const int sensorAdIzq = 2; 
const int sensorAdDer = 3;
const int sensorDer = 4;
const int Sensor4 = 7;
const int Sensor5 = 8;

const int MotIzqFw = 9;
const int MotDerFw = 5;
const int MotDerBk = 6;
const int MotIzqBk = 10;

const int CNYAdDe = A0;
const int CNYAdIz = A1;
const int CNYAtDe = A3;
const int CNYAtIz = A2;

const int SHOVEL = A5;

const int MidSpeedPersent = 70;

const int MaxSpeedPersent = 100;

const int delayGiro = 15000;
const int delayGiroInterrupcion = 100;

const unsigned int SensorFrecuncy = 40000;
const int SensorSignalLenth = 201;
const int DistMaxSensor = 150; 

const int delayLinea = 20000; // tiempo que el robot se mueve recto al tocar una linea.
const int porcentageParaGiroEnLinea = 50;// porcentaje del que depende la curba que realiza el robot al tocar linea. 
const int delayEsquivar = 30000; // tiempo en que el robot mantiene la curva al esquibar una linea;

void setup(){
  pinMode(SignalPin, OUTPUT);
  pinMode(ReciverPin, INPUT);
  pinMode(sensorAdIzq, OUTPUT);
  pinMode(sensorAdDer, OUTPUT);
  pinMode(sensorDer, OUTPUT);
  pinMode(Sensor4, OUTPUT);
  pinMode(Sensor5, OUTPUT);
  
  pinMode(SHOVEL, OUTPUT);
  
  pinMode(MotDerFw, OUTPUT);
  pinMode(MotDerBk, OUTPUT);
  pinMode(MotIzqFw, OUTPUT);  
  pinMode(MotIzqBk, OUTPUT);
   
  pinMode(CNYAdDe, INPUT);
  pinMode(CNYAdIz, INPUT);
  pinMode(CNYAtDe, INPUT);
  pinMode(CNYAtIz, INPUT);
 /* 
  PCintPort::attachInterrupt(CNYAdDe, CNYAdDeDetecto, RISING);
  PCintPort::attachInterrupt(CNYAdIz, CNYAdIzDetecto, RISING);
  PCintPort::attachInterrupt(CNYAtDe, CNYAtDeDetecto, RISING);
  PCintPort::attachInterrupt(CNYAtIz, CNYAtIzDetecto, RISING);
 */ 
  
  start();
}

void loop() {
  //Detect with all sensors
  if(senso(sensorAdDer)){
    senseFrontRigth = true;
    frontRigthTime = millis();
    searching = false;
  }
  
  if(senso(sensorAdIzq)){
    senseFrontLeft = true;
    frontLeftTime = millis();
    searching = false;
  }
  
  if(searching){
    search();
  }
  else{
    follow();
  } 
}

void search(){
  stopIt();
  //TODO secuencia de busqueda
}

void follow(){
  if(senseFrontRigth && senseFrontLeft){
    stopIt();
    start();
  } else if(senseFrontRigth && (millis()-frontRigthTime) < minTimeFind){
    girarDer();
  } else if(senseFrontLeft && (millis()-frontLeftTime) < minTimeFind){
    girarIzq();
  }else {
    start();
  }
}

void start(){
  senseFrontRigth = false;
  senseFrontLeft = false;
  senseLeft = false;
  senseRigth = false;
  senseBack = false;
  searching = true;
}

boolean senso(int sensor){
  boolean result = false;
  
  digitalWrite(sensor,HIGH);
  delayMicroseconds(2000);
  tone(SignalPin, SensorFrecuncy);
  delayMicroseconds(400);
  noTone(SignalPin);
  
  long time2 = 0;
  while(digitalRead(ReciverPin) != HIGH && time2 < 930){
    time2++;
  }
  
  digitalWrite(sensor,LOW);
  if (time2 < 930){
    result = true;
  }
  //delayMicroseconds(1000);
  return result; 
}

int getMotorPinContrario(int motor){
  if(motor == MotDerFw){
    return MotDerBk;
  } else if(motor == MotDerBk){
    return MotDerFw;
  } else if(motor == MotIzqFw){
    return MotIzqBk;
  } else if(motor == MotIzqBk){
    return MotIzqFw;
  } else {
    return -1;
  }
		
}

void dropShovel(){
  girarIzq();
  delay(1000);
}

void movAdFullSpeed(){
  digitalWrite(MotDerFw, HIGH);
  digitalWrite(MotDerBk, LOW);
  digitalWrite(MotIzqFw, HIGH);
  digitalWrite(MotIzqBk, LOW);
}

void midSpeed(int motor){
  analogWrite(motor, MidSpeedPersent*255/100);
  digitalWrite(getMotorPinContrario(motor),LOW);
}

void preferedSpeed(int motor, int persent){
  analogWrite(motor, persent*255/100);
  digitalWrite(getMotorPinContrario(motor),LOW);
}
 
void maxSpeed(int motor){
  analogWrite(motor, MaxSpeedPersent*255/100);
  digitalWrite(getMotorPinContrario(motor),LOW);
}

boolean CNYSenso(int cny){
  return digitalRead(cny);
}


void CNYAdDeDetecto(){
  endLineInterrupt = false;
  sei();
  if ( CNYSenso(CNYAdIz)){
    esquivarLinAd();
  } else if( CNYSenso(CNYAtDe)){
    esquivarLinDer();
  } else {
    esquivarLinDerAd();
  }
  endLineInterrupt = true;
}

void CNYAdIzDetecto(){
  endLineInterrupt = false;
  sei();
  if ( CNYSenso(CNYAdDe)){
    esquivarLinAd();
  } else if( CNYSenso(CNYAtIz)){
    esquivarLinIzq();
  } else {
    esquivarLinIzqAd();
  }
  endLineInterrupt = true;
}

void CNYAtDeDetecto(){
  endLineInterrupt = false;
  sei();
  if ( CNYSenso(CNYAtIz)){
    esquivarLinAt();
  } else if( CNYSenso(CNYAdDe)){
    esquivarLinDer();
  } else {
    esquivarLinDerAt();
  }
  endLineInterrupt = true;
}

void CNYAtIzDetecto(){
  endLineInterrupt = false;
  sei();
  if ( CNYSenso(CNYAtDe)){
    esquivarLinAt();
  } else if( CNYSenso(CNYAdIz)){
    esquivarLinIzq();
  } else {
    esquivarLinIzqAt();
  }
  endLineInterrupt = true;
}


void movAt(){
  maxSpeed(MotDerBk);
  maxSpeed(MotIzqBk);
}

void stopIt(){
  digitalWrite(MotDerFw, LOW);
  digitalWrite(MotDerBk, LOW);
  digitalWrite(MotIzqFw, LOW);
  digitalWrite(MotIzqBk, LOW);
  
}

void stopLeft(){
  digitalWrite(MotDerFw, HIGH);
  digitalWrite(MotDerBk, HIGH);
}

void stopRigth(){
  digitalWrite(MotIzqFw, HIGH);
  digitalWrite(MotIzqBk, HIGH);
}

void movAd(){
  maxSpeed(MotDerFw);
  maxSpeed(MotIzqFw);
}

void girarDer(){
  maxSpeed(MotDerBk);
  maxSpeed(MotIzqFw);
}

void girarIzq(){
  maxSpeed(MotDerFw);
  maxSpeed(MotIzqBk);
} 

void doblarDer(){
  girarDer();
  delay(delayGiro);
  movAd();
}

void doblarIzq(){
  girarIzq();
  delay(delayGiro);
  movAd();
}

void doblarDer180(){
  girarDer();
  delay(delayGiro * 2);
  movAd();
}

void doblarIzq180(){
  girarIzq();
  delay(delayGiro * 2);
  movAd();
}

void esquivarLinIzqAd(){
  //esquivarLinAd();
  movAt();
  int time = 0;
  while ( time < 25000){
    if (CNYSenso(CNYAdIz)){
      time = 0;
    }
    time++;
  }
  maxSpeed(MotDerBk);
  maxSpeed(MotIzqFw);
  time = 0;
  while ( time < 16000){
    if (CNYSenso(CNYAdIz)){
      time = 0;
    }
    time++;
  }
  movAd();
}

void esquivarLinDerAd(){
  //esquivarLinAd();
  movAt();
  int time = 0;
  while ( time < 25000){
    if (CNYSenso(CNYAdDe)){
      time = 0;
    }
    time++;
  }
  maxSpeed(MotDerFw);
  maxSpeed(MotIzqBk);
  time = 0;
  while ( time < 16000){
    if (CNYSenso(CNYAdDe)){
      time = 0;
    }
    time++;
  }
  movAd();
}

void esquivarLinIzqAt(){
  //esquivarLinAt();
  midSpeed(MotDerFw);
  maxSpeed(MotIzqFw);
  int time = 0;
  while ( time < 15000){
    if ( CNYSenso(CNYAtIz)){
      time = 0;
    }
    time++;
  }
  movAd();
}

void esquivarLinDerAt(){
  //esquivarLinAt();
  midSpeed(MotIzqFw);
  maxSpeed(MotDerFw);
  int time = 0;
  while ( time < 15000){
    if ( CNYSenso(CNYAtDe)){
      time = 0;
    }
    time++;
  }
  movAd();
}

void esquivarLinIzq(){
  unsigned int time = 0;
  while(CNYSenso(CNYAtIz) || CNYSenso(CNYAdIz) || time < delayGiroInterrupcion){
     stopRigth();
     maxSpeed(MotIzqFw);    
     time++; 
  }  
  movAd();
}

void esquivarLinDer(){
  unsigned int time = 0;
  while(CNYSenso(CNYAtDe) || CNYSenso(CNYAdDe) || time < delayGiroInterrupcion){
     stopRigth();
     maxSpeed(MotIzqFw);    
     time++; 
  }  
  movAd();  
}

void esquivarLinAd(){
  movAt();
  int time = 0;
  while ( time < 12000){
    if ( CNYSenso(CNYAdIz) || CNYSenso(CNYAdDe) ){
      time = 0;
    }
    time++;
  }
}

void esquivarLinAt(){
  movAd();
  int time = 0;
  while (time < 12000){
    if ( CNYSenso(CNYAtIz) || CNYSenso(CNYAtDe) ){
      time = 0;
    }
    time++;
  }
}


