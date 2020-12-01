#include "struktura.h"
#include <Servo.h> 
//Servo Servo1; 
//Servo Servo2; 
struct stepper PrviMotor;
struct stepper DrugiMotor;
struct stepper TreciMotor;


void setup() {
  PrviMotor.stepPin=2;
  DrugiMotor.stepPin=3;
  TreciMotor.stepPin=4;
  PrviMotor.dirPin=5;
  DrugiMotor.dirPin=6;
  TreciMotor.dirPin=7;
  pinMode(PrviMotor.stepPin,OUTPUT); 
  pinMode(DrugiMotor.stepPin,OUTPUT);
  pinMode(TreciMotor.stepPin,OUTPUT);
  pinMode(PrviMotor.dirPin,OUTPUT);
  pinMode(DrugiMotor.dirPin,OUTPUT);
  pinMode(TreciMotor.dirPin,OUTPUT); 
//  Servo1.attach(CetvrtiMotor); 
 // Servo2.attach(PetiMotor); 
  Serial.begin(9600);
  Serial.println("Unesi x, y, z, roll i postotak zatvorenosti gripera:");

  cli();//stop interrupts
  //set timer1 interrupt at 10kHz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set timer count for 1Mhz increments
  OCR1A = 199;// = (16*10^6) / (10000*8) - 1
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 bit for 8 prescaler
  TCCR1B |= (1 << CS11);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();//allow interrupts
}

ISR(TIMER1_COMPA_vect){ //timer 1 ponavlja se svake 0.1 ms
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//STEPPER CONTROL
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if(start){
  vrijeme++;
  }
  
  if (vrijeme==PrviMotor.vrijemekoraka && PrviMotor.pin==false && PrviMotor.korak<PrviMotor.ukupnikoraci){  //provjera dali je vrijeme da korak pokrene
    ZapocniKorak(&PrviMotor);
  }else if (vrijeme==PrviMotor.vrijemekoraka && PrviMotor.pin==true && PrviMotor.korak<=PrviMotor.ukupnikoraci){
    ZavrsiKorak(&PrviMotor);
    izracunajDilej(&PrviMotor);
  }
  
  if (vrijeme==DrugiMotor.vrijemekoraka && DrugiMotor.pin==false  && DrugiMotor.korak<DrugiMotor.ukupnikoraci){
    ZapocniKorak(&DrugiMotor);
  }else if (vrijeme==DrugiMotor.vrijemekoraka && DrugiMotor.pin==true  && DrugiMotor.korak<=DrugiMotor.ukupnikoraci){
    ZavrsiKorak(&DrugiMotor); 
    izracunajDilej(&DrugiMotor);
  }
  
  if (vrijeme==TreciMotor.vrijemekoraka && TreciMotor.pin==false  && TreciMotor.korak<TreciMotor.ukupnikoraci){
    ZapocniKorak(&TreciMotor);
  }else if (vrijeme==TreciMotor.vrijemekoraka && TreciMotor.pin==true  && TreciMotor.korak<=TreciMotor.ukupnikoraci){
    ZavrsiKorak(&TreciMotor);
    izracunajDilej(&TreciMotor);
  }
  
  if (start && PrviMotor.korak==PrviMotor.ukupnikoraci && DrugiMotor.korak==DrugiMotor.ukupnikoraci && TreciMotor.korak==TreciMotor.ukupnikoraci){
    start=false; 
    vrijeme++;
  }  
}    
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//CITANJE PODATAKA I POKRETANJE MOTORA
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  if (Serial.available()){
    x=Serial.parseFloat();
    y=Serial.parseFloat();
    z=Serial.parseFloat();  
    roll=Serial.parseInt();
    GripperPostotak=Serial.parseFloat();
    if (sqrt(sq(x) + sq(y))>41){       
       Serial.println("Podatci uneseni!");    
       proracunKoraka();
       proracunBrzina();
       proracunAkc(&PrviMotor);
       proracunAkc(&DrugiMotor);
       proracunAkc(&TreciMotor);
       reset();
    }else if (x!=0 && y!=0){
      Serial.println("Podatci uneseni su krivi!"); 
    }
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PRAZNJENJE VARIJABLA I POKRETANJE KRETNJE
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void reset(){
  vrijeme=0;
  PrviMotor.korak=0;
  DrugiMotor.korak=0;
  TreciMotor.korak=0;
  PrviMotor.vrijemekoraka=PrviMotor.cekanje[0];
  DrugiMotor.vrijemekoraka=DrugiMotor.cekanje[0];
  TreciMotor.vrijemekoraka=TreciMotor.cekanje[0];
  start=true;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PRORACUNA KORAKA, SMJEROVA I KUTOVA MOTORA
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void proracunKoraka(){
  finew=atan2(y, x)*RAD_TO_DEGRESS;   //računanje kutova 
  l=sqrt(sq(x) + sq(y));  
  if (z>C){
    b=sqrt(sq(z-C) + sq(l));
    alfanew=acos(l/b) + acos((sq(KRAK1) + sq(b) - sq(KRAK2)) / (2*KRAK1*b));   //inverzna kinematika, cosinusov poučak
    betanew=alfanew + acos((sq(KRAK1) + sq(KRAK2) - sq(b)) / (2*KRAK1*KRAK2)); 
  }else {    
    b=sqrt(sq(z-C) + sq(l));
    alfanew=acos((sq(KRAK1) + sq(b) - sq(KRAK2)) / (2*KRAK1*b))-acos(l/b);   //inverzna kinematika, cosinusov poučak
    betanew=alfanew + acos((sq(KRAK1) + sq(KRAK2) - sq(b)) / (2*KRAK1*KRAK2)); 
  }
      
  alfanew*= RAD_TO_DEGRESS;
  betanew*= RAD_TO_DEGRESS; 
  
  Serial.println("alfa:");    
  Serial.println(alfanew); 
  Serial.println("beta:"); 
  Serial.println(betanew); 
  Serial.println("fi:"); 
  Serial.println(finew); 
  
  if(fi>finew){      //pretvorba iz kuta u broj koraka i namještavanje smjera
    digitalWrite(PrviMotor.dirPin,HIGH);
    PrviMotor.ukupnikoraci=(fi-finew)/FI_STEP;
  }else {
    digitalWrite(PrviMotor.dirPin,LOW);
    PrviMotor.ukupnikoraci=(finew-fi)/FI_STEP;
  }
 
  if(alfa>alfanew){
    digitalWrite(DrugiMotor.dirPin,LOW);
    DrugiMotor.ukupnikoraci=(alfa-alfanew)/AFLA_STEP;
  }else {
    digitalWrite(DrugiMotor.dirPin,HIGH);
    DrugiMotor.ukupnikoraci=(alfanew-alfa)/AFLA_STEP;
  }

  if(beta>betanew){
    digitalWrite(TreciMotor.dirPin,HIGH);
    TreciMotor.ukupnikoraci=(beta-betanew)/BETA_STEP;
  }else {
    digitalWrite(TreciMotor.dirPin,LOW);
    TreciMotor.ukupnikoraci=(betanew-beta)/BETA_STEP;
  }
  
  fi=finew; //pamčenje kutova
  alfa=alfanew;
  beta=betanew;
  gripper=((GRIPPER_MAX-GRIPPER_MIN)*GripperPostotak)+GRIPPER_MIN;
//  Servo1.write(roll); 
//  Servo2.write(gripper);
  
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PRORACUN BRZINA ZA SVAKI STEPPER MOTOR
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void proracunBrzina(){
  if((PrviMotor.ukupnikoraci >= DrugiMotor.ukupnikoraci) && (PrviMotor.ukupnikoraci >= TreciMotor.ukupnikoraci)){
        PrviMotor.period=PAUZA;
        DrugiMotor.period=float(PrviMotor.ukupnikoraci/DrugiMotor.ukupnikoraci)*PAUZA;
        TreciMotor.period=float(PrviMotor.ukupnikoraci/TreciMotor.ukupnikoraci)*PAUZA;
  }else if ((DrugiMotor.ukupnikoraci >= PrviMotor.ukupnikoraci) && (DrugiMotor.ukupnikoraci >= TreciMotor.ukupnikoraci)){
        DrugiMotor.period=PAUZA;
        PrviMotor.period=float(DrugiMotor.ukupnikoraci/PrviMotor.ukupnikoraci)*PAUZA;
        TreciMotor.period=float(DrugiMotor.ukupnikoraci/TreciMotor.ukupnikoraci)*PAUZA;
  } else{
        TreciMotor.period=PAUZA;
        PrviMotor.period=float(TreciMotor.ukupnikoraci/PrviMotor.ukupnikoraci)*PAUZA;
        DrugiMotor.period=float(TreciMotor.ukupnikoraci/DrugiMotor.ukupnikoraci)*PAUZA;
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PRORACUN AKCELERACIJE ZA SVAKI STEPPER MOTOR
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void proracunAkc(struct stepper * motor){
  if (motor->ukupnikoraci>=(STEPSA*2)){
      for (i=0; i<(STEPSA*2); i++){
        if (i<STEPSA){
          motor->cekanje[i]=(STEPSA-i)*A+motor->period;
        }  else{
          motor->cekanje[i]=motor->cekanje[i-1]+A;
        }
      }
  }else {
      for (i=0; i<motor->ukupnikoraci; i++){
        if (i<(motor->ukupnikoraci/2)){
          motor->cekanje[i]=(STEPSA-i)*A+motor->period;
        }else{
          motor->cekanje[i]=motor->cekanje[i-1]+A;
        }
      }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PRORACUN VREMENA CEKANJA ZA SLJEDECI IMPULS STEPPER MOTORA
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void izracunajDilej (struct stepper * motor){
    if(motor->ukupnikoraci>=(STEPSA*2)){
      if (motor->korak<STEPSA){
         motor->vrijemekoraka+= motor->cekanje[motor->korak];
      }else if (motor->korak>(motor->ukupnikoraci-STEPSA)){
         motor->vrijemekoraka+= motor->cekanje[motor->korak - motor->ukupnikoraci + STEPSA*2];
      }else{
         motor->vrijemekoraka+= motor->period;
      }
    }else{
      motor->vrijemekoraka+=  motor->cekanje[motor->korak];
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//KONTROLA IMPULSA STEPPER MOTORA
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ZapocniKorak (struct stepper * motor){
    digitalWrite(motor->stepPin,HIGH);
    motor->vrijemekoraka+=IMPULS;
    motor->korak++;
    motor->pin=true;
}
void ZavrsiKorak (struct stepper * motor){
    digitalWrite(motor->stepPin,LOW);
    motor->pin=false;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
