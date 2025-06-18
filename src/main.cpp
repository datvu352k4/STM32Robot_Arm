#include <Arduino.h>
#include <AccelStepper.h>

#define STEP_PIN1L PB12
#define DIR_PIN1L PB13
#define STEP_PIN1R PB14
#define DIR_PIN1R PB15
#define STEP_PIN2 PA11
#define DIR_PIN2 PA12
#define STEP_PIN3 PA15
#define DIR_PIN3 PB3
#define STEP_PIN4 PB4
#define DIR_PIN4 PB5

AccelStepper stepper1L(AccelStepper::DRIVER, STEP_PIN1L, DIR_PIN1L);
AccelStepper stepper1R(AccelStepper::DRIVER, STEP_PIN1R, DIR_PIN1R);
AccelStepper stepper2(AccelStepper::DRIVER, STEP_PIN2, DIR_PIN2);
AccelStepper stepper3(AccelStepper::DRIVER, STEP_PIN3, DIR_PIN3);
AccelStepper stepper4(AccelStepper::DRIVER, STEP_PIN4, DIR_PIN4);

bool homed1,homed2,homed3,homed4 = false;
bool waitingAfterHome = false;
bool homingDone = false;
unsigned long homeTime = 0;
int a1 = 90 , a2 = 90, a3 = 90, a4 = 90;

long  step1 = a1 * 4380.0 / 180.0 ,
      step2 = a2 * 3300.0 / 180.0 ,
      step3 = a3 * 3000.0 / 180.0 ;

void setup() {
  Serial.begin(115200);
  pinMode(PB9, INPUT);
  pinMode(PB8, INPUT);
  pinMode(PB7, INPUT);
  pinMode(PB6, INPUT);
  pinMode(PC13,OUTPUT);

  stepper1L.setMaxSpeed(800);
  stepper1L.setAcceleration(500);
  
  stepper1R.setMaxSpeed(800);
  stepper1R.setAcceleration(500);

  stepper2.setMaxSpeed(1000);
  stepper2.setAcceleration(800);

  stepper3.setMaxSpeed(1000);
  stepper3.setAcceleration(800);

  stepper4.setMaxSpeed(600);
  stepper4.setAcceleration(300);

}
void check(){
  if (digitalRead(PB8) == 1) {
      stepper2.stop(); 
      stepper2.setCurrentPosition(1650*2); 
  }
  if (digitalRead(PB7) == 1) {
      stepper3.stop(); 
      stepper3.setCurrentPosition(1500*2); 
  }
  if (digitalRead(PB9) == 1) {
      stepper1L.stop(); 
      stepper1R.stop();
      stepper1L.setCurrentPosition(0); 
      stepper1R.setCurrentPosition(0);
  }
  if (digitalRead(PB6) == 1) {
        stepper4.stop(); 
        stepper4.setCurrentPosition(0);
  }
}
void home(){
  if(!homed3){
    stepper3.moveTo(10000); 
    stepper3.run();

    if (digitalRead(PB7) == 1) {
      stepper3.stop(); 
      stepper3.setCurrentPosition(1500*2); 

      homed3 = true;
    }
  }
  else if(!homed2) {
    stepper2.moveTo(10000); 
    stepper2.run();

    if (digitalRead(PB8) == 1) {
      stepper2.stop(); 
      stepper2.setCurrentPosition(1650*2); 

      homed2 = true;
    }
  }
  else if(!homed1) {
    stepper1L.moveTo(-10000); 
    stepper1R.moveTo(-10000);

    stepper1L.run();
    stepper1R.run();

    if (digitalRead(PB9) == 1) {
      stepper1L.stop(); 
      stepper1R.stop();
      
      stepper1L.setCurrentPosition(0); 
      stepper1R.setCurrentPosition(0);

      homed1 = true;
    }
  }
  else if(!homed4){
    stepper2.moveTo(3300);
    stepper3.moveTo(1000);
    stepper2.run();
    stepper3.run();
    if(stepper2.distanceToGo() == 0 && 
    stepper3.distanceToGo() == 0){
    stepper1L.moveTo(4380);
    stepper1R.moveTo(4380);
    stepper1L.run();
    stepper1R.run();
    }

    if (stepper1L.distanceToGo() == 0 && 
    stepper1R.distanceToGo() == 0 && 
    stepper2.distanceToGo() == 0 && 
    stepper3.distanceToGo() == 0) 
    {

      stepper4.moveTo(10000); 
      stepper4.run();

      if (digitalRead(PB6) == 1) {
        stepper4.stop(); 
        stepper4.setCurrentPosition(0);
        stepper4.moveTo(-200); 
        stepper4.run();

        homed4 = true;
        waitingAfterHome = true;
        homeTime = millis();  // ghi lại thời điểm home xong
      }
    }
  }
  else {
    if (waitingAfterHome) {
      if (millis() - homeTime >= 2000) { // đợi 2 giây
        waitingAfterHome = false;
        
        stepper1L.setMaxSpeed(2000);
        stepper1L.setAcceleration(1200);
        
        stepper1R.setMaxSpeed(2000);
        stepper1R.setAcceleration(1200);

        stepper1L.moveTo(2190);  // chạy sau 2 giây
        stepper1R.moveTo(2190);

        stepper2.setMaxSpeed(2000);
        stepper2.setAcceleration(1200)  ;
        stepper2.moveTo(1650);  

        stepper3.setMaxSpeed(2000);
        stepper3.setAcceleration(1200);
        stepper3.moveTo(1500);  //

        stepper4.setMaxSpeed(2000);
        stepper4.setAcceleration(1200);
        stepper4.moveTo(-2000);  //
      }
    }
    else {
      stepper4.run();
      if(stepper4.distanceToGo() == 0){
      stepper1L.run();
      stepper1R.run();
      stepper2.run();
      stepper3.run();
      if(stepper1L.distanceToGo() == 0 && stepper1R.distanceToGo() == 0 && stepper2.distanceToGo() == 0 && stepper3.distanceToGo() == 0 )homingDone = true;
      }
    }
  }
}

void readSerialInput() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
      a1 = input.substring(0, input.indexOf(',')).toInt();
      a2 = input.substring(input.indexOf(',')+1, input.lastIndexOf(',')).toInt();
      a3 = input.substring(input.lastIndexOf(',')+1).toInt();
  }
}

void moveRobot(long step1, long step2, long step3){
  stepper1L.moveTo(step1);
  stepper1R.moveTo(step1);
  stepper2.moveTo(step2);
  stepper3.moveTo(step3);
  stepper1L.run();
  stepper1R.run();
  stepper2.run();
  stepper3.run();
}
/*
void loop(){
  if (digitalRead(PB8)==1)digitalWrite(PC13,1);
  else digitalWrite(PC13,0);
}*/

void loop() {
  if(!homingDone)
    home();
  else{
    readSerialInput();
    step1 = a1 * 4380.0 / 180.0;
    step2 = a2 * 3300.0 / 180.0;
    step3 = a3 * 3000.0 / 180.0;
    moveRobot(step1, step2, step3);
  }  
}

