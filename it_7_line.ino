#include <Adafruit_MotorShield.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motor1 = AFMS.getMotor(3);
Adafruit_DCMotor *motor2 = AFMS.getMotor(2);
bool moving = false;
int ledB = 1;
int ledR = 3;
int v3 = 0;
unsigned long previousTime = millis();
int ledState = LOW;
int magnet = 0;
int magnetPin = 2;
int leftFrontLinePin = 7;
int rightFrontLinePin = 6; 
int leftBackLinePin = 4;
int rightBackLinePin = 5;
int leftFrontLine = 0;
int rightFrontLine = 0;
int leftBackLine = 0;
int rightBackLine = 0;

int ultraSoundDistance = 0;
int timeOfFlightDistance = 0;

int smallLoop[16] = {1,3,1,1,4,1,4,1,1,1,1,4,1,4,1,1};
bool lineMode = true;

void setup(){
  Serial.begin(9600);
  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
  while (1);
  }
  pinMode(ledB, OUTPUT);
  pinMode(ledR, OUTPUT);
  pinMode(magnetPin, INPUT);
  pinMode(leftFrontLine, INPUT);
  pinMode(rightFrontLine, INPUT);
}

void loop(){
  unsigned long currentTime = millis();
  if((currentTime - previousTime > 500) and moving) {
    previousTime = currentTime;
    // flashing light
    if (ledState == HIGH) {
      ledState = LOW;
    }
    else {
      ledState = HIGH;
    }
    digitalWrite(ledB, ledState);
  }
  magnet_test();

  if(lineMode){
    grid();
  } else {
    field();
  }
}

void lineFollow(){
    leftFrontLine = digitalRead(leftFrontLinePin);
    rightFrontLine = digitalRead(rightFrontLinePin);
    leftBackLine = digitalRead(leftBackLinePin);
    rightBackLine = digitalRead(rightBackLinePin);
    
    if(leftFrontLine == HIGH and rightFrontLine == HIGH){
      motor(1, 200);
      moving = true;
    }
    else if(leftFrontLine == HIGH and rightFrontLine == LOW){
        motor1->run(FORWARD);
        motor2->run(FORWARD);
        motor2->setSpeed(225);
        motor1->setSpeed(70);
        moving = true;
      }
     else if(leftFrontLine == LOW and rightFrontLine == HIGH){
        motor1->run(FORWARD);
        motor2->run(FORWARD);
        motor2->setSpeed(70);
        motor1->setSpeed(225);
        moving = true;
     }
}

void lock_onto_block(){
  //ultraSound = digitalRead(ultraSound);
  //timeOfFlight = digitalRead(timeOfFlight);


}

//{1,3,1,1,4,1,4,1,1,1,1,4,1,4,1,1}

void grid(){
  //assumes starting in starting box facing towards field
  // NEXT: implement scanning for blocks en route, dropping off blocks, ending up in centre of long edge of field after both blocks dropped off
  // and turn lineMode to false after both grid blocks collected
  int i = 0;
  while(smallLoop[i] == 1){
    leftBackLine = digitalRead(leftBackLinePin);
    rightBackLine = digitalRead(rightBackLinePin);
    leftFrontLine = digitalRead(leftFrontLinePin);
    rightFrontLine = digitalRead(rightFrontLinePin);
    lineFollow();

    if(leftBackLine == HIGH or rightBackLine == HIGH){
      i++;
      if (smallLoop[i]==3){ // turn right
         while(leftFrontLine == HIGH and rightFrontLine == HIGH){
          motor(3,200); //turn right until off line
          leftFrontLine = digitalRead(leftFrontLinePin);
          rightFrontLine = digitalRead(rightFrontLinePin);
         }while(leftFrontLine == LOW or rightFrontLine == LOW){
          motor(3,200); //turn right until on new line
          leftFrontLine = digitalRead(leftFrontLinePin);
          rightFrontLine = digitalRead(rightFrontLinePin);
         }
          i++;
    } else if (smallLoop[i]==4){ // turn left
         while(leftFrontLine == HIGH and rightFrontLine == HIGH){
          motor(4,200);
          leftFrontLine = digitalRead(leftFrontLinePin);
          rightFrontLine = digitalRead(rightFrontLinePin);
         } while(leftFrontLine == LOW or rightFrontLine == LOW){
          motor(4,200);
          leftFrontLine = digitalRead(leftFrontLinePin);
          rightFrontLine = digitalRead(rightFrontLinePin);
         } 
         i++;
  }
}
}
}

void scan_field(){
  //assumes starting at centre of long edge of field facing into field
  while(leftFrontLine == LOW or rightFrontLine == LOW){
    motor(4,200); //slower than usual turn
    leftFrontLine = digitalRead(leftFrontLinePin);
    rightFrontLine = digitalRead(rightFrontLinePin);
  } 
  //scan from left to right
  while(leftFrontLine == HIGH and rightFrontLine == HIGH and ultraSoundDistance > 170){
    motor(3,100); //slower than usual turn
    leftFrontLine = digitalRead(leftFrontLinePin);
    rightFrontLine = digitalRead(rightFrontLinePin);
  } while(leftFrontLine == LOW and rightFrontLine == LOW and ultraSoundDistance > 170){
    motor(3,100);
    leftFrontLine = digitalRead(leftFrontLinePin);
    rightFrontLine = digitalRead(rightFrontLinePin);
  }
}

void return_block(){
  //How to do a 180 turn??!!
  magnet = digitalRead(magnetPin); 
  if (magnet == HIGH) { 
    digitalWrite(ledR, HIGH);
  } else {
    digitalWrite(ledR,LOW);
  }
}

void field(){
  scan_field()
  lock_onto_block()
  return_block()
  scan_field()
  lock_onto_block()
  return_block()
}


void speeed_up(int speeed){
  /*
  uint8_t i;
  for (i=0; i<speeed; i++) {
     motor1->setSpeed(i);
     motor2->setSpeed(i);
     delay(10);
  }
  for (i=speeed;i>0;i--) {
     motor1->setSpeed(i);
     motor2->setSpeed(i);
     delay(10);
  }
  */
     motor1->setSpeed(speeed);
     motor2->setSpeed(speeed);
}

void motor(int dir,int speeed){
 // motor1->setSpeed(speeed);
 // motor2->setSpeed(speeed);
  switch (dir){
    case 0: //OFF
      motor1->run(RELEASE);
      motor2->run(RELEASE);
      break;
    case 1: //FORWARD
      motor1->run(FORWARD);
      motor2->run(FORWARD);
      speeed_up(speeed);
      break;
    case 2: //BACKWARD
      motor1->run(BACKWARD);
      motor2->run(BACKWARD);
      speeed_up(speeed);
      break;
    case 3: // TURN RIGHT
      motor1->run(FORWARD);
      motor2->run(BACKWARD);
      speeed_up(speeed);
      break;
    case 4: // TURN LEFT
      motor2->run(FORWARD);
      motor1->run(BACKWARD);  
      speeed_up(speeed);
      break;
  }
  if (dir != 0){
    moving = true;
  }else{
    moving = false;
  }
  /*
  if(dir == 0){
  motor1->run(RELEASE);
  motor2->run(RELEASE);
}

  if(dir == 1){
  motor1->run(FORWARD);
  motor2->run(FORWARD);
  */
}  

void magnet_test(){
  magnet = digitalRead(magnetPin); // read input value
  if (magnet == HIGH) { // check if the input is HIGH
    digitalWrite(ledR, HIGH);
  } else {
    digitalWrite(ledR,LOW);
  }
}
