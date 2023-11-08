#include <Adafruit_MotorShield.h>
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motor1 = AFMS.getMotor(3);
Adafruit_DCMotor *motor2 = AFMS.getMotor(2);
bool moving = false;
int ledB = 1;
int ledR = 3;
unsigned long previousTime = millis();
int ledState = LOW;
int magnet = 0;
int magnetPin = 2;

void setup(){
  Serial.begin(9600);
  // put your setup code here, to run once:
  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
  while (1);
  }
  pinMode(ledB, OUTPUT);
}

void loop(){
  unsigned long currentTime = millis();
  if(currentTime - previousTime > 500) {
    previousTime = currentTime;
    if (ledState == HIGH) {
      ledState = LOW;
    }
    else {
      ledState = HIGH;
    }
    digitalWrite(ledB, ledState);
  }
   motor(3,200);
   magnet_test();
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
    case 3: // TURN
      motor1->run(FORWARD);
      motor2->run(BACKWARD);
      speeed_up(speeed);
      break;
    case 4: // TURN
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
