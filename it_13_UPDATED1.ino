#include <Adafruit_MotorShield.h>
#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motor1 = AFMS.getMotor(3);
Adafruit_DCMotor *motor2 = AFMS.getMotor(2);
bool moving = false;
#define HIGH_ACCURACY

int leftFrontLinePin = 0;
int rightFrontLinePin = 1; 
int leftBackLinePin = 2;
int rightBackLinePin = 3;
int ledR = 4;
int ledG = 5;
int ledB = 6;
int magnetPin = 7;
int ultraSoundPin = 0; //analogue read
int buttonPin = 8;

int magnet = 0;
int v3 = 0;
unsigned long previousTime = millis();
unsigned long lastNodeDetectTime = millis();
int ledState = LOW;
int blockCount = 0;

int leftFrontLine = 0;
int rightFrontLine = 0;
int leftBackLine = 0;
int rightBackLine = 0;

int ultraSoundDistance = 50;
unsigned long timeOfFlightDistance = 55;

int smallLoop[23] = {1,1,4,1,1,3,1,3,1,1,1,1,3,1,3,1,1,3,1,4,1,1,3};
int bigLoop[8] = {1,3,1,3,1,3,1,1};

bool gridMode = true;
bool blockMagnetic = false;
bool stop = false;

void setup(){
  Serial.begin(9600);
  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
  while (1);
  }
  //set up pins
  pinMode(ledB, OUTPUT);
  pinMode(ledR, OUTPUT);
  pinMode(magnetPin, INPUT);
  pinMode(leftFrontLine, INPUT);
  pinMode(rightFrontLine, INPUT);
  pinMode(leftBackLine, INPUT);
  pinMode(rightBackLine, INPUT);
  pinMode(buttonPin, INPUT);
  //wait until button is pressed before continuing
  while (digitalRead(buttonPin) == LOW) {}
  //move forward a wee bit
  motor(1,200);
  delay(700);
  //set up tof sensor
  Wire.begin();
  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }
#if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  sensor.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif
#if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
  sensor.setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
  // increase timing budget to 200 ms
  sensor.setMeasurementTimingBudget(200000);
#endif
}

void loop(){
  Serial.print(sensor.readRangeSingleMillimeters());
  if (sensor.timeoutOccurred()){Serial.print(" TIMEOUT");}
  Serial.println();
  
  if(gridMode){
    grid();
  } else {
    field();
  }
}
void blink(){
  //the only important function
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

}
void lineFollow(){
  //get state of each line sensor
    leftFrontLine = digitalRead(leftFrontLinePin);
    rightFrontLine = digitalRead(rightFrontLinePin);
    leftBackLine = digitalRead(leftBackLinePin);
    rightBackLine = digitalRead(rightBackLinePin);
    
    //move forward if on the line completely
    if(leftFrontLine == HIGH and rightFrontLine == HIGH){
      motor(1, 200);
      moving = true;
    }     
    //if goes off line to the left then slow down right motor
    else if(leftFrontLine == LOW and rightFrontLine == HIGH){
        motor1->run(FORWARD);
        motor2->run(FORWARD);
        motor2->setSpeed(90);
        motor1->setSpeed(225);
        moving = true;
     }
     //vice versa
    else if(leftFrontLine == HIGH and rightFrontLine == LOW){
        motor1->run(FORWARD);
        motor2->run(FORWARD);
        motor2->setSpeed(225);
        motor1->setSpeed(90);
        moving = true;
      }

}

void lock_onto_block(){

}

//{1,3,1,1,4,1,4,1,1,1,1,1,4,1,4,1,1}

void grid(){
  //assumes starting in starting box facing towards field
  int i = 0;
  Serial.println("While loop");
  //while loop basically iterates through an array containing the instructions for what to do at each node
  while(i < 23 and smallLoop[i] == 1 and stop == false){
    //get sensor readings
    blink();
    leftBackLine = digitalRead(leftBackLinePin);
    rightBackLine = digitalRead(rightBackLinePin);
    leftFrontLine = digitalRead(leftFrontLinePin);
    rightFrontLine = digitalRead(rightFrontLinePin);
    lineFollow();
    //if a block is detected, set by threshold timeOfFlightDistance then stop and execute the return block function
    if (sensor.readRangeSingleMillimeters() <= timeOfFlightDistance)
    {
      stop = true;
      motor(0,0);
      delay(500);
      return_block(i);
      //break out of while loop when the block is returned
      break;
    }
    //get current time
    unsigned long currentNodeDetectTime = millis();

    //if back sensors cross line and sufficient time has passed, detect a node
    if((leftBackLine == HIGH or rightBackLine == HIGH) and currentNodeDetectTime - lastNodeDetectTime > 1600) {
      Serial.println("Node");
      lastNodeDetectTime = currentNodeDetectTime;
      currentNodeDetectTime = millis();
      //go to next instruction, wll be a turn direction in the form of a number
      i++;
      Serial.print(i); Serial.println(smallLoop[i]);
      if (smallLoop[i]==3){ // turn right
         while(leftFrontLine == HIGH and rightFrontLine == HIGH){
          blink();
          motor(3,200); //turn right until off line
          leftFrontLine = digitalRead(leftFrontLinePin);
          rightFrontLine = digitalRead(rightFrontLinePin);
         }
         delay(1000);
         Serial.println("off line");
         while(leftFrontLine == LOW or rightFrontLine == LOW){
          blink();
          motor(3,200); //turn right until on new line
          leftFrontLine = digitalRead(leftFrontLinePin);
          rightFrontLine = digitalRead(rightFrontLinePin);
         }
         Serial.println("on new line");
          currentNodeDetectTime = millis();
          //go to next instruction, will be line following (1)
          i++;
      } else if (smallLoop[i]==4){ // turn left
         while(leftFrontLine == HIGH and rightFrontLine == HIGH){ //turn left until off line
         blink();
          motor(4,200);
          leftFrontLine = digitalRead(leftFrontLinePin);
          rightFrontLine = digitalRead(rightFrontLinePin);
         } 
         delay(500);
         while(leftFrontLine == LOW or rightFrontLine == LOW){ //turn left until on new line
         blink();
          motor(4,200);
          leftFrontLine = digitalRead(leftFrontLinePin);
          rightFrontLine = digitalRead(rightFrontLinePin);
         } 
         currentNodeDetectTime = millis();
         //go to next instruction, will be line following (1)
         i++;
      }
      currentNodeDetectTime = millis();
    }
  }
  //gridMode = false;
  if (blockCount == 2)
  {
    gridMode = false;
  }
}


void return_block(int i){
  blockCount++;
  //magnet test - delay exists to ensure block is under magnet sensor, don't remove this delay
  delay(200);
  magnet_test();

  //180 turn
  //turns a ways off the line >90 deg
  motor(3,200);
  delay(1000);
  leftFrontLine = digitalRead(leftFrontLinePin);
  rightFrontLine = digitalRead(rightFrontLinePin);
  //keeps turning until it has hit a line and thus turned 180
  while(leftFrontLine == LOW and rightFrontLine == LOW)
  {
    blink();
    motor(3,200);
    leftFrontLine = digitalRead(leftFrontLinePin);
    rightFrontLine = digitalRead(rightFrontLinePin);
  }
  motor(0,0);
  //back to start, iterates backwards through grid array
  while(i>1 and smallLoop[i]==1){
    blink();
    leftBackLine = digitalRead(leftBackLinePin);
    rightBackLine = digitalRead(rightBackLinePin);
    leftFrontLine = digitalRead(leftFrontLinePin);
    rightFrontLine = digitalRead(rightFrontLinePin);
    lineFollow();
    unsigned long currentNodeDetectTime = millis();

    if((leftBackLine == HIGH or rightBackLine == HIGH) and currentNodeDetectTime - lastNodeDetectTime > 1600) {
      lastNodeDetectTime = currentNodeDetectTime;
      currentNodeDetectTime = millis();
      i--;
      if (smallLoop[i]==4){ // turn right
         while(leftFrontLine == HIGH and rightFrontLine == HIGH){
          blink();
          motor(3,200); //turn right until off line
          leftFrontLine = digitalRead(leftFrontLinePin);
          rightFrontLine = digitalRead(rightFrontLinePin);
         }
         delay(500);
         while(leftFrontLine == LOW or rightFrontLine == LOW){
          blink();
          motor(3,200); //turn right until on new line
          leftFrontLine = digitalRead(leftFrontLinePin);
          rightFrontLine = digitalRead(rightFrontLinePin);
         }
          currentNodeDetectTime = millis();
          i--;
      } else if (smallLoop[i]==3){ // turn left
         while(leftFrontLine == HIGH and rightFrontLine == HIGH){
          blink();
          motor(4,200);
          leftFrontLine = digitalRead(leftFrontLinePin);
          rightFrontLine = digitalRead(rightFrontLinePin);
         } 
         delay(500);
         while(leftFrontLine == LOW or rightFrontLine == LOW){
          blink();
          motor(4,200);
          leftFrontLine = digitalRead(leftFrontLinePin);
          rightFrontLine = digitalRead(rightFrontLinePin);
         } 
         currentNodeDetectTime = millis();
         i--;
      }
      currentNodeDetectTime = millis();
    }
  }
  unsigned long prevTime = millis();
  //line follows for a bit along the line coming out the start box
  while(millis() - prevTime < 3500){
    blink();
    lineFollow();
  }
  //goes forward a bit until in the box
  //motor(1,200);
  motor1->run(FORWARD);
  motor2->run(FORWARD);
  motor1->setSpeed(255);
  motor2->setSpeed(215);
  delay(2500);
  motor(0,0);
  delay(500);
  //turns towards the right box to put the block in
  if (blockMagnetic == true)
  {
    motor(3, 200);
  }
  else
  {
    motor(4, 200);
  }
  
  delay(1400);
  motor(0,0);
  //goes forward until in new box
  leftBackLine = digitalRead(leftBackLinePin);
  rightBackLine = digitalRead(rightBackLinePin);
  motor1->run(FORWARD);
  motor2->run(FORWARD);
  motor1->setSpeed(255);
  motor2->setSpeed(215);
  delay(4000);
  while (leftBackLine == LOW && rightBackLine == LOW)
  {
    leftBackLine = digitalRead(leftBackLinePin);
    rightBackLine = digitalRead(rightBackLinePin);
    motor(1,200);
  }
  motor(0,0);
  delay(500);
  //reverses until back at start box
  leftFrontLine = digitalRead(leftFrontLinePin);
  rightFrontLine = digitalRead(rightFrontLinePin);
  motor1->run(BACKWARD);
  motor2->run(BACKWARD);
  motor1->setSpeed(255);
  motor2->setSpeed(215);
  delay(2000);
  while (leftFrontLine == LOW && rightFrontLine == LOW)
  {
    leftFrontLine = digitalRead(leftFrontLinePin);
    rightFrontLine = digitalRead(rightFrontLinePin);
    motor1->setSpeed(255);
    motor2->setSpeed(215);
  }
  delay(500);
  motor(0,0);
  if (blockCount == 1)
  {
    digitalWrite(ledB, HIGH);
    delay(6000);
  }
  else
  {
    delay(1000);
  }
  digitalWrite(ledB, LOW);
  //delay(2000);
  if (blockMagnetic == true)
  {
    motor(3, 200);
  }
  else
  {
    motor(4, 200);
  }
  delay(1700);
  motor(0,0);
  delay(1000);
  motor(1,200);
  delay(500);
  motor(0,0);
  delay(500);

}

//{1,1,3,1,3,1,3,1,1}

void field(){
  int i = 0;
  Serial.println("While loop");
  while(i < 8 and bigLoop[i] == 1 and ultraSoundDistance < 60){
    leftBackLine = digitalRead(leftBackLinePin);
    rightBackLine = digitalRead(rightBackLinePin);
    leftFrontLine = digitalRead(leftFrontLinePin);
    rightFrontLine = digitalRead(rightFrontLinePin);
    lineFollow();
    unsigned long currentNodeDetectTime = millis();

    if((leftBackLine == HIGH or rightBackLine == HIGH) and currentNodeDetectTime - lastNodeDetectTime > 1600) {
      lastNodeDetectTime = currentNodeDetectTime;
      currentNodeDetectTime = millis();
      i++;
      if (bigLoop[i]==3){ // turn right
         while(leftFrontLine == HIGH and rightFrontLine == HIGH){
          motor(3,200); //turn right until off line
          leftFrontLine = digitalRead(leftFrontLinePin);
          rightFrontLine = digitalRead(rightFrontLinePin);
         }while(leftFrontLine == LOW or rightFrontLine == LOW){
          motor(3,200); //turn right until on new line
          leftFrontLine = digitalRead(leftFrontLinePin);
          rightFrontLine = digitalRead(rightFrontLinePin);
         }
          currentNodeDetectTime = millis();
          i++;
      } else if (bigLoop[i]==4){ // turn left
         while(leftFrontLine == HIGH and rightFrontLine == HIGH){
          motor(4,200);
          leftFrontLine = digitalRead(leftFrontLinePin);
          rightFrontLine = digitalRead(rightFrontLinePin);
         } while(leftFrontLine == LOW or rightFrontLine == LOW){
          motor(4,200);
          leftFrontLine = digitalRead(leftFrontLinePin);
          rightFrontLine = digitalRead(rightFrontLinePin);
         } 
         currentNodeDetectTime = millis();
         i++;
      }
      currentNodeDetectTime = millis();
    }
  }
}


void speeed_up(int speeed){
  //this is the most useluss function ever written, but i love it and will not remove it, technically it saves 1 line of code total
     motor1->setSpeed(speeed);
     motor2->setSpeed(speeed);
}

void motor(int dir,int speeed){
 //switches on the integer direction indicating the desired direction
  switch (dir){
    case 0: //OFF
      motor1->run(RELEASE);
      motor2->run(RELEASE);
      break;
    case 1: //FORWARD
      motor1->run(FORWARD);
      motor2->run(FORWARD);
      speeed_up(speeed*1.275);
      break;
    case 2: //BACKWARD
      motor1->run(BACKWARD);
      motor2->run(BACKWARD);
      speeed_up(speeed*1.275);
      break;
    case 3: // TURN RIGHT
      motor1->run(FORWARD);
      motor2->run(BACKWARD);
      speeed_up(speeed*1.275);
      break;
    case 4: // TURN LEFT
      motor2->run(FORWARD);
      motor1->run(BACKWARD);  
      speeed_up(speeed*1.275);
      break;
  }
  //sets boolean moving which will be used for the blinking blinking blue light
  if (dir != 0){
    moving = true;
  }else{
    moving = false;
  }
}  

void magnet_test(){
  magnet = digitalRead(magnetPin); // read input value
  if (magnet == HIGH) { // check if the input is HIGH
    digitalWrite(ledR, HIGH);
    digitalWrite(ledG, LOW);
    blockMagnetic = true;
  } else {
    digitalWrite(ledR,LOW);
    digitalWrite(ledG,HIGH);
    blockMagnetic = false;
  }
  //stops, waits 7 seconds with the relevant LED on, then turns both LEDs off
  motor(0,200);
  delay(7000);
  digitalWrite(ledR,LOW);
  digitalWrite(ledG,LOW);

}
