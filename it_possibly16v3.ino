#include <Adafruit_MotorShield.h>
#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motor1 = AFMS.getMotor(3);
Adafruit_DCMotor *motor2 = AFMS.getMotor(2);
bool moving = false;
#define HIGH_ACCURACY
#define MAX_RANG (520)//the max measurement value of the module is 520cm(a little bit longer
//than effective max range)
#define ADC_SOLUTION (1023.0)//ADC accuracy of Arduino UNO is 10bit
//#define LONG_RANGE

// initialises all pin numbers
int leftFrontLinePin = 0;
int rightFrontLinePin = 1; 
int leftBackLinePin = 2;
int rightBackLinePin = 3;
int ledR = 4;
int ledG = 5;
int ledB = 6;
int magnetPin = 7;
int ultraSoundPin = A0; //analogue read
int buttonPin = 8;

int magnet = 0;
int v3 = 0;
unsigned long previousTime = millis();
unsigned long lastNodeDetectTime = millis();
unsigned long currentUltraSoundTime = millis();
unsigned long previousUltraSoundTime = 0;
int timeToTurn180 = 1800;
unsigned long turnTime = 0;

int ledState = LOW;
int blockCount = 0;
int fieldBlockCount = 0;

int leftFrontLine = 0;
int rightFrontLine = 0;
int leftBackLine = 0;
int rightBackLine = 0;

int ultraSoundDistance = 50;
unsigned long timeOfFlightDistance = 55;

int smallLoop[23] = {1,1,4,1,1,3,1,3,1,1,1,1,3,1,3,1,1,3,1,4,1,1,3};
int bigLoop[15] = {1,1,4,1,1,3,1,3,1,3,1,3,1,1,1};
int returnPath[10] = {1,3,1,3,1,1,4,1,1,1};
int toStartOfField[7] = {1,1,1,4,1,1,3};
int savedPosition = 3;

bool gridMode = true; //set to false for isolated field testing
bool blockMagnetic = false;
bool stop = false;
bool detecting;
float sensity_t;
float dist_t;

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
  // print time of flight sensor readings
  Serial.print(sensor.readRangeSingleMillimeters());
  if (sensor.timeoutOccurred()){Serial.print(" TIMEOUT");}
  Serial.println();
  
  if(gridMode){ // if we need to traverse the small box
  Serial.println("grid block");
    grid();
  } else {  // if we need to traverse large box
    if(fieldBlockCount < 2){
      Serial.println("field block");
      field();
    }
  }
}

void blink(){
  // ensures the blue light is blinking whenever the car is moving
  unsigned long currentTime = millis();
  if((currentTime - previousTime > 500) and moving) {  // changes state of blue light every 0.5 seconds
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

/*
void lock_onto_block(){

}
*/

//{1,3,1,1,4,1,4,1,1,1,1,1,4,1,4,1,1}

void grid(){
  //assumes starting in starting box facing towards field
  int i;
  //i = blockCount >= 1 ? 1 : 0;
  i = 0;
  Serial.println("While loop");
  
  while(leftFrontLine == LOW and rightFrontLine == LOW){
    blink();
    motor(1,200); //turn right until off line
    leftFrontLine = digitalRead(leftFrontLinePin);
    rightFrontLine = digitalRead(rightFrontLinePin);
  }

  //while loop basically iterates through an array containing the instructions for what to do at each node
  while(i < 23 and smallLoop[i] == 1){
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
      motor(0,0);
      delay(500);
      savedPosition = i;
      return_block(i);
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
  //back to start, iterates backwards through grid array (same while loop as in grid function, but now we iterate backwards)
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
  //line follows for a bit along the line coming out the start box (for 3.5s) until going straight
  while(millis() - prevTime < 3500){
    blink();
    lineFollow();
  }
  //goes forward a bit until in the box
  // different speeds so that it goes straight
  motor1->run(FORWARD);
  motor2->run(FORWARD);
  motor1->setSpeed(255);
  motor2->setSpeed(215);
  delay(3000); // GOES FORWARD TILL FULLY IN WHITE BOX
  motor(0,0);
  delay(500);
  if (blockMagnetic == true) //turns towards the red box to put the block in
  {
    motor(3, 200);
    delay(1200);
  }
  else  // turns towards green box to put block in
  {
    motor(4, 200);
    delay(1500);
  }
  
  //delay(1400);
  motor(0,0);

  // after turning, we move forwards until at least one of the front line sensors are on
  leftBackLine = digitalRead(leftBackLinePin);
  rightBackLine = digitalRead(rightBackLinePin);
  while(leftBackLine == LOW and rightBackLine == LOW){
    motor1->run(FORWARD);
    motor2->run(FORWARD);
    motor1->setSpeed(255);
    motor2->setSpeed(215); 
    leftBackLine = digitalRead(leftBackLinePin);
    rightBackLine = digitalRead(rightBackLinePin);  
  }

  leftBackLine = digitalRead(leftBackLinePin);
  rightBackLine = digitalRead(rightBackLinePin);
  // Add correction when hits white line of white box so that it is straight
  Serial.println(leftBackLine);
  Serial.println(rightBackLine);
  while(leftBackLine == HIGH and rightBackLine == LOW){
    Serial.println("correction");
    motor(4,100);
    leftBackLine = digitalRead(leftBackLinePin);
    rightBackLine = digitalRead(rightBackLinePin);
    //delay(100);
  }
  // Add correction when hits white line of white box so that it is straight
  while(leftBackLine == LOW and rightBackLine == HIGH){
    Serial.println("correction");
    motor(3,100);
    leftBackLine = digitalRead(leftBackLinePin);
    rightBackLine = digitalRead(rightBackLinePin);
    //delay(100);
  }
  motor(0,0);
  delay(1000);
/*
  if(leftBackLine == HIGH and rightBackLine == HIGH){
    motor(0,0);
    delay(1000);
  }
*/

  //goes forward until in new box
  motor1->run(FORWARD);
  motor2->run(FORWARD);
  motor1->setSpeed(240);
  motor2->setSpeed(210);
  delay(4000);
  leftBackLine = digitalRead(leftBackLinePin);
  rightBackLine = digitalRead(rightBackLinePin);
  
  while (leftBackLine == LOW && rightBackLine == LOW)
  {
    leftBackLine = digitalRead(leftBackLinePin);
    rightBackLine = digitalRead(rightBackLinePin);
    motor1->setSpeed(240);
    motor2->setSpeed(210);
  }
  motor(0,0);
  delay(500);
  //reverses until back at start box
  motor1->run(BACKWARD);
  motor2->run(BACKWARD);
  motor1->setSpeed(240);
  motor2->setSpeed(225);
  delay(2000);

  // continues reversing until at least one of the back line sensors detects HIGH
  leftBackLine = digitalRead(leftBackLinePin);
  rightBackLine = digitalRead(rightBackLinePin);
  while (leftBackLine == LOW && rightBackLine == LOW)
  {
    leftBackLine = digitalRead(leftBackLinePin);
    rightBackLine = digitalRead(rightBackLinePin);
    motor1->setSpeed(240);
    motor2->setSpeed(225);
  }
  Serial.println("back at start box");
  motor(0,0);
  delay(500);
  // rotates until one of the front line sensors detects HIGH i.e. back almost parallel to start box white line
  leftFrontLine = digitalRead(leftFrontLinePin);
  rightFrontLine = digitalRead(rightFrontLinePin);
  while (leftFrontLine == LOW or rightFrontLine == LOW)
  {
    leftFrontLine = digitalRead(leftFrontLinePin);
    rightFrontLine = digitalRead(rightFrontLinePin);
    if (blockMagnetic == true)  // must rotate in different directions depending on whether the block just delivered was magnetic or not
    {
      motor(3,200);
    }
    else
    {
      motor(4,200);
    }
  }
  Serial.println("rotated onto edge");
  motor(0,0);
  delay(200);

  // line follows this start box line until one of the back line sensors turns HIGH
  leftBackLine = digitalRead(leftBackLinePin);
  rightBackLine = digitalRead(rightBackLinePin);
  /*
  prevTime = millis();
  //line follows for a bit along the line coming out the start box (for 3.5s) until going straight
  while(millis() - prevTime < 3500){
    blink();
    lineFollow();
  }*/
  while (leftBackLine == LOW && rightBackLine == LOW)
  {
    lineFollow();
    leftBackLine = digitalRead(leftBackLinePin);
    rightBackLine = digitalRead(rightBackLinePin);
  }
  motor(0,0);
  Serial.println("At corner");
  //delay(20000);
  
  int turnDir;  // determines the direction we need to turn depending on the state of blockMagentic
  turnDir = blockMagnetic == true ? 3 : 4;
  leftFrontLine = digitalRead(leftFrontLinePin);
  rightFrontLine = digitalRead(rightFrontLinePin);
  // turns at corner of start box until one of the front line sensors turns HIGH
  while(leftFrontLine == LOW and rightFrontLine == LOW){
    motor(turnDir, 200);
    leftFrontLine = digitalRead(leftFrontLinePin);
    rightFrontLine = digitalRead(rightFrontLinePin);
  }
  // line follows for an extra 0.2s after this to reduce error
  unsigned long delay1 = millis();
  while (millis() - delay1 < 200){
    lineFollow();
  }

  // line follows along this line until we reach the intersecting node
  leftFrontLine = digitalRead(leftFrontLinePin);
  rightFrontLine = digitalRead(rightFrontLinePin);
  leftBackLine = digitalRead(leftBackLinePin);
  rightBackLine = digitalRead(rightBackLinePin);
  while (leftBackLine == LOW or rightBackLine == LOW)
  {
    if ((blockMagnetic == true && leftBackLine == HIGH) || (blockMagnetic == false && rightBackLine == HIGH))
    {
      break;
    }
    if(leftFrontLine == LOW and rightFrontLine == LOW){
      motor(turnDir, 200);
    }
    lineFollow();
    leftFrontLine = digitalRead(leftFrontLinePin);
    rightFrontLine = digitalRead(rightFrontLinePin);
    leftBackLine = digitalRead(leftBackLinePin);
    rightBackLine = digitalRead(rightBackLinePin);
  }
  motor(0,0);
  delay(200);

  // swaps the value of turnDir and rotates for half a second
  turnDir = turnDir == 3 ? 4 : 3;
  motor(turnDir, 150);
  delay(500);
  // then continues rotating until we are incident on the line out of the start box
  leftFrontLine = digitalRead(leftFrontLinePin);
  rightFrontLine = digitalRead(rightFrontLinePin);
  while(leftFrontLine == LOW or rightFrontLine == LOW){
    motor(turnDir, 200);
    leftFrontLine = digitalRead(leftFrontLinePin);
    rightFrontLine = digitalRead(rightFrontLinePin);
  }
  motor(0,0);
  delay(200);
  
  // line follows for 2s to get it parallel to this line
  delay1 = millis();
  while(millis() - delay1 < 2000){
    lineFollow();
  }
  motor(0,0);
  delay(500);
  // reverse until we are in start box fully
  leftBackLine = digitalRead(leftBackLinePin);
  rightBackLine = digitalRead(rightBackLinePin);
  while(!(leftBackLine == HIGH and rightBackLine == HIGH)){
   leftBackLine = digitalRead(leftBackLinePin);
    rightBackLine = digitalRead(rightBackLinePin);
    motor(2,200);
  }
  delay(1000);

  // NEED TO ADD BLUE LIGHT TURNING ON FOR 6 SECONDS

  /*
  while (leftFrontLine == LOW && rightFrontLine == LOW)
  {
    motor(3,200);
    leftFrontLine = digitalRead(leftFrontLinePin);
    rightFrontLine = digitalRead(rightFrontLinePin);
  }
  while (!(leftFrontLine==LOW and rightFrontLine == LOW)){
    lineFollow();
    leftFrontLine = digitalRead(leftFrontLinePin);
    rightFrontLine = digitalRead(rightFrontLinePin);
  }
  while(leftFrontLine == LOW){
    motor(3,200);
  }
  while(rightBackLine == LOW){
    lineFollow();
  }
  motor(4,200);
  delay(500);
  while(rightFrontLine == LOW){
    motor(4,200);
  }
  lineFollow();
  delay(1000);
  */
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

  motor(1,200);
  delay(700);
  //delay(2000);
  /*
  if (blockMagnetic == true)
  {
    motor(3, 200);
  }
  else
  {
    motor(4, 200);
  }

  delay(1500);
  motor(0,0);
  delay(1000);
  motor(1,200);
  delay(500);
  motor(0,0);
  //delay(500);
  lineFollow();
  */
}




//{1,1,3,1,3,1,3,1,1,1}

void field(){
  int i = 0;
  Serial.println("field");
  while(i < 15 and bigLoop[i] == 1){
    currentUltraSoundTime = millis();
    leftBackLine = digitalRead(leftBackLinePin);
    rightBackLine = digitalRead(rightBackLinePin);
    leftFrontLine = digitalRead(leftFrontLinePin);
    rightFrontLine = digitalRead(rightFrontLinePin);
    lineFollow();

    currentUltraSoundTime = millis();
    if (i == 8 and currentUltraSoundTime - previousUltraSoundTime > 500 and millis() - lastNodeDetectTime > 3500){
      previousUltraSoundTime = currentUltraSoundTime;
      sensity_t = analogRead(ultraSoundPin);
      dist_t = sensity_t * MAX_RANG / ADC_SOLUTION;
      if(dist_t < 60){
        Serial.println("break");
        break;
      }
    }

    unsigned long currentNodeDetectTime = millis();
    if(((leftBackLine == HIGH or rightBackLine == HIGH) or (leftFrontLine == LOW and rightFrontLine == LOW)) and currentNodeDetectTime - lastNodeDetectTime > 6000) {
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

          if(i == 8){
            detecting = true; 
            Serial.println("Detecting");
          } 
          else{
            detecting = false;
          }
          
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
  collectBlockInField(i);
}

void collectBlockInField(int k){
  Serial.println("COLLECTING BLOCK");
  float a0 =  10000;
  float a1 =  10000;
  float a2 =  10000;
  float av = 100;
  //turns until it picks it up on the time of flight
  while(av > 85){
    a2 = a1;
    a1 = a0;
    a0 = sensor.readRangeSingleMillimeters();
    av = (a2 + a1 + a0)/3;
    Serial.println(av);
    blink();
    motor(3,200);
  }
  //goes to get it
  while(sensor.readRangeSingleMillimeters() > 55){
    blink();
    motor(1,200);
    moving = true;
  }
  delay(200);
  magnet_test();
  fieldBlockCount ++;

  if(k==8){
    turnTime = millis();
    while(millis() - turnTime < timeToTurn180){
      blink();
      motor(4,200);
    }
  }
  //go back to far side of field
  while(leftBackLine == LOW and rightBackLine == LOW){
    blink();
    motor(1,200);
    leftBackLine = digitalRead(leftBackLinePin);
    rightBackLine = digitalRead(rightBackLinePin);
  }
  //turn right until back on line
  while(leftFrontLine == LOW and rightFrontLine == LOW){
    blink();
    motor(3,200);
    leftFrontLine = digitalRead(leftFrontLinePin);
    rightFrontLine = digitalRead(rightFrontLinePin);
  }
  k=0;
  //go back to box
  while(k < 10 and returnPath[k] == 1){
    leftBackLine = digitalRead(leftBackLinePin);
    rightBackLine = digitalRead(rightBackLinePin);
    leftFrontLine = digitalRead(leftFrontLinePin);
    rightFrontLine = digitalRead(rightFrontLinePin);
    lineFollow();

    unsigned long currentNodeDetectTime = millis();
    if(((leftBackLine == HIGH or rightBackLine == HIGH) or (leftFrontLine == LOW and rightFrontLine == LOW)) and currentNodeDetectTime - lastNodeDetectTime > 1600) {
      lastNodeDetectTime = currentNodeDetectTime;
      currentNodeDetectTime = millis();
      k++;
      if (returnPath[k]==3){ // turn right
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
          k++;
      } else if (returnPath[k]==4){ // turn left
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
         k++;
      }
      currentNodeDetectTime = millis();
    }
  }

  //CODE FOR DROPPING BLOCK OFF

  unsigned long prevTime = millis();
  //line follows for a bit along the line coming out the start box (for 3.5s) until going straight
  while(millis() - prevTime < 3500){
    blink();
    lineFollow();
  }
  //goes forward a bit until in the box
  // different speeds so that it goes straight
  motor1->run(FORWARD);
  motor2->run(FORWARD);
  motor1->setSpeed(255);
  motor2->setSpeed(215);
  delay(3000); // GOES FORWARD TILL FULLY IN WHITE BOX
  motor(0,0);
  delay(500);
  if (blockMagnetic == true) //turns towards the red box to put the block in
  {
    motor(3, 200);
    delay(1400);
  }
  else  // turns towards green box to put block in
  {
    motor(4, 200);
    delay(1600);
  }
  
  //delay(1400);
  motor(0,0);

  // after turning, we move forwards until at least one of the front line sensors are on
  leftBackLine = digitalRead(leftBackLinePin);
  rightBackLine = digitalRead(rightBackLinePin);
  while(leftBackLine == 0 or rightBackLine == 0){
    motor1->run(FORWARD);
    motor2->run(FORWARD);
    motor1->setSpeed(255);
    motor2->setSpeed(215); 
    leftBackLine = digitalRead(leftBackLinePin);
    rightBackLine = digitalRead(rightBackLinePin);  
  }

  leftBackLine = digitalRead(leftBackLinePin);
  rightBackLine = digitalRead(rightBackLinePin);
  // Add correction when hits white line of white box so that it is straight
  while(leftBackLine == HIGH and rightBackLine == LOW){
    motor(3,75);
    leftBackLine = digitalRead(leftBackLinePin);
    rightBackLine = digitalRead(rightBackLinePin);
    //delay(100);
  }
  // Add correction when hits white line of white box so that it is straight
  while(leftBackLine == LOW and rightBackLine == HIGH){
    motor(4,75);
    leftBackLine = digitalRead(leftBackLinePin);
    rightBackLine = digitalRead(rightBackLinePin);
    //delay(100);
  }
  motor(0,0);
  delay(1000);
/*
  if(leftBackLine == HIGH and rightBackLine == HIGH){
    motor(0,0);
    delay(1000);
  }
*/

  //goes forward until in new box
  motor1->run(FORWARD);
  motor2->run(FORWARD);
  motor1->setSpeed(240);
  motor2->setSpeed(210);
  delay(4000);
  leftBackLine = digitalRead(leftBackLinePin);
  rightBackLine = digitalRead(rightBackLinePin);
  while (leftBackLine == LOW && rightBackLine == LOW)
  {
    leftBackLine = digitalRead(leftBackLinePin);
    rightBackLine = digitalRead(rightBackLinePin);
    motor1->setSpeed(240);
    motor2->setSpeed(210);
  }
  motor(0,0);
  delay(500);
  //reverses until back at start box
  motor1->run(BACKWARD);
  motor2->run(BACKWARD);
  motor1->setSpeed(240);
  motor2->setSpeed(225);
  delay(2000);

  // continues reversing until at least one of the back line sensors detects HIGH
  leftBackLine = digitalRead(leftBackLinePin);
  rightBackLine = digitalRead(rightBackLinePin);
  while (leftBackLine == LOW && rightBackLine == LOW)
  {
    leftBackLine = digitalRead(leftBackLinePin);
    rightBackLine = digitalRead(rightBackLinePin);
    motor1->setSpeed(240);
    motor2->setSpeed(225);
  }
  motor(0,0);
  delay(500);
  // rotates until one of the front line sensors detects HIGH i.e. back almost parallel to start box white line
  leftFrontLine = digitalRead(leftFrontLinePin);
  rightFrontLine = digitalRead(rightFrontLinePin);
  while (leftFrontLine == LOW && rightFrontLine == LOW)
  {
    leftFrontLine = digitalRead(leftFrontLinePin);
    rightFrontLine = digitalRead(rightFrontLinePin);
    if (blockMagnetic == true)  // must rotate in different directions depending on whether the block just delivered was magnetic or not
    {
      motor(3,200);
    }
    else
    {
      motor(4,200);
    }
  }
  motor(0,0);
  delay(200);

  // line follows this start box line until one of the back line sensors turns HIGH
  leftBackLine = digitalRead(leftBackLinePin);
  rightBackLine = digitalRead(rightBackLinePin);
  while (leftBackLine == LOW && rightBackLine == LOW)
  {
    lineFollow();
    leftBackLine = digitalRead(leftBackLinePin);
    rightBackLine = digitalRead(rightBackLinePin);
  }
  motor(0,0);
  //delay(20000);
  
  int turnDir;  // determines the direction we need to turn depending on the state of blockMagentic
  turnDir = blockMagnetic == true ? 3 : 4;
  leftFrontLine = digitalRead(leftFrontLinePin);
  rightFrontLine = digitalRead(rightFrontLinePin);
  // turns at corner of start box until one of the front line sensors turns HIGH
  while(leftFrontLine == LOW and rightFrontLine == LOW){
    motor(turnDir, 200);
    leftFrontLine = digitalRead(leftFrontLinePin);
    rightFrontLine = digitalRead(rightFrontLinePin);
  }
  // line follows for an extra 0.2s after this to reduce error
  unsigned long delay1 = millis();
  while (millis() - delay1 < 200){
    lineFollow();
  }

  // line follows along this line until we reach the intersecting node
  leftFrontLine = digitalRead(leftFrontLinePin);
  rightFrontLine = digitalRead(rightFrontLinePin);
  leftBackLine = digitalRead(leftBackLinePin);
  rightBackLine = digitalRead(rightBackLinePin);
  while (leftBackLine == LOW or rightBackLine == LOW)
  {
    if ((blockMagnetic == true && leftBackLine == HIGH) || (blockMagnetic == false && rightBackLine == HIGH))
    {
      break;
    }
    lineFollow();
    leftBackLine = digitalRead(leftBackLinePin);
    rightBackLine = digitalRead(rightBackLinePin);
  }
  motor(0,0);
  delay(200);

  // swaps the value of turnDir and rotates for half a second
  turnDir = turnDir == 3 ? 4 : 3;
  motor(turnDir, 150);
  delay(500);
  // then continues rotating until we are incident on the line out of the start box
  leftFrontLine = digitalRead(leftFrontLinePin);
  rightFrontLine = digitalRead(rightFrontLinePin);
  while(leftFrontLine == LOW and rightFrontLine == LOW){
    motor(turnDir, 200);
    leftFrontLine = digitalRead(leftFrontLinePin);
    rightFrontLine = digitalRead(rightFrontLinePin);
  }
  motor(0,0);
  delay(200);
  
  // line follows for 0.3s to get it parallel to this line
  delay1 = millis();
  while(millis() - delay1 < 300){
    lineFollow();
  }
  motor(0,0);
  delay(500);
  // reverse until we are in start box fully
  motor(2,200);
  delay(2000);
  motor(0,0);
  delay(1000);

  if (fieldBlockCount == 1)
  {
    digitalWrite(ledB, HIGH);
    delay(6000);
  }
  else
  {
    delay(1000);
  }
  digitalWrite(ledB, LOW);

  if(fieldBlockCount < 2){
    while(leftFrontLine == LOW and rightFrontLine == LOW){
      blink();
      motor(1,200); 
      leftFrontLine = digitalRead(leftFrontLinePin);
      rightFrontLine = digitalRead(rightFrontLinePin);
    }

    //Go back to start of field
    k = 0;
    while(k<7 and toStartOfField[k] == 1){
      //get sensor readings
      blink();
      leftBackLine = digitalRead(leftBackLinePin);
      rightBackLine = digitalRead(rightBackLinePin);
      leftFrontLine = digitalRead(leftFrontLinePin);
      rightFrontLine = digitalRead(rightFrontLinePin);
      lineFollow();
      //get current time
      unsigned long currentNodeDetectTime = millis();

      //if back sensors cross line and sufficient time has passed, detect a node
      if((leftBackLine == HIGH or rightBackLine == HIGH) and currentNodeDetectTime - lastNodeDetectTime > 1600) {
        Serial.println("Node");
        lastNodeDetectTime = currentNodeDetectTime;
        currentNodeDetectTime = millis();
        //go to next instruction, wll be a turn direction in the form of a number
        k++;
        if (toStartOfField[k]==3){ // turn right
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
          currentNodeDetectTime = millis();
          //go to next instruction, will be line following (1)
          k++;
        } else if (toStartOfField[k]==4){ // turn left
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
          k++;
        }
        currentNodeDetectTime = millis();
      }
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