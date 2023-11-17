#include <Adafruit_MotorShield.h>

#define MAX_RANG (520)//the max measurement value of the module is 520cm(a little bit longer
//than effective max range)
#define ADC_SOLUTION (1023.0)//ADC accuracy of Arduino UNO is 10bit

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motor1 = AFMS.getMotor(3);
Adafruit_DCMotor *motor2 = AFMS.getMotor(2);
bool moving = false;
bool detecting = false;
bool gridMode = false;
bool blockMagnetic = true;

int leftFrontLinePin = 0;
int rightFrontLinePin = 1; 
int leftBackLinePin = 2;
int rightBackLinePin = 3;
int ledR = 4;
int ledG = 5;
int ledB = 6;
int magnetPin = 7;
int ultraSoundPin = A0; //analogue read
float dist_t, sensity_t;

int magnet = 0;
int v3 = 0;
unsigned long previousTime = millis();
unsigned long previousUltraSoundTime = millis();
unsigned long lastNodeDetectTime = millis();
int ledState = LOW;

int leftFrontLine = 0;
int rightFrontLine = 0;
int leftBackLine = 0;
int rightBackLine = 0;

int ultraSoundDistance = 100;
int timeOfFlightDistance = 0;
//vector <int> ultraSoundReadings;

int smallLoop[23] = {1,1,4,1,1,3,1,3,1,1,1,1,3,1,3,1,1,3,1,4,1,1,3};
int bigLoop[9] = {1,3,1,3,1,3,1,1,1};

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
  
  if(gridMode){
    grid();
  } else {
    field();
  }
}

int ultrasound_average(){
    sensity_t = analogRead(ultraSoundPin);
    dist_t = sensity_t * MAX_RANG / ADC_SOLUTION;//
    /*
    ultraSoundReadings.pushback(dist_t);
    auto it = ultraSoundReadings.begin(); 
    ultraSoundReadings.erase(it);
    int sum = 0
    for (int value : ultraSoundReadings){
      sum += value
    }
    Serial.println(ultraSoundReadings)
    return static_cast<double>(sum) / ultraSoundReadings.size();
    */
    return dist_t;
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
    else if(leftFrontLine == LOW and rightFrontLine == HIGH){
        motor1->run(FORWARD);
        motor2->run(FORWARD);
        motor2->setSpeed(70);
        motor1->setSpeed(225);
        moving = true;
     }
    else if(leftFrontLine == HIGH and rightFrontLine == LOW){
        motor1->run(FORWARD);
        motor2->run(FORWARD);
        motor2->setSpeed(225);
        motor1->setSpeed(70);
        moving = true;
    }
    /*
    unsigned long currentUltraSoundTime = millis();
    if(detecting == true and (currentUltraSoundTime - previousUltraSoundTime > 100)) {
      previousUltraSoundTime = currentUltraSoundTime;
      ultraSoundDistance = ultrasound_average();
      Serial.println("Ultrasound reading: ");
      Serial.print(ultraSoundDistance);
      Serial.println(" ");
      if(ultraSoundDistance < 60){
        collectBlockInField();
      }
    }
    */
}

void lock_onto_block(){
  //timeOfFlight = digitalRead(timeOfFlight);


}

//{1,3,1,1,4,1,4,1,1,1,1,1,4,1,4,1,1}

void grid(){
  //assumes starting in starting box facing towards field
  // NEXT: implement scanning for blocks en route, dropping off blocks, ending up in centre of long edge of field after both blocks dropped off
  int i = 0;
  Serial.println("While loop");
  while(i < 23 and smallLoop[i] == 1){
    leftBackLine = digitalRead(leftBackLinePin);
    rightBackLine = digitalRead(rightBackLinePin);
    leftFrontLine = digitalRead(leftFrontLinePin);
    rightFrontLine = digitalRead(rightFrontLinePin);
    lineFollow();
    unsigned long currentNodeDetectTime = millis();

    if((leftBackLine == HIGH or rightBackLine == HIGH) and currentNodeDetectTime - lastNodeDetectTime > 1600) {
      lastNodeDetectTime = currentNodeDetectTime;
      currentNodeDetectTime = millis();
      //Serial.print(currentNodeDetectTime);
      //Serial.print(" ");
      //Serial.print(smallLoop[i]);
      //Serial.println(" ");
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
          currentNodeDetectTime = millis();
          //Serial.print(currentNodeDetectTime);
          //Serial.print(" ");
          //Serial.print(smallLoop[i]);
          //Serial.println(" ");
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
         currentNodeDetectTime = millis();
         //Serial.print(currentNodeDetectTime);
         //Serial.print(" ");
         //Serial.print(smallLoop[i]);
         //Serial.println(" ");
         i++;
      }
      currentNodeDetectTime = millis();
    }
  }
  gridMode = false;
}

/*
void scan_field(){
  //assumes starting at centre of edge of field
  while(leftFrontLine == LOW or rightFrontLine == LOW){
    motor(4,200); 
    leftFrontLine = digitalRead(leftFrontLinePin);
    rightFrontLine = digitalRead(rightFrontLinePin);
  } 
  //scan from left to right
  while(leftFrontLine == HIGH and rightFrontLine == HIGH and ultraSoundDistance > 170){
    motor(3,100); //slower than usual turn
    leftFrontLine = digitalRead(leftFrontLinePin);
    rightFrontLine = digitalRead(rightFrontLinePin);
  } while(leftFrontLine == LOW or rightFrontLine == LOW and ultraSoundDistance > 170){
    motor(3,100);
    leftFrontLine = digitalRead(leftFrontLinePin);
    rightFrontLine = digitalRead(rightFrontLinePin);
  }
}
*/

//{1,3,1,3,1,3,1,1,1}

void field(){
  int i = 0;
  Serial.println("field");
  while(i < 8 and bigLoop[i] == 1){
    

    leftBackLine = digitalRead(leftBackLinePin);
    rightBackLine = digitalRead(rightBackLinePin);
    leftFrontLine = digitalRead(leftFrontLinePin);
    rightFrontLine = digitalRead(rightFrontLinePin);
    lineFollow();
    
    if (detecting == true){
      sensity_t = analogRead(ultraSoundPin);
      dist_t = sensity_t * MAX_RANG / ADC_SOLUTION;
      if(dist_t < 6){
        Serial.println("break");
        break;
      }
    }
    

    /*unsigned long currentUltraSoundTime = millis();
    if(detecting == true and (currentUltraSoundTime - previousUltraSoundTime > 100)) {
      previousUltraSoundTime = currentUltraSoundTime;
      ultraSoundDistance = ultrasound_average();
      Serial.println("Ultrasound reading: ");
      Serial.print(ultraSoundDistance);
      Serial.println(" ");
      if(ultraSoundDistance < 60){
        break;
      }
    }*/

    unsigned long currentNodeDetectTime = millis();
    if((leftBackLine == HIGH or rightBackLine == HIGH) and currentNodeDetectTime - lastNodeDetectTime > 1600) {
      lastNodeDetectTime = currentNodeDetectTime;
      currentNodeDetectTime = millis();
      //Serial.print(currentNodeDetectTime);
      //Serial.print(" ");
      //Serial.print(bigLoop[i]);
      //Serial.println(" ");
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
          
          //Serial.print(currentNodeDetectTime);
          //Serial.print(" ");
          //Serial.print(bigLoop[i]);
          //Serial.println(" ");
          i++;
          if(i == 2){
            detecting = true; 
            Serial.println("Detecting");
          } else{
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
         delay(2000);
         currentNodeDetectTime = millis();
         //Serial.print(currentNodeDetectTime);
         //Serial.print(" ");
         //Serial.print(bigLoop[i]);
         //Serial.println(" ");
         i++;
      }
      currentNodeDetectTime = millis();
    }
  } 
  collectBlockInField();
}


void collectBlockInField(){
  Serial.println("COLLECTING BLOCK");
  //turns a bit
  while(leftBackLine == LOW and rightBackLine == LOW){
    motor(3,200); 
    leftBackLine = digitalRead(leftBackLinePin);
    rightBackLine = digitalRead(rightBackLinePin);
  }
  motor(0,0);

  

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