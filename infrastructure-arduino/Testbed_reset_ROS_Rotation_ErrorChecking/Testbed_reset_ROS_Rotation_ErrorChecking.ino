// Created 7/8/2020 By: Josh Campbell, Based off Grant Foster's code
// Code written for an Arduino Mega / Mega 2560

#include <AccelStepper.h>
#include <Servo.h>
#include<ros.h>
#include<messages/TestInfo.h>
#include<std_msgs/Int32.h>

//ROS variables
ros::NodeHandle n;
std_msgs::Int32 answer;
int rotation = 0;
std_msgs::Int32 report;
ros::Publisher pub("reset_complete", &answer);

//ROS callback function and subscriber declaration
void callback(const std_msgs::Int32& copy){
  rotation = copy.data;
  report.data = reset();
  
  if(report.data != 0){
    pub.publish(&report);
    Serial.println("Exit Status: " + String(report.data));
  }
  else{
    answer.data = 0;
    pub.publish(&answer);
  }
   
}
ros::Subscriber<std_msgs::Int32> sub("reset_start", &callback);

// Above is stuff for ROS communication 
/******************************************************************************/

//Stepper pins
int conePUL = 8;
int coneDIR = 9;
int stringPUL = 11;
int stringDIR = 12;

int noSlack = 0; //prevents too much slack
int testRun = 0; //skips rotation when 

// Turn Table motors
int tablePWM = 7;
int tableIn1 = 24; //output pin
int tableIn2 = 25; //output pin
int tableEN = 23; //Determines if motor driver is enabled

int millisecPerDeg = 0; //callibrated in setup then used to rotate turn table to an angle

// Sensor pins & variables
int limitPin = 14;
int photoPin = 10;
int tableHallPin = 22;
int tableHallVal;
int objectStartingRotation = 0;


AccelStepper ConeStepper(1, conePUL, coneDIR);
AccelStepper StringStepper(1, stringPUL, stringDIR);
Servo tableMotor;

void setup() {
  //initializes ros node and pub/subs
  n.initNode();
  n.subscribe(sub);
  n.advertise(pub);
  
  //stepper motors inialize
  ConeStepper.setMaxSpeed(50000);
  ConeStepper.setSpeed(0);
  StringStepper.setMaxSpeed(50000);
  StringStepper.setSpeed(0);

  Serial.begin(57600);

  pinMode(tableHallPin, INPUT);
  //sets motor pin as a servo to write PWM 
  tableMotor.attach(tablePWM);

  reset();
  //calibrateTurnTable();
  testRun = 1;
}

bool coneUp() {
  int initialTime = millis();
  int dt = 0;
  
  ConeStepper.setSpeed(20000);
  
  while (analogRead(limitPin) <= 1020 && dt <= 20000){
    ConeStepper.runSpeed();
    n.spinOnce(); //Keeps ROS from losing connection
    dt = initialTime + millis();
  }
  ConeStepper.setSpeed(0);
  ConeStepper.runSpeed();
  
  if (dt >= 20000) {
    return 1;
  }
  else{
    return 0;
  }
}

bool coneDown() {
  int initialTime = millis();
  int dt = 0;
  
  ConeStepper.setSpeed(-20000);
  
  while (dt <= 2000) {
    ConeStepper.runSpeed();
    dt = millis() - initialTime;
    n.spinOnce(); //Keeps ROS from losing connection
  }
  ConeStepper.setSpeed(0);
  ConeStepper.runSpeed();
  
  if (dt >= 2000) {
    return 1;
  }
  else{
    return 0;
  }
}

bool spinReelIn() {
  int dt = 0;
  int initialTime = millis();
  
  StringStepper.setSpeed(40000);
  
  while (dt <= 30000 && analogRead(photoPin) !=0) {
    StringStepper.runSpeed();
     // Serial.println("spinReelIn");
    dt = millis() - initialTime;
    n.spinOnce(); //Keeps ROS from losing connection
  }

  if (dt <= 100) {
    noSlack = 1;
  }

  StringStepper.setSpeed(0);

  if (dt >= 20000) {
    return 1;
  }
  else{
    return 0;
  }
}

bool spinReelOut() {
  int dt = 0;
  int initialTime = millis();
  StringStepper.setSpeed(-20000);
  while (dt <= 5000 && noSlack != 1) {
    StringStepper.runSpeed();
    dt = millis() - initialTime;
    n.spinOnce(); //Keeps ROS from losing connection
  }
  StringStepper.setSpeed(0);
  noSlack = 0;
  
  if (dt >= 5000) {
    return 1;
  }
  else{
    return 0;
  }
}

/***************************************************************************************************/
// Turn table functions

bool resetTableRotation() {
  int dt = 0;
  int initialTime = millis();

  while (digitalRead(tableHallPin) != 0 && dt <= 30000) {
    tableRotateCW();
    dt = millis() - initialTime;
    n.spinOnce(); //Keeps ROS from losing connection
  }
  
  if (dt <= 1000) {
   while (dt <= 2000 or digitalRead(tableHallPin) != 0) {
    tableRotateCW();
    dt = millis() - initialTime;
    n.spinOnce(); //Keeps ROS from losing connection
    }
  }
  tableBrake();
 
  if (dt >= 20000) {
    return 1;
  }
  else{
    return 0;
  }
}

bool calibrateTurnTable() {
  int dt = 0;
  int initialTime = millis();

// rotates object pass the hall effect sensor
  while(dt <= 4000) {
    tableRotateCW();
    dt = millis() - initialTime;
    n.spinOnce();
  }

  //rotates the object until its back at home
  while (digitalRead(tableHallPin) != 0 && dt <= 40000) {
    tableRotateCW();
    dt = millis() - initialTime;
    n.spinOnce();
  }
  tableBrake();
  millisecPerDeg = int(dt / 360);

  if (dt >= 20000) {
    return 1;
  }
  else{
    return 0;
  }
}

bool tableRotation(int angle = 0){
  int dt = 0;
  int initialTime = millis();
  
  if (angle >=0){
    int rotateTime = millisecPerDeg * angle; 
    while (dt <= rotateTime) {
    tableRotateCW();
    dt = millis() - initialTime;
    n.spinOnce();   
    }
  }
  else {
    int rotateTime = millisecPerDeg * (180 + angle);
    while (dt <= rotateTime) {
    tableRotateCCW();
    dt = millis() - initialTime;
    n.spinOnce();    
    }
  }
  tableBrake();
   
  if (dt >= 20000) {
    return 1;
  }
  else{
    return 0;
  }
}


void tableRotateCW() {
  n.spinOnce(); //Keeps ROS from losing connection
  digitalWrite(tableEN,HIGH);
  digitalWrite(tableIn1, HIGH);
  digitalWrite(tableIn2, LOW);
  tableMotor.write(90);
  
}

void tableRotateCCW() {
  n.spinOnce(); //Keeps ROS from losing connection
  digitalWrite(tableEN,HIGH);
  digitalWrite(tableIn1, LOW);
  digitalWrite(tableIn2, HIGH);
  tableMotor.write(90);
  n.spinOnce(); //Keeps ROS from losing connection
}

void tableBrake() {
  n.spinOnce(); //Keeps ROS from losing connection
  digitalWrite(tableIn1,HIGH);
  digitalWrite(tableIn2,HIGH);
  delay(200);
  
}

//Main code
/******************************************************************************/
int reset() {
  bool upCone = coneUp();
  if (upCone == 1){return 001;}
  delay(50);
  bool reelIn = spinReelIn();
  if (reelIn == 1){return 002;}
  n.spinOnce(); //Keeps ROS from losing connection
  delay(500);
  n.spinOnce(); //Keeps ROS from losing connection
  bool reelOut = spinReelOut();
  if (reelOut == 1){return 003;}
  delay(50);
  bool downCone = coneDown();
  if (downCone == 1){return 004;}
  delay(500);
  bool resetRotation = resetTableRotation();
  if (resetRotation == 1){return 005;}
  delay(50);
  bool tableRotate = tableRotation(rotation);
  if (tableRotate == 1){return 006;}
}
  

void loop() {
    n.spinOnce(); //Keeps ROS from losing connection
    delay(1);
    Serial.println("loop func");
}
