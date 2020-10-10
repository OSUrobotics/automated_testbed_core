//necessary libraries
#include <AccelStepper.h>
#include <MultiStepper.h>
#include "Adafruit_VL53L0X.h"
#include <ros.h>
#include <std_msgs/Int32.h>

//reset motor pins
#define pulse_reset 3
#define direction_reset 2

//friction motor pins
#define pulse_friction 9
#define direction_friction 8

//FSR402 data pins (from handle)
#define fsr_1 A0
#define fsr_2 A1
#define fsr_3 A2
#define fsr_4 A3
#define fsr_5 A4
#define fsr_6 A5
#define fsr_7 A6
#define fsr_8 A7
#define fsr_9 A8
#define fsr_10 A9
#define fsr_11 A10
#define fsr_12 A11

AccelStepper reset_motor(1, pulse_reset, direction_reset);
AccelStepper friction_motor(1, pulse_friction, direction_friction);

Adafruit_VL53L0X tof = Adafruit_VL53L0X();
int start_pos; //distance of drawer from tof in starting pos, the '0' position.
const int buffer_val = 5; //buffer value for tof sensor for calculating distance
const float fric_steps = .00032; //constant that represents relation between friction setting to motor steps
const float base_friction = .3; //min resistance that drawer has
const float min_steps = 2500; //min steps it takes to get brake to touch drawer fin
float force_input; //input from user to set resistance rating
char junk = ' ';

const int time_unwind = 4000; //in ms
unsigned long time;
unsigned long time_stop;
unsigned long start_time;

//ros variables
ros::NodeHandle n;
int data_collection_flag = 0;
std_msgs::Int32 reset_answer;
std_msgs::Int32 data_point;
ros::Publisher pub("reset_complete", &data_point);
ros::Publisher datapub("door_data", &data_point);

//ros callback functions for data_start and reset_start topics 
void reset_callback(const std_msgs::Int32& req){
  data_collection_flag = 0;
  reset_answer = req;
}
void data_callback(const std_msgs::Int32& req){
  data_collection_flag = 1;
  force_input = req.data; //var sent to enable data collection is value for force_input
}

ros::Subscriber<std_msgs::Int32> sub("reset_start", &reset_callback);
ros::Subscriber<std_msgs::Int32> datasub("collection_start", &data_callback);

void setup() {
  Serial.begin(57600);
  n.initNode();
  n.subscribe(sub);
  n.subscribe(datasub);
  n.advertise(pub);
  n.advertise(datapub);
  //don't run drawer if TOF is not working.
  //TODO: send error message to ROS here
  if (!tof.begin()) {
    while (1){
      n.spinOnce();
      data_point.data = -10;
      datapub.publish(&data_point);
      delay(1);
    }
  }

  pinMode(pulse_reset, OUTPUT);
  pinMode(direction_reset, OUTPUT);
  pinMode(pulse_friction, OUTPUT);
  pinMode(direction_friction, OUTPUT);
  friction_motor.setMaxSpeed(20000);
  reset_motor.setMaxSpeed(20000);

  pinMode(fsr_1, INPUT);
  pinMode(fsr_2, INPUT);
  pinMode(fsr_3, INPUT);
  pinMode(fsr_4, INPUT);
  pinMode(fsr_5, INPUT);
  pinMode(fsr_6, INPUT);
  pinMode(fsr_7, INPUT);
  pinMode(fsr_8, INPUT);
  pinMode(fsr_9, INPUT);
  pinMode(fsr_10, INPUT);
  pinMode(fsr_11, INPUT);
  pinMode(fsr_12, INPUT);
}

void loop() {
  while(!data_collection_flag){
    n.spinOnce();
    delay(1);
  }
  
  set_friction(force_input);
  VL53L0X_RangingMeasurementData_t measure; //value from tof sensor
  tof.rangingTest(&measure, false);
  start_pos = measure.RangeMilliMeter; // initialize starting pos of drawer. Assumes drawer is closed.
  start_time = millis();

  while (data_collection_flag) {
    n.spinOnce();
    collect_data();
  }

  n.spinOnce();
  reset_friction();
  reset_drawer(measure);
  pub.publish(&reset_answer);
}

void reset_drawer(VL53L0X_RangingMeasurementData_t &measure) {
  reset_motor.setSpeed(-5000); //set speed to negative value to change direction
  bool did_move = false;
  while (true) {
    tof.rangingTest(&measure, false);
    if (measure.RangeMilliMeter < (start_pos + buffer_val)) {
      break;
    }
    did_move = true;
    time = millis();
    time_stop = time + 100; //runs motor for 100ms before checking if drawer is closed
    while (time < time_stop) {
      n.spinOnce();
      reset_motor.runSpeed();
      time = millis();
    }
  }
  if (did_move) { //if drawer did not move at all (or did not need to be reeled in), don't unwind
    time = millis();
    time_stop = time + time_unwind;
    reset_motor.setSpeed(5000);
    while (time < time_stop) { //unwinds motor so string has slack
      n.spinOnce();
      reset_motor.runSpeed();
      time = millis();
    }
  }
}

void set_friction(float resistance) {
  float steps = ((resistance - base_friction) / fric_steps) + min_steps;
  friction_motor.setSpeed(5000);
  friction_motor.moveTo(steps);
  friction_motor.setSpeed(5000);
  do {
    friction_motor.runSpeed();
    n.spinOnce();
  } while (friction_motor.currentPosition() < friction_motor.targetPosition());
}

void reset_friction() {
  friction_motor.setSpeed(-5000);
  friction_motor.moveTo(0);
  friction_motor.setSpeed(-5000);
  do {
    friction_motor.runSpeed();
    n.spinOnce();
  } while (friction_motor.currentPosition() > friction_motor.targetPosition());
}

void read_handle_val() {
  data_point.data = analogRead(fsr_1);
  datapub.publish(&data_point);
  data_point.data = analogRead(fsr_2);
  datapub.publish(&data_point);
  data_point.data = analogRead(fsr_3);
  datapub.publish(&data_point);
  data_point.data = analogRead(fsr_4);
  datapub.publish(&data_point);
  data_point.data = analogRead(fsr_5);
  datapub.publish(&data_point);
  data_point.data = analogRead(fsr_6);
  datapub.publish(&data_point);
  data_point.data = analogRead(fsr_7);
  datapub.publish(&data_point);
  data_point.data = analogRead(fsr_8);
  datapub.publish(&data_point);
  data_point.data = analogRead(fsr_9);
  datapub.publish(&data_point);
  data_point.data = analogRead(fsr_10);
  datapub.publish(&data_point);
  data_point.data = analogRead(fsr_11);
  datapub.publish(&data_point);
  data_point.data = analogRead(fsr_12);
  datapub.publish(&data_point);
}

void read_TOF_val() {
  VL53L0X_RangingMeasurementData_t measure; //value from tof sensor. pointer
  tof.rangingTest(&measure, false);
  int drawer_distance;
  if (measure.RangeMilliMeter <= start_pos) {
    drawer_distance = 0;
   }
  else {
   drawer_distance = measure.RangeMilliMeter - start_pos;
  }
  data_point.data = drawer_distance;
  datapub.publish(&data_point);
}

//outputs negative values as placeholders for fsr13 and fsr14.
//Used so we can use the same custom message for the door and drawer
void dummy_val()  {
  data_point.data = -1;
  datapub.publish(&data_point);
  datapub.publish(&data_point);
}

void collect_data() {
  read_TOF_val();
  read_handle_val();
  dummy_val();
  time = millis() - start_time;
  data_point.data = time;
  datapub.publish(&data_point);
}
