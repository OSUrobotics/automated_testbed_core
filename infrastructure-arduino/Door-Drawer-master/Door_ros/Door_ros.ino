// Include the necessary libraries
#include "math.h" //Includes math library
#include <Adafruit_MCP3008.h> //Includes library for SPI communications
#include "Adafruit_VL53L0X.h" //library for time of fligh sensor (tof)
#include <ros.h>
#include <std_msgs/Int32.h>

//define pins for motor controller
#define enable_motor_channel 6 //pwm pin
#define motor_channel3  48
#define motor_channel4  42

// define pins for relay/electromagnet
#define relay25 44 //44
#define relay35 35 //35
#define relay45 33 //33

// Initialize FSR and Potentiometer in door frame
#define fsr_13 A0
#define fsr_14 A2
#define fsr_1 A7
#define fsr_2 A15
#define fsr_3 A10
#define fsr_4 A4
#define fsr_5 A12
#define fsr_6 A6
#define fsr_7 A5
#define fsr_8 A13
#define fsr_9 A8
#define fsr_10 A14
#define fsr_11 A9
#define fsr_12 A11

// Initialize the SPI devices
Adafruit_MCP3008 adc1; //SPI 1
Adafruit_MCP3008 adc2; //SPI 2

//tof sensor
Adafruit_VL53L0X tof = Adafruit_VL53L0X();
float door_angle;

// Initialize user input variables
int magnet_input;
unsigned long time;
unsigned long stoptime;
unsigned long start_time;

//declare motor variable for time
const int time_unwind = 2000; // in ms

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
  magnet_input = req.data; //var sent to enable data collection is value for magnet_input
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
  //don't run door if TOF is not working.
  //TODO: send error message to ROS here
  if (!tof.begin()) {
    while (1){
      n.spinOnce();
      data_point.data = -10;
      datapub.publish(&data_point);
      delay(1);
    }
  }

  // Begins the SPI devices and declares the pins in order of (CLK, MOSI, MISO, CS)
  adc1.begin(52, 51, 50, 53);
  adc2.begin(9, 11, 10, 12);
  // initialize motor pins as outputs
  pinMode(motor_channel3, OUTPUT);
  pinMode(motor_channel4, OUTPUT);
  pinMode(enable_motor_channel, OUTPUT);
  // Initializes electromagnet relay pins as outputs
  pinMode(relay25, OUTPUT);
  pinMode(relay35, OUTPUT);
  pinMode(relay45, OUTPUT);
  //initialize fsr's
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
  pinMode(fsr_13, INPUT);
  pinMode(fsr_14, INPUT);
  //pinMode(fsr_15, INPUT);
  //pinMode(fsr_16, INPUT);
}

void loop() {
  while(!data_collection_flag){
    n.spinOnce();
    delay(1);
  }
 
  Enable_Relays(magnet_input); // changes electromagnets based on input
  //bool switched_off = false;
  //bool switched_on = true;
  start_time = millis();
  while (data_collection_flag) {
    n.spinOnce();
    collect_data();
    //Turns off electromagnets when not being used.
    //saves energy and reduces heat from electromagnets.
    //TODO: this needs a delay or else it switches too fast for mechanical relay to keep up
    /*if (door_angle > 15 && !switched_off) {
      Enable_Relays(0);
      switched_on = false;
      switched_off = true;
    }
    else if (door_angle <= 15 && !switched_on) {
      Enable_Relays(magnet_input);
      switched_off = false;
      switched_on = true;
    }*/
  }

  n.spinOnce();
  Reset_Door(); // automatically close door
  pub.publish(&reset_answer); //publishes that reset is complete
}

void Reset_Door() {
  bool did_move = false;
  int motor_speed = 100; //max speed for the motor
  Enable_Relays(0); // turns off magnets. makes motor faster
  analogWrite(enable_motor_channel, motor_speed); //turns motor on
  digitalWrite(motor_channel3, LOW);// turns motor
  digitalWrite(motor_channel4, HIGH); // counter clockwise
  while (true) { // door is open more than 1 degree
    n.spinOnce();
    VL53L0X_RangingMeasurementData_t measure; //value from tof sensor
    tof.rangingTest(&measure, false);
    door_angle = calc_degree(measure.RangeMilliMeter);
    if (door_angle < 1) {
      break;
    }
    did_move = true;
  }
  if (did_move) {
    time = millis();
    unsigned long time_stop = time + time_unwind;
    while(time < (time_stop)){
      digitalWrite(motor_channel3, HIGH); // turns motor
      digitalWrite(motor_channel4, LOW);// clockwise
      n.spinOnce();
      time=millis();
      //testing purposes only
      data_point.data = -5;
      datapub.publish(&data_point);
    }
  }
  analogWrite(enable_motor_channel, 0); // turns motor off
}

void Enable_Relays(int user_in) {
  int relay25_val = 0;
  if (user_in == 0) {
    digitalWrite(relay25, relay25_val);
    digitalWrite(relay35, LOW);
    digitalWrite(relay45, LOW);
  }
  else if (user_in == 1) {
    digitalWrite(relay25, (relay25_val + 1));
    digitalWrite(relay35, LOW);
    digitalWrite(relay45, LOW);
  }
  else if (user_in == 2) {
    digitalWrite(relay25, relay25_val);
    digitalWrite(relay35, HIGH);
    digitalWrite(relay45, LOW);
  }
  else if (user_in == 3) {
    digitalWrite(relay25, relay25_val);
    digitalWrite(relay35, LOW);
    digitalWrite(relay45, HIGH);
  }
  else if (user_in == 4) {
    digitalWrite(relay25, (relay25_val + 1));
    digitalWrite(relay35, HIGH);
    digitalWrite(relay45, LOW);
  }
  else if (user_in == 5) {
    digitalWrite(relay25, (relay25_val + 1));
    digitalWrite(relay35, LOW);
    digitalWrite(relay45, HIGH);
  }
  else if (user_in == 6) {
    digitalWrite(relay25, relay25_val);
    digitalWrite(relay35, HIGH);
    digitalWrite(relay45, HIGH);
  }
  else if (user_in == 7) {
    digitalWrite(relay25, (relay25_val + 1));
    digitalWrite(relay35, HIGH);
    digitalWrite(relay45, HIGH);
  }
}

float calc_degree(int distance) {
  float D0 = 289; // value of distance from tof when door is closed
  float D1 = 418; //value of distance from tof when door is fully open
  return (distance - D0) / ((D1 - D0) / 90); // 90 is max degree door can open from closed position
}

void read_handle_val() {
  //pub.publish(analogRead(fsr_1)); will this work?
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
  door_angle = calc_degree(measure.RangeMilliMeter); 
  if (door_angle < 0) {
    door_angle = 0;
  }
  data_point.data = door_angle;
  datapub.publish(&data_point);
}

void read_pull_force() {
  data_point.data = analogRead(fsr_13);
  datapub.publish(&data_point);
  data_point.data = analogRead(fsr_14);
  datapub.publish(&data_point);
}

void collect_data(){
    read_TOF_val();
    read_handle_val();
    read_pull_force();
    time = millis() - start_time;
    data_point.data = time;
    datapub.publish(&data_point);
}
