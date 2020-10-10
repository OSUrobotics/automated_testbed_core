/*
 * Order of csv file: Distance drawer is out, time, fsr1, fsr2, ..., fsr12
 */

#include <AccelStepper.h>
#include <MultiStepper.h>
#include "Adafruit_VL53L0X.h"

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

void set_friction(float resistance) {
  float steps = ((resistance - base_friction) / fric_steps) + min_steps;
  //int starting_pos = friction_motor.currentPosition();
  // Serial.print("Starting Position: "); Serial.print(starting_pos); Serial.println();
  friction_motor.setSpeed(5000);
  friction_motor.moveTo(steps);
  friction_motor.setSpeed(5000);
  // Serial.print("Target Position: "); Serial.print(friction_motor.targetPosition()); Serial.println();
  // delay(2000);
  do {
    friction_motor.runSpeed();
  } while (friction_motor.currentPosition() < friction_motor.targetPosition());
}

void read_handle_val() {
  Serial.println(analogRead(fsr_1));
  Serial.println(analogRead(fsr_2)); 
  Serial.println(analogRead(fsr_3)); 
  Serial.println(analogRead(fsr_4));
  Serial.println(analogRead(fsr_5));
  Serial.println(analogRead(fsr_6));
  Serial.println(analogRead(fsr_7));
  Serial.println(analogRead(fsr_8));
  Serial.println(analogRead(fsr_9));
  Serial.println(analogRead(fsr_10));
  Serial.println(analogRead(fsr_11));
  Serial.println(analogRead(fsr_12));
}

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ;
  }
  if (!tof.begin()) { //if tof isn't connected, don't continue program
    Serial.println(F("Failed to boot Time of Flight Sensor (VL53L0X)"));
    while (1);
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
  set_friction(.5); // <--- Set different frictin settings here
}

void loop() {
  VL53L0X_RangingMeasurementData_t measure; //value from tof sensor
  tof.rangingTest(&measure, false);
  start_pos = measure.RangeMilliMeter; // initialize starting pos of drawer
  time = millis();
  time_stop = time + 15000;
  while (true) { // main loop for getting fsr values and pos of drawer
    time = millis();
    tof.rangingTest(&measure, false);
    if (measure.RangeMilliMeter <= start_pos) {
      Serial.println("0"); //for script
    }
    else {
      Serial.println((measure.RangeMilliMeter - start_pos)); //for script
    }
    Serial.println(time); //for script
    read_handle_val();
    delay(50);
  }
}
