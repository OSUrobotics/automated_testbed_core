//this version of the code is used for testing the Door with the Arduino Serial Monitor. Thus, the UI is through the serial monitor

// Include the necessary libraries
#include "math.h" //Includes math library
#include <Adafruit_MCP3008.h> //Includes library for SPI communications
#include "Adafruit_VL53L0X.h" //library for time of fligh sensor (tof)

//define pins for motor controller
#define enable_motor_channel 6 //pwm pin
#define motor_channel3  48
#define motor_channel4  42

// define pins for relay/electromagnet
#define relay25 35
#define relay35 33
#define relay45 44

// Initialize FSR and Potentiometer in door frame
#define fsr_13 A0
#define fsr_14 A1
#define fsr_15 A2
#define fsr_16 A3
#define fsr_1 A7 //A7
#define fsr_2 A15 //A15
#define fsr_3 A10 //A10
#define fsr_4 A4 //A4
#define fsr_5 A12 //A12
#define fsr_6 A5 //TBA
#define fsr_7 A6 //TBA
#define fsr_8 A13 //TBA
#define fsr_9 A8 //A8
#define fsr_10 A14 //A14
#define fsr_11 A9 //A9
#define fsr_12 A11 //A11

// Initialize the SPI devices
Adafruit_MCP3008 adc1; //SPI 1
Adafruit_MCP3008 adc2; //SPI 2

//tof sensor
Adafruit_VL53L0X tof = Adafruit_VL53L0X();
float door_angle;

// Initialize user input variables
float magnet_input;
float time_input;
char junk = ' ';
unsigned long time;
unsigned long stoptime;

//declare motor variable for time
const int time_unwind = 2000; // in ms

void setup() {
  // Initialize the Serial monitor
  Serial.begin(115200);
  while (!Serial){
    ;
  }
  //don't run door if TOF is not working
  if (!tof.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while (1);
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
  pinMode(fsr_15, INPUT);
  pinMode(fsr_16, INPUT);
}

void loop() {
  //Temporary Interface for Door testing purposes.
  get_magnet_val();
  get_trial_length();
  
  Enable_Relays(magnet_input); // changes electromagnets based on input
  bool switched_off = false;
  bool switched_on = true;
  time = millis();
  stoptime = time + (time_input * 1000); // converts time_input to seconds
  while (time < stoptime) {
    read_TOF_val();
    read_handle_val();
    //read_pull_force();
    Serial.println(time);
    //Turns off electromagnets when not being used.
    //saves energy and reduces heat from electromagnets.
    if (door_angle > 15 && !switched_off) {
      Enable_Relays(0);
      switched_on = false;
      switched_off = true;
    }
    else if (door_angle <= 15 && !switched_on) {
      Enable_Relays(magnet_input);
      switched_off = false;
      switched_on = true;
    }

    delay(50);
    time = millis();
  }
  Reset_Door(); // automatically close door
}

void Reset_Door() {
  bool did_move = false;
  int motor_speed = 100; //max speed for the motor
  Enable_Relays(0); // turns off magnets; makes motor faster
  analogWrite(enable_motor_channel, motor_speed); //turns motor on
  digitalWrite(motor_channel3, LOW);// turns motor
  digitalWrite(motor_channel4, HIGH); // counter clockwise
  while (true) { // door is open more than 5 degrees
    VL53L0X_RangingMeasurementData_t measure; //value from tof sensor
    tof.rangingTest(&measure, false);
    door_angle = calc_degree(measure.RangeMilliMeter);
    if (door_angle < 1) {
      break;
    }
    did_move = true;
    delay(100);
  }
  if (did_move) {
    digitalWrite(motor_channel3, HIGH); // turns motor
    digitalWrite(motor_channel4, LOW);// clockwise
    delay(time_unwind); //run for certain amount of time
  }
  analogWrite(enable_motor_channel, 0); // turns motor off
}

void Enable_Relays(int user_in) {
  int relay25_val = 0; //for testing purposes
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

void get_magnet_val() {
  Serial.print("Enter desired force rating on a scale of 0 to 7: ");
  while (Serial.available() == 0); {
    magnet_input = Serial.parseFloat();
    Serial.print(magnet_input, 0);
    Serial.print("\n");
    while (Serial.available() > 0) {
      junk = Serial.read();
    }
  }
}

void get_trial_length() {
  Serial.print("Enter How long you would like to run each test (in seconds): ");
  while (Serial.available() == 0); {
    time_input = Serial.parseFloat();
    Serial.print(time_input, 0);
    Serial.print("\n");
    while (Serial.available() > 0) {
      junk = Serial.read();
    }
  }
}

void read_handle_val() {
  /*convert_force(analogRead(fsr_1)); Serial.print(" ");
  convert_force(analogRead(fsr_2)); Serial.print(" ");
  convert_force(analogRead(fsr_3)); Serial.print(" ");
  convert_force(analogRead(fsr_4)); Serial.print(" ");
  convert_force(analogRead(fsr_5)); Serial.print(" ");
  convert_force(analogRead(fsr_6)); Serial.print(" ");
  convert_force(analogRead(fsr_7)); Serial.print(" ");
  convert_force(analogRead(fsr_8)); Serial.print(" ");
  convert_force(analogRead(fsr_9)); Serial.print(" ");
  convert_force(analogRead(fsr_10)); Serial.print(" ");
  convert_force(analogRead(fsr_11)); Serial.print(" ");
  convert_force(analogRead(fsr_12)); Serial.print(" ");*/
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

/*void read_pull_force() {
  convert_force(analogRead(fsr_13));
  Serial.print("\t");
  convert_force(analogRead(fsr_14));
  Serial.print("\t");
  convert_force(analogRead(fsr_15));
  Serial.print("\t");
  convert_force(analogRead(fsr_16));
  Serial.print("\t");
}*/

void read_TOF_val() {
  VL53L0X_RangingMeasurementData_t measure; //value from tof sensor. pointer
  tof.rangingTest(&measure, false);
  door_angle = calc_degree(measure.RangeMilliMeter); 
  if (door_angle < 0) {
    door_angle = 0;
  }
  Serial.prinlnt(door_angle);
  //Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter); //used for calibrating tof sensor
}

/*void convert_force(int reading){
  float voltage;
  float resistance;
  float conductance;
  float force;
  int resistor = 10000; //we use a 10k resistor in the voltage divider circuit
  int Vcc = 5000; //in mV, we used 5v
  voltage = map(reading, 0, 1023, 0, Vcc); //re-maps voltage val to fsr reading range. 1023 is FSR max output when using 10k resistor and 5v
  //voltage = long(reading * Vcc) / 616; 
  if(voltage == 0){
    Serial.print(0); //no force
  }
  else{
    //fsr = ((Vcc - v) * R) / v
    resistance = Vcc -  voltage;
    resistance *= resistor;
    resistance /= voltage;

    conductance = 100000; //in micro-Ohms
    conductance /= resistance;
    //the value of the FSR's is a parabola based on the force applied.
    //this splits the parabola into two linear equations for linear approximation
    if(conductance <= 1000){
      force = conductance / 80;
    }
    else{
      force = conductance - 1000;
      force /= 30;
    }
    Serial.print(force);
  }
}*/
