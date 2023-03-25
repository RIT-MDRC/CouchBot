/*
 * Project: Couchbot v2.0 RC Drivecode
 * Author:  Sidney Davis
 * Date:    8/25/2022
 * Contributor: Henry Gelber
 * Version: 3.3.1
 * Changelog:
 *  Updated pins
 *  Exponential speed control

/*
 * Known bugs:
 * Max speed steering issues
 */

// Compile Options
#define SERIAL_ON true

// Libraries
#include "CytronMotorDriver.h"
  
// RC Pins - switches are re-assignable in controller menus
#define CH5_PIN A4  //switch b
#define CH6_PIN A5  //switch g (3-position)
//#define CH7_PIN A2  //switch f (to be used for siren)
//#define CH8_PIN A3  //switch h (springy) (to be used for horn)

// Radio Ctrl and Motor Ctrl Power
/*#define RC_PWR_PIN A1
#define RC_GND_PIN A0
#define MC_PWR_PIN 13
#define MC_GND_PIN 12*/

// joystick constants
#define RY_PIN   A1 // ch2
#define RY_OFFSET -48
#define RY_UP    2047 + RY_OFFSET
#define RY_DOWN  1024 + RY_OFFSET
#define RY_IDLE  ((RY_UP + RY_DOWN)/2)

#define RX_PIN   A0 // ch1
#define RX_OFFSET -48
#define RX_RIGHT 2047 + RX_OFFSET
#define RX_LEFT  1024 + RX_OFFSET
#define RX_IDLE  ((RX_LEFT + RX_RIGHT)/2)

#define LY_PIN   A2 // ch4
#define LY_UP    2047
#define LY_DOWN  1024
#define LY_IDLE  LY_DOWN

#define LX_PIN   A3 // ch3
#define LX_RIGHT 2047
#define LX_LEFT  1024
#define LX_IDLE  ((LX_LEFT + LX_RIGHT)/2)


//SPEED_MIN and TURN_MULT should never be 0 otherwise no movement/turn
#define SPEED_MIN 0.6  // to be used when driving in pedestrian traffic
#define SPEED_MAX 5
#define SPEED_EXP 2

#define TURN_MULT_1 0.5   // parking
#define TURN_MULT_2 0.75  // driving
#define TURN_MULT_3 1.25  // showing off

#define DRIVE_ACCEL 20 
#define TURN_ACCEL 80
#define SHOWBOAT_MULTIPLIER 1.75

// variables
int lx, ly, rx, ry, ch5, ch6, ch7, ch8;
int drive_accel,         turn_accel, accel_mult;
int speed_setpoint,      turn_setpoint;
int left_motor_setpoint, right_motor_setpoint;
int left_motor_value,    right_motor_value;
float speed_multiplier;
float turn_multiplier;

// Configure the motor drivers
CytronMD  leftMotor(PWM_DIR, 5, 10); // PWM = Pin 5, DIR = Pin 10.
CytronMD rightMotor(PWM_DIR, 6, 11); // PWM = Pin 6, DIR = Pin 11.


int countUpValue = 0;

void setup() {
  // put your setup code here, to run once:
  if (SERIAL_ON) Serial.begin(115200);

  // RC and Motor Controller Power
  /*pinMode(RC_PWR_PIN, OUTPUT); digitalWrite(RC_PWR_PIN, HIGH); // Vcc
  pinMode(RC_GND_PIN, OUTPUT); digitalWrite(RC_GND_PIN, LOW);  // Gnd
  pinMode(MC_PWR_PIN, OUTPUT); digitalWrite(MC_PWR_PIN, HIGH); // Vcc
  pinMode(MC_GND_PIN, OUTPUT); digitalWrite(MC_GND_PIN, LOW);  // Gnd*/

  // RC Channels
  pinMode(RY_PIN, INPUT); pinMode(RX_PIN, INPUT);
  pinMode(LY_PIN, INPUT); pinMode(LX_PIN, INPUT);
}

void loop() {
  countUpValue++;
  bool RC_connection = get_RC_values();
  if (RC_connection & ch5) {
    // connection is good
    update_joystick_to_tank_setpoints();
  } else {
    // connection is bad. stop motors
    if (SERIAL_ON) Serial.println("⚠ ⚠ ⚠ RC CONNECTION LOST ⚠ ⚠ ⚠ \n\n");
    drive_accel = pow(SPEED_MAX, SPEED_EXP);
    speed_setpoint = 0;
    turn_setpoint = 0;
    left_motor_setpoint = 0;
    right_motor_setpoint = 0;
  }
  
  update_motor_values();
  if (SERIAL_ON) print_function();
  leftMotor.setSpeed(left_motor_value);
  rightMotor.setSpeed(right_motor_value);
  delay(10);
}

// get the values of each joystick, and return false if no response
bool get_RC_values(void) {
  bool RC_connection = true; 
  unsigned long timeout = 25000; // microseconds
  
  // right joystick, x-axis
  rx = pulseIn(RX_PIN, HIGH, timeout);
  if (rx == 0) RC_connection = false;

  // right joystick, y-axis
  ry = pulseIn(RY_PIN, HIGH, timeout);
  if (ry == 0) RC_connection = false;

  // left joystick, y-axis
  ly = pulseIn(LY_PIN, HIGH, timeout);
  if (ly == 0) RC_connection = false;

  // CH5 - switch f
  ch5 = (pulseIn(CH5_PIN, HIGH, timeout) < 1152);

  // CH6 - switch e 3pos, turn speed control
  ch6 = (pulseIn(CH6_PIN, HIGH, timeout)) > 1750 ? 3 : (pulseIn(CH6_PIN, HIGH, timeout) > 1152 ? 2 : 1);
  if (ly == 0) RC_connection = false;
  
/*  // CH7 - switch f (3-pos)
  ch7 = {pulseIn(CH7_PIN, HIGH, timeout) < 1152);

  // CH8 - switch h (springy)
  ch8 = (pulseIn(CH8_PIN, HIGH, timeout) < 1152); 

  Serial.print(ch5);
  Serial.print(",");
  Serial.print(ch6);
  Serial.print(",");
  Serial.print(ch7);
  Serial.print(",");
  Serial.println(ch8);
  */
  // if all values != 0, then connection TRUE
  return RC_connection;
}

// Ref: https://home.kendra.com/mauser/joystick.html 
void update_joystick_to_tank_setpoints(void) {
 
  // Set turn multiplier from 3-way switch
  turn_multiplier = (ch6 == 1 ? TURN_MULT_1 : ch6 == 2 ? TURN_MULT_2 : TURN_MULT_3);
  
  // 1&2. scale inputs and invert X
  int X = map(rx, RX_LEFT, RX_RIGHT, -100 * turn_multiplier, 100 * turn_multiplier);
  int Y = map(ry, RY_DOWN, RY_UP,    -100, 100);
  
  //set speed multiplier from left y

  float LY = map(ly, LY_UP, LY_DOWN, SPEED_MIN * 16, SPEED_MAX * 16);
  accel_mult = (LY / 32);
  if(accel_mult < 1){
    accel_mult = 1;
  }
  LY = pow(LY, SPEED_EXP) / (SPEED_MAX * 256);
  Y = Y * LY;
  

     
  /*if(ch5 == 1) {
     Y = map(ly, LY_UP, LY_DOWN, SHOWBOAT_SPEED_MIN * Y, SHOWBOAT_SPEED_MAX * Y);
     accel_mult = SHOWBOAT_MULTIPLIER;
     Serial.println("SHOWBOAT MODE ENABLED");
  } else {
    Y = map(ly, LY_UP, LY_DOWN, STANDARD_SPEED_MIN * Y, STANDARD_SPEED_MAX * Y);
    accel_mult = 1;
  }*/

  if (speed_setpoint < Y) {
    speed_setpoint = min(speed_setpoint + (DRIVE_ACCEL * accel_mult), Y);
  } else if (speed_setpoint > Y) {
    speed_setpoint = max(speed_setpoint - (DRIVE_ACCEL * accel_mult), Y);
  }

  if (turn_setpoint < X) {
    turn_setpoint = min(turn_setpoint + (TURN_ACCEL * accel_mult), X);
  } else if (turn_setpoint > X) {
    turn_setpoint = max(turn_setpoint - (TURN_ACCEL * accel_mult), X);
  }
  // 3. calc V = (R+L)
  int V = (500 - abs(turn_setpoint)) * (speed_setpoint/500) + speed_setpoint;

  // 4. calc W = (R-L)
  int W = (500 - abs(speed_setpoint)) * (turn_setpoint/500) + turn_setpoint;

  //Serial.println(V);
  //Serial.println(W);
  
  // 5&6. calc R & L = (V+W)/2 & (V-W)/2
  int R = (V + W) / 2;
  int L = (V - W) / -2;

  // results
  left_motor_setpoint  = map(L, -100, 100, -256, 256);
  right_motor_setpoint = map(R, -100, 100, -256, 256);

  return;
}

// const acceleration according to throttle
void update_motor_values(void) {

  // change left motor values based on setpoints and accel
    left_motor_value = left_motor_setpoint;
    right_motor_value = right_motor_setpoint;
  return;
}

void print_function(void) {
  // raw values
  int pry = ry-1536;
  int prx = rx-1536;
  int ply = ly-1536;
  
  Serial.print((String)"\nRightY:" + pry * -1 + " RightXJoy:" + prx + " LeftYJoy:" + ply * -1 + " ");
  //Serial.print((String)"\nLeft X Joy:" + lx + ", "); not used currently

  // setpoints
  Serial.print((String)"DriveSetpoint:" + speed_setpoint + " TurnSetpoint:" + turn_setpoint + " ");
  Serial.print((String)"LeftMotorSetpoint:" + left_motor_setpoint + " RightMotorSetpoint:" + right_motor_setpoint * -1 + " ");

  // values
  Serial.print((String)"LeftMotorValue:" + left_motor_value + "RightMotorValue:" + right_motor_value * -1 + "\n\n");

  return;
}
