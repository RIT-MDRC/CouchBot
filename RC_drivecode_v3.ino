/*
 * Project: Couchbot v2.0 RC Drivecode
 * Author:  Sidney Davis
 * Date:    5/2/2022
 * Contributor: Henry Gelber
 * Version: 3.0
 */

// Compile Options
#define SERIAL_ON true

// Libraries
#include "CytronMotorDriver.h"

// RC Pins
//#define CH5 7   //switch a
//#define CH6 12  //switch c (3-position)
//#define CH7 13  //switch f
//#define CH8 9   //switch h (springy)

// Radio Ctrl and Motor Ctrl Power
#define RC_PWR_PIN A1
#define RC_GND_PIN A0
#define MC_PWR_PIN 13
#define MC_GND_PIN 12

// joystick constants
#define RY_PIN   2 // ch2
#define RY_UP    2047
#define RY_DOWN  1024
#define RY_IDLE  ((RY_UP + RY_DOWN)/2)

#define RX_PIN   8 // ch1
#define RX_RIGHT 2047
#define RX_LEFT  1024
#define RX_IDLE  ((RX_LEFT + RX_RIGHT)/2)

#define LY_PIN   3 // ch4
#define LY_UP    2047
#define LY_DOWN  1024
#define LY_IDLE  LY_DOWN

#define LX_PIN   4 // ch3
#define LX_RIGHT 2047
#define LX_LEFT  1024
#define LX_IDLE  ((LX_LEFT + LX_RIGHT)/2)

#define MIN_ACCEL 10  // never 0 else no motion when LY down all the way
#define MAX_ACCEL 200 

// variables
int lx, ly, rx, ry, accel;
int left_motor_setpoint, right_motor_setpoint;
int left_motor_value,    right_motor_value;

// Configure the motor drivers
CytronMD  leftMotor(PWM_DIR, 5, 10); // PWM = Pin 5, DIR = Pin 10.
CytronMD rightMotor(PWM_DIR, 6, 11); // PWM = Pin 6, DIR = Pin 11.

void setup() {
  // put your setup code here, to run once:
  if (SERIAL_ON) Serial.begin(115200);

  // RC and Motor Controller Power
  pinMode(RC_PWR_PIN, OUTPUT); digitalWrite(RC_PWR_PIN, HIGH); // Vcc
  pinMode(RC_GND_PIN, OUTPUT); digitalWrite(RC_GND_PIN, LOW);  // Gnd
  pinMode(MC_PWR_PIN, OUTPUT); digitalWrite(MC_PWR_PIN, HIGH); // Vcc
  pinMode(MC_GND_PIN, OUTPUT); digitalWrite(MC_GND_PIN, LOW);  // Gnd

  // RC Channels
  pinMode(RY_PIN, INPUT); pinMode(RX_PIN, INPUT);
  pinMode(LY_PIN, INPUT); pinMode(LX_PIN, INPUT);
}

void loop() {
  bool RC_connection = get_RC_values();

  if (RC_connection) {
    // connection is good
    update_joystick_to_tank_setpoints();
  } else {
    // connection is bad. stop motors
    if (SERIAL_ON) Serial.println("RC CONNECTION LOST!!!");
    accel = MAX_ACCEL;
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

  // if all values != 0, then connection TRUE
  return RC_connection;
}

// Ref: https://home.kendra.com/mauser/joystick.html 
void update_joystick_to_tank_setpoints(void) {
  
  // 1&2. scale inputs and invert X
  int X = map(rx, RX_LEFT, RX_RIGHT, 100, -100);
  int Y = map(ry, RY_DOWN, RY_UP,    -100, 100);

  // 3. calc V = (R+L)
  int V = (100 - abs(X)) * (Y/100) + Y;

  // 4. calc W = (R-L)
  int W = (100 - abs(Y)) * (X/100) + X;

  // 5&6. calc R & L = (V+W)/2 & (V-W)/2
  int R = (V + W) / 2;
  int L = (V - W) / 2;

  // results
  left_motor_setpoint  = map(L, -100, 100, -256, 256);
  right_motor_setpoint = map(R, -100, 100, -256, 256);

  // get the acceleration
  accel = map(ly, LY_DOWN, LY_UP, MIN_ACCEL, MAX_ACCEL); 

  return;
}

// const acceleration according to throttle
void update_motor_values(void) {

  // change left motor values based on setpoints and accel
  if (left_motor_value < left_motor_setpoint) {
    left_motor_value = min(left_motor_value + accel, left_motor_setpoint);
  } else 
  if (left_motor_value > left_motor_setpoint) {
    left_motor_value = max(left_motor_value - accel, left_motor_setpoint);
  }

  // change right motor values based on setpoints and accel
  if (right_motor_value < right_motor_setpoint) {
    right_motor_value = min(right_motor_value + accel, right_motor_setpoint);
  } else 
  if (right_motor_value > right_motor_setpoint) {
    right_motor_value = max(right_motor_value - accel, right_motor_setpoint);
  } 

  return;
}

void print_function(void) {
  // raw values
  Serial.print("\nRight Y Joy: "); Serial.println(ry);
  Serial.print("Right X Joy: "); Serial.println(rx);
  Serial.print("Left Y Joy: "); Serial.println(ly);
  Serial.print("Left X Joy: "); Serial.println(lx);

  // setpoints
  Serial.print("\nLeft Motor Setpoint: "); Serial.println(left_motor_setpoint);
  Serial.print("Right Motor Setpoint: "); Serial.println(right_motor_setpoint);

  // values
  Serial.print("\nAcceleration: "); Serial.println(accel); 
  Serial.print("Left Motor Value: "); Serial.println(left_motor_value);
  Serial.print("Right Motor Value: "); Serial.println(right_motor_value);

  return;
}
