/*
 * Project: Couchbot v2.0 RC Drivecode
 * Author:  Sidney Davis
 * Date:    8/25/2022
 * Contributor: Henry Gelber
 * Version: 4.0
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
#define CH5_PIN A3  //switch b
#define CH6_PIN A4  //switch g (3-position)
#define CH7_PIN A5  //switch f (to be used for siren)
//#define CH8_PIN A3  //switch h (springy) (to be used for horn)

// Motor Controller Pins
#define AN1 13
#define AN2 12
#define IN1 11
#define IN2 10

// joystick constants
#define JOY_MAX 1910
#define JOY_MIN 1080
#define JOY_IDLE 1495

#define RY_PIN   A1 // ch2
#define RX_PIN   A0 // ch1
#define LY_PIN   A2 // ch4

#define SPEED_MIN 4  // to be used when driving in pedestrian traffic
#define SPEED_MAX 32
#define SPEED_EXP 1.4

#define TURN_MIN 2
#define TURN_MAX 8
#define TURN_EXP 2  

#define ACCEL_MIN 2 
#define ACCEL_MAX 8
#define ACCEL_EXP 1

// variables
int lx, ly, rx, ry, ch5, ch6, ch7, ch8;
int drive_accel,         turn_mult;
int speed_setpoint,      turn_setpoint, accel_mult;
int left_motor_setpoint, right_motor_setpoint;
int left_motor_value,    right_motor_value;
float speed_multiplier;
float turn_multiplier;
bool RCconnection;

// Configure the motor drivers
CytronMD  leftMotor(PWM_DIR, AN2, IN2);
CytronMD rightMotor(PWM_DIR, AN1, IN1);

int countUpValue = 0;
  
void setup() {
  // put tfyour setup code here, to run once:
  if (SERIAL_ON) Serial.begin(115200);

  // RC and Motor Controller Power
  /*pinMode(RC_PWR_PIN, OUTPUT); digitalWrite(RC_PWR_PIN, HIGH); // Vcc
  pinMode(RC_GND_PIN, OUTPUT); digitalWrite(RC_GND_PIN, LOW);  // Gnd
  pinMode(MC_PWR_PIN, OUTPUT); digitalWrite(MC_PWR_PIN, HIGH); // Vcc
  pinMode(MC_GND_PIN, OUTPUT); digitalWrite(MC_GND_PIN, LOW);  // Gnd*/

  // RC Channels
  pinMode(RY_PIN, INPUT); pinMode(RX_PIN, INPUT);
  //pinMode(LY_PIN, INPUT); pinMode(LX_PIN, INPUT);
}

void loop() {
  countUpValue++;
  RCconnection = get_RC_values();
  set_setpoints(RCconnection);

  //update_motor_values();
  if (SERIAL_ON) print_function();
  // leftMotor.setSpeed(left_motor_setpoint);
  // rightMotor.setSpeed(right_motor_setpoint);
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

  // CH5 - switch f - estop
  ch5 = (pulseIn(CH5_PIN, HIGH, timeout) < 1152);
  if (ch5 == false) RC_connection = false;

  // left slider, accel mult
  ch6 = pulseIn(CH6_PIN, HIGH, timeout);
 // if (ch6 == 0) RC_connection = false;

  // right slider, turn mult
  ch7 = pulseIn(CH7_PIN, HIGH, timeout);
  //Serial.print((String)"RY:" + ry + " ch7: " +ch7 + " ");

  //if (abs(lx-JOY_IDLE) <= 10) and abs(ly-JOY_IDLE) <= 10) and abs(rx-JOY_IDLE) <= 10) and abs(ry-JOY_IDLE) <= 10) and abs(ly-JOY_IDLE) <= 10) and abs(accel_mult-JOY_IDLE) <= 10)) RC_connection = false;
  
  return RC_connection;
}

// Ref: https://home.kendra.com/mauser/joystick.html 
void set_setpoints(bool estop) {
  if ((abs(ly) < JOY_MIN) or (abs(ry) < JOY_MIN) or (abs(rx) < JOY_MIN)) return;
  float speed_mult = map(ly, JOY_MIN, JOY_MAX, SPEED_MAX, SPEED_MIN);
  int Y = map(ry, JOY_MIN, JOY_MAX, -100 * speed_mult, 100 * speed_mult) * pow(speed_mult, SPEED_EXP) / (SPEED_MAX * 16);
  
  float turn_mult = map(ch7, JOY_MIN, JOY_MAX, TURN_MAX, TURN_MIN) * 6;
  int X = map(rx, JOY_MIN, JOY_MAX, -100, 100) * pow(turn_mult, TURN_EXP) / (TURN_MAX * 32);

  float accel_mult = map(ch6, JOY_MIN, JOY_MAX, ACCEL_MAX * speed_mult, ACCEL_MIN * speed_mult) * 4 / (ACCEL_MAX);

  if(estop == false) {
    X = 0;
    Y = 0;
    accel_mult = SPEED_MAX * 4;
    if (SERIAL_ON) Serial.println("EMERGENCY STOP TRIGGERED\n\n");
  }
  
  if (speed_setpoint < Y) {
    speed_setpoint = min(speed_setpoint + accel_mult, Y);
  } else if (speed_setpoint > Y) {
    speed_setpoint = max(speed_setpoint - accel_mult, Y);
  }
  if(abs(speed_setpoint) < 1.5) speed_setpoint = 0;
  
  if (turn_setpoint < X) {
    turn_setpoint = min(turn_setpoint + (turn_mult * 4), X);
  } else if (turn_setpoint > X) {
    turn_setpoint = max(turn_setpoint - (turn_mult * 4), X);
  }

  int R = 1 * (speed_setpoint + turn_setpoint);
  int L = -1 * (speed_setpoint - turn_setpoint);
  
  // results
  speed_multiplier = 32.5;
  left_motor_setpoint  = map(L, -100, 100, -1 * speed_multiplier, 1 * speed_multiplier);
  right_motor_setpoint = map(R, -100, 100, -1 * speed_multiplier, 1 * speed_multiplier);

  return;
}

void print_function(void) {
  // raw values
  int pry = ry-1536;
  int prx = rx-1536;
  int ply = ly-1536;

  Serial.print("RCConnection:");
  Serial.print(RCconnection);

  Serial.print("ly:");
  Serial.print(ly);

  Serial.print(",rx:");
  Serial.print(rx);

  Serial.print(",ry:");
  Serial.print(ry);

  Serial.print(",ch5:");
  Serial.print(ch5);

  Serial.print(",ch6:");
  Serial.print(ch6);

  Serial.print(",ch7:");
  Serial.print(ch7);
  
  //Serial.print((String)"\nRightY:" + (ry) * -1 + " RightXJoy:" + (rx) + " LeftYJoy:" + (ly) * -1 + " ");
  //Serial.print((String)"\nLeftXJoy:" + lx + ", ");
  // setpoints
  // Serial.print((String)"Accel:" + ch6 + " Turn:" + ch7 + " ");
  //Serial.print((String)"DriveSetpoint:" + (speed_setpoint * -.25) + " TurnSetpoint:" + turn_setpoint + " ");
  Serial.print((String)",LeftMotorSetpoint:" + left_motor_setpoint + ",RightMotorSetpoint:" + right_motor_setpoint * -1 + "");
  Serial.println();

  return;
}
