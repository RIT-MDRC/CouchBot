/* COUCHBOT CODE :)
 * Created: 11-07-2025
 * Author: Patrick Brennan
 * Roughly based on previous couchbot code (v4.1.0)
*/

// Enable serial outputs
#define SERIAL_ON true

// ===== DRIVE PARAMETERS =====
// Change these to change driving behavior
// Arbitrary speed and accel constants
// Will be more meaningful with PID implementation maybe in future?
// Decimal representing % of full speed
#define THROTTLE_MAX 1.0
#define TURN_MAX 0.45

#define ACCEL 0.05 // Seperate acceleration and deceleration curves
#define DECEL 0.10
#define TURN_ACCEL 0.15

// Right stick deadzones
// Having a deadzone of less than 0.02 may lead to weird acceleration
// behavior
#define DRIVE_DEADZONE 0.05
#define ROT_DEADZONE 0.05

// ===== END DRIVE PARAMETERS =====

// RC Reciever Pins
#define CH1_PIN A0 // RX
#define CH2_PIN A1 // RY
#define CH3_PIN A2 // LY
#define CH5_PIN A3 // Switch F (Estop)
#define CH6_PIN A4 // Switch H (Springy)
#define CH7_PIN A5 // Back right slider (turn accel)

// RC Connection Timeout
#define TIMEOUT 25000 // microseconds

// Motor driver pins
#define IN1 13
#define IN2 12
#define AN1 11
#define AN2 10

// Piezzo buzzer pin
#define BUZZER 2

#define JOY_MAX 1920
#define JOY_MIN 1080

// =============================

#include "CytronMotorDriver.h"

CytronMD leftMotor(PWM_DIR, AN2, IN2);
CytronMD rightMotor(PWM_DIR, AN1, IN1);

// Channel values object
struct RCValues {
  bool connection = false;
  int ly = 0;
  int rx = 0;
  int ry = 0;
  int ch5 = 0;
  int ch6 = 0;
  int ch7 = 0;
};

RCValues rc;
int leftMotorSetpoint = 0;
int rightMotorSetpoint = 0;
float drive = 0.0;
float rot = 0.0;

bool remote_stop = true;
bool stop_tone_played = false;
bool enable_tone_played = false;

// Typesafe sign function
template <typename T> int signum(T val) {
    if (val >= 0){
      return 1;
    } else {
      return -1;
    }
}

template <typename T,typename M> T deadzone(T val, M deadzone) {
    if (val > deadzone || val < -deadzone) return val;
    return (T)0;
}

// Update RC values
// Returns array of reciever values
// [LY, RX, RY, CH5, CH6, CH7]
void update_rc() {
  rc.connection = true;
  // Stick values
  rc.rx = pulseIn(CH1_PIN, HIGH, TIMEOUT);
  if (rc.rx == 0) rc.connection = false;

  rc.ry = pulseIn(CH2_PIN, HIGH, TIMEOUT);
  if (rc.ry == 0) rc.connection = false;

  rc.ly = pulseIn(CH3_PIN, HIGH, TIMEOUT);
  if (rc.ly == 0) rc.connection = false;

  // Switches
  rc.ch5 = (pulseIn(CH5_PIN, HIGH, TIMEOUT) < 1152);
  rc.ch6 = (pulseIn(CH6_PIN, HIGH, TIMEOUT) < 1152);

  // Rot. multipler slider
  rc.ch7 = pulseIn(CH7_PIN, HIGH, TIMEOUT);
}

void update_setpoints(RCValues rc) {
  
  // Not sure how safe this line is
  // Joysticks go below value -> setpoints stay at last value?
  // if ((abs(ly) < JOY_MIN) or (abs(ry) < JOY_MIN) or (abs(rx) < JOY_MIN)) return;

  // Map and reverse ranges (UP on joysticks is lower pwm value)
  float throttle = map(rc.ly, JOY_MIN, JOY_MAX, 255*THROTTLE_MAX, 0)/255.0;
  float turn_accel = map(rc.ch7, JOY_MIN, JOY_MAX, 255*TURN_MAX, 0)/255.0;

  // DOWN -100 | 100 UP
  float driveSetpoint = map(rc.ry, JOY_MIN, JOY_MAX, 255, -255)/255.0;
  // Add deadzone and joystick curve (generated using best fit curve in excel)
  // (0.0463 + (1.53*driveSetpoint)+(-0.621 * pow(driveSetpoint, 2))
  driveSetpoint = deadzone(throttle * driveSetpoint, DRIVE_DEADZONE);

  // LEFT 100 | -100 RIGHT
  float rotSetpoint = map(rc.rx, JOY_MIN, JOY_MAX, 255, -255)/255.0;
  rotSetpoint = deadzone(rotSetpoint * turn_accel, ROT_DEADZONE);

  // Drive acceleration curves
  if ((driveSetpoint-drive) * signum(drive) < 0) {
    // Drive decceleration curve
    if (drive < driveSetpoint) {
      drive = min(drive + DECEL, driveSetpoint);
    } else if (drive > driveSetpoint) {
      drive = max(drive - DECEL, driveSetpoint);
    }
  } else {
    // Drive acceleration curve
    if (drive < driveSetpoint) {
      drive = min(drive + ACCEL, driveSetpoint);
    } else if (drive > driveSetpoint) {
      drive = max(drive - ACCEL, driveSetpoint);
    }
  }

  // Rotation acceleration curves
  if (rot < rotSetpoint) {
    rot = min(rot + TURN_ACCEL, rotSetpoint);
  } else if (rot > rotSetpoint) {
    rot = max(rot - TURN_ACCEL, rotSetpoint);
  }

  rightMotorSetpoint = -1 * ((drive * 255) - (rot * 255));
  leftMotorSetpoint = 1 * ((drive * 255) + (rot * 255));

  if (SERIAL_ON) {
    // Serial.print(" throttle:");
    // Serial.print(throttle);
    // Serial.print(" turn_accel:");
    // Serial.print(turn_accel);
    Serial.print(" driveSetpoint:");
    Serial.print(driveSetpoint);
    Serial.print(" ry:");
    Serial.print(rc.ry);
    Serial.print(" drive:");
    Serial.print(drive);
    Serial.print(" rotSetpoint:");
    Serial.print(rotSetpoint);
    Serial.print(" rot:");
    Serial.print(rot);
    Serial.print(" leftSetpoint:");
    Serial.print(leftMotorSetpoint);
    Serial.print(" rightSetpoint:");
    Serial.print(rightMotorSetpoint);
    Serial.println();
  }

}

void setup() {
  // Pins are by default inputs
  // Motor driver handles output pinmodes
  if (SERIAL_ON) Serial.begin(115200);
  pinMode(2, OUTPUT);
}

void loop() {
  update_rc();

  // if (SERIAL_ON) {
  //   Serial.print("conn:");
  //   Serial.println(rc.ch5);
  //   Serial.print("stop:");
  //   Serial.println(remote_stop);
  // }

  if (rc.connection) {
    update_setpoints(rc);
  }

  // Remote stop resume safety feature
  // ensure right stick is centered before the couch starts moving again
  // after transmitter connection or remote stop
  if (remote_stop && rc.ch5 && drive == 0.0 && rot == 0.0) {
    remote_stop = false;

    stop_tone_played = false; 
    // Play enable tone
    if (!enable_tone_played) {
      tone(BUZZER, 500, 50);
      delay(100);
      tone(BUZZER, 800, 50);
      delay(100);
      tone(BUZZER, 1000, 50);
      enable_tone_played = true;
    }
    
  } else if (remote_stop) {
    // Full-stop condition
    if (SERIAL_ON) Serial.println("!!! REMOTE STOP ENABLED !!!");
    leftMotorSetpoint = 0;
    rightMotorSetpoint = 0;
    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);

    enable_tone_played = false;
    // Play stop tone
    if (!stop_tone_played) {
      tone(BUZZER, 1000, 50);
      delay(100);
      tone(BUZZER, 800, 50);
      delay(100);
      tone(BUZZER, 500, 50);
      stop_tone_played = true;
    }

    return;
  }

  if (rc.connection && rc.ch5) {
    // Normal run condition
    leftMotor.setSpeed(leftMotorSetpoint);
    rightMotor.setSpeed(rightMotorSetpoint);
  } else {
    if (SERIAL_ON && !rc.connection) Serial.println("Transmitter not connected");
    if (!rc.connection || !rc.ch5) remote_stop = true;
  }

  delay(10);
}

