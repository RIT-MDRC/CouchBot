#include <Servo.h>


const uint8_t LEFT_SERVO_PIN = 5;
const uint8_t RIGHT_SERVO_PIN = 6;
const uint8_t RC_CH1 = A0;
const uint8_t RC_CH2 = A1;
const uint8_t RC_CH3 = A2;
const uint16_t CH1_MAX = 1800;
const uint16_t CH1_MIN = 1100;
const uint16_t CH2_MAX = 1800;
const uint16_t CH2_MIN = 1100;
const uint16_t CH3_MAX = 1800;
const uint16_t CH3_MIN = 1100;
const uint8_t reverseRight = 1;
const uint8_t reverseLeft = 0;


//working variables
uint16_t CH1_last_value = 0;
uint16_t CH2_last_value = 0;
uint16_t CH3_last_value = 0;
uint32_t timeout = 100;
uint32_t lastTime = 0;
int16_t velocity = 0;
int16_t difference = 0;
int16_t maxSpeed = 0;
int16_t leftMotorSpeed = 0;
int16_t rightMotorSpeed = 0;
uint8_t maxLeftPWM = 180;
uint8_t minLeftPWM = 0;
uint8_t maxRightPWM = 180;
uint8_t minRightPWM = 0;


Servo leftMotor;
Servo rightMotor;


void setup() {
  Serial.begin(115200);
  leftMotor.attach(LEFT_SERVO_PIN); leftMotor.write(90);
  rightMotor.attach(RIGHT_SERVO_PIN); rightMotor.write(90);
  pinMode(RC_CH1, INPUT);
  pinMode(RC_CH2, INPUT);
  pinMode(RC_CH3, INPUT);
}


void loop() {
  int32_t ch1Average = 0;
  int32_t ch2Average = 0;
  int32_t ch3Average = 0;
  for (uint8_t i = 0; i < 3; i++) {
    getReceiver();
    ch1Average += CH1_last_value;
    ch2Average += CH2_last_value;
    ch3Average += CH3_last_value;
  }
  CH1_last_value = ch1Average / 3.0;
  CH2_last_value = ch2Average / 3.0;
  CH3_last_value = ch3Average / 3.0;
  scaleNumbers();
  controlMotor();
  serialUpdate();
}


void getReceiver() {
  CH1_last_value = 0;
  CH2_last_value = 0;
  CH3_last_value = 0;

  lastTime = millis();
  while (CH1_last_value == 0 && millis() - lastTime < timeout) {
    CH1_last_value = pulseIn(RC_CH1, HIGH, 20000);
  }
  lastTime = millis();
  while (CH2_last_value == 0 && millis() - lastTime < timeout) {
    CH2_last_value = pulseIn(RC_CH2, HIGH, 20000);
  }
  lastTime = millis();
  while (CH3_last_value == 0 && millis() - lastTime < timeout) {
    CH3_last_value = pulseIn(RC_CH3, HIGH, 20000);
  }
}


void scaleNumbers() {
  if (CH1_last_value > CH1_MAX) difference = CH1_MAX;
  else if (CH1_last_value < CH1_MIN) difference = CH1_MIN;
  else difference = CH1_last_value;
  difference = map(difference, CH1_MIN, CH1_MAX, -99, 99);

  if (CH2_last_value > CH2_MAX) velocity = CH2_MAX;
  else if (CH2_last_value < CH2_MIN) velocity = CH2_MIN;
  else velocity = CH2_last_value;
  velocity = map(velocity, CH2_MIN, CH2_MAX, -99, 99);

  if (CH3_last_value > CH3_MAX) maxSpeed = CH3_MAX;
  else if (CH3_last_value < CH3_MIN) maxSpeed = CH3_MIN;
  else maxSpeed = CH3_last_value;
  maxSpeed = map(maxSpeed, CH3_MIN, CH3_MAX, 1, 99);

  // set everything to 0 if transmitter is off
  if (CH1_last_value == 0) difference = 0;  //CH1 returns 0 if radio is off
  if (CH2_last_value == 0) velocity = 0;  //CH2 returns 0 if radio is off
  if (CH3_last_value == 0) velocity = 0;  //CH3 returns 0 if radio is off

}


void controlMotor() {
  if (reverseLeft) {
    maxLeftPWM = 0;
    minLeftPWM = 180;
  }
  if (reverseRight) {
    maxRightPWM = 0;
    minRightPWM = 180;
  }

  if (velocity >= -6 && velocity <= 6) velocity = 0;
  if (difference >= -6 && difference <= 6) difference = 0;

  leftMotorSpeed = constrain((maxSpeed/99.0)*(velocity + difference),-99,99);
  rightMotorSpeed = constrain((maxSpeed/99.0)*(velocity - difference),-99,99);

  leftMotor.write(map(leftMotorSpeed, -99, 99, minLeftPWM, maxLeftPWM));
  rightMotor.write(map(rightMotorSpeed, -99, 99, minRightPWM, maxRightPWM));
}


void serialUpdate() {
  Serial.print("CH1: "); Serial.print(CH1_last_value);
  Serial.print("  CH2: "); Serial.print(CH2_last_value);
  Serial.print("  CH3: "); Serial.print(CH3_last_value);
  Serial.print("  Velocity: "); Serial.print(velocity);
  Serial.print("  Diff: "); Serial.print(difference);
  Serial.print("  Max Speed: "); Serial.print(maxSpeed);
  Serial.print("  L: "); Serial.print(leftMotorSpeed);
  Serial.print("  R: "); Serial.println(rightMotorSpeed);
}

