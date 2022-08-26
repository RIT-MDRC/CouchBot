#include <Servo.h>

Servo leftMotor;
Servo rightMotor;

const uint8_t LEFT_SERVO_PIN = 5;
const uint8_t RIGHT_SERVO_PIN = 6;

const byte leftEncoderInt = 2;
const byte rightEncoderInt = 3;
const byte leftEncoder = 10;
const byte rightEncoder = 11;



float leftVelocity = 0;
float rightVelocity = 0;
uint32_t velocityTimer = 0;
int32_t leftPositionLast = 0;
int32_t rightPositionLast = 0;
volatile int32_t leftPosition = 0;
volatile int32_t rightPosition = 0;



void EncChangeL() {
  if (digitalRead(leftEncoderInt) == HIGH) {
    if (digitalRead(leftEncoder) == HIGH) leftPosition--;
    else leftPosition++;
  }
  else {
    if (digitalRead(leftEncoder) == HIGH) leftPosition++;
    else leftPosition--;
  }
}

void EncChangeR() {
  if (digitalRead(rightEncoderInt) == HIGH) {
    if (digitalRead(rightEncoder) == HIGH) rightPosition++;
    else rightPosition--;
  }
  else {
    if (digitalRead(rightEncoder) == HIGH) rightPosition--;
    else rightPosition++;
  }
}



void setup() {
  Serial.begin (115200);

  leftMotor.attach(LEFT_SERVO_PIN); leftMotor.write(90);
  rightMotor.attach(RIGHT_SERVO_PIN); rightMotor.write(90);

  pinMode(leftEncoder, INPUT_PULLUP);
  pinMode(rightEncoder, INPUT_PULLUP);
  pinMode(leftEncoderInt, INPUT_PULLUP);
  pinMode(rightEncoderInt, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(leftEncoderInt), EncChangeL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoderInt), EncChangeR, CHANGE);
}

void loop() {
  updateVelocity();
  if (Serial.available() > 0) {
    byte incomingByte = Serial.read();
    if(incomingByte == 'l') leftMotor.write(100);  //forward
    else if(incomingByte == 'h') leftMotor.write(125);  //forward
    else if(incomingByte == 'x') leftMotor.write(90);  //forward
    else if(incomingByte == 's') leftMotor.write(200);  //forward
  }
  Serial.print("\tL: "); Serial.print(leftPosition);
  Serial.print("\tR: "); Serial.print(rightPosition);
  Serial.print("\tLV: "); Serial.print(leftVelocity);
  Serial.print("\tRV: "); Serial.println(rightVelocity);
  //delay(500);
}

void updateVelocity() {
  velocityTimer = micros() - velocityTimer;
  Serial.print("T: "); Serial.print(velocityTimer);
  Serial.print("\tD: "); Serial.print((leftPosition - leftPositionLast));
  
  rightVelocity = (rightPosition - rightPositionLast) / velocityTimer;

  leftPositionLast = leftPosition;
  rightPositionLast = rightPosition;
  velocityTimer = micros();
}

