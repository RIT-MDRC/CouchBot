#include "CytronMotorDriver.h"

#define CH1 8 //right x
#define CH2 2 //right y
#define CH3 3 //left y (not springy)
#define CH4 4 //left x
#define CH5 7 //switch a
#define CH6 12 //switch c (3-positon)
#define CH7 13 //switch f
#define CH8 9 //switch h (springy)

#define joyX_pin A3
#define joyY_pin A2
#define joyButt_pin A4

int joyXOff = 0;
int joyYOff = 0;
int speedMultiplier = 1;
double curveX = 1.5;
double curveY = 1.5;

#define JOY_DEADZONE 15

//#define rightMotorMagnitude 9
//#define rightMotorDirection 10
//#define rightMotorDirectionReverse 11
//#define leftMotorMagnitude 5
//#define leftMotorDirection 6
//#define leftMotorDirectionReverse 7
CytronMD leftMotor(PWM_DIR, 5, 10);
CytronMD rightMotor(PWM_DIR, 6, 11);


#define PWM_VCC 13
#define PWM_GND 12

const int chXInterval = 5; // the window to integrate over. Increase this value to increase smoothing
float chXRunningSum = 0;  // this is integrated
int chXReadingList[chXInterval]; 
int chXPointer = 0;

const int chYInterval = 5; // the window to integrate over. Increase this value to increase smoothing
float chYRunningSum = 0;  // this is integrated
int chYReadingList[chYInterval]; 
int chYPointer = 0;

uint8_t framesToSkip = 80; // how many iterations of the loop to go through before making a new controller reading
uint8_t frameCounter = 0;

void setup() {
  pinMode(A1, OUTPUT);
  pinMode(A0, OUTPUT);
  digitalWrite(A1, HIGH); // power for the joystick
  digitalWrite(A0, LOW);

  digitalWrite(PWM_GND, LOW); // power for pwm
  digitalWrite(PWM_VCC, HIGH);
  
  pinMode(CH1, INPUT);
  pinMode(CH2, INPUT);
  pinMode(CH3, INPUT);
  pinMode(CH4, INPUT);
  pinMode(CH5, INPUT);
  pinMode(CH6, INPUT);
  pinMode(CH7, INPUT);
  pinMode(CH8, INPUT);

  pinMode(joyX_pin, INPUT);
  pinMode(joyY_pin, INPUT);

  //pinMode(rightMotorMagnitude, OUTPUT);
  //pinMode(rightMotorDirectionForward, OUTPUT);
  //pinMode(rightMotorDirectionReverse, OUTPUT);
  //pinMode(leftMotorMagnitude, OUTPUT);
  //pinMode(leftMotorDirectionForward, OUTPUT);
  //pinMode(leftMotorDirectionReverse, OUTPUT);
  
  Serial.begin(115200);

  // initialize to all 0
  for (int i = 0; i < chXInterval; i++) {
    chXReadingList[i] = 0;
  }
  for (int i = 0; i < chYInterval; i++) {
    chYReadingList[i] = 0;
  }

  // account for joystick offset
  joyXOff = analogRead(joyX_pin) - 1024/2;
  joyYOff = analogRead(joyY_pin) - 1024/2;
  
  waitForStartCondition();
}

void loop() {
  // this might be done better with delay(ms);
  if(++frameCounter % framesToSkip == 0) {
  
    int joyX_raw = analogRead(joyX_pin) - joyXOff; //thumbpad
    int joyY_raw = analogRead(joyY_pin) - joyYOff;
    
    int rightX_raw = readcontrol(CH1); //remote control
    int rightY_raw = readcontrol(CH2);
    int leftY_raw = readcontrol(CH3);
    int leftX_raw = readcontrol(CH4);
    
    bool CH5Enabled;
    if(896 < readcontrol(CH5) && readcontrol(CH5) < 1152){CH5Enabled = true;} else {CH5Enabled = false;}
    bool CH6Enabled;
    if(896 < readcontrol(CH6) && readcontrol(CH6) < 1152){CH6Enabled = true;} else {CH6Enabled = false;}
    bool CH7Enabled;
    if(896 < readcontrol(CH7) && readcontrol(CH7) < 1152){CH7Enabled = false;} else {CH7Enabled = true;}
    bool CH8Enabled;
    if(896 < readcontrol(CH8) && readcontrol(CH8) < 1152){CH8Enabled = false;} else {CH8Enabled = true;}
    
    joyX_raw = map(joyX_raw, 0, 1023, -255, 255);
    joyY_raw = map(joyY_raw, 0, 1023, -255, 255) * (CH5Enabled + 1);

    rightX_raw = (map(rightX_raw, 1024, 2047, -127, 127)+12);
    rightY_raw = (map(rightY_raw, 1024, 2047, 255, -255)-24) * (CH5Enabled + 1);
    leftY_raw = (map(leftY_raw, 1024, 2047, 255, 0)-24);
    leftX_raw = (map(leftX_raw, 1024, 2047, 127, -127)-12) * (CH5Enabled + 1);
    
    int couchX, couchY, leftPower, rightPower;
    //if(CH5Enabled == 1){
      couchX = smoothControl(rightX_raw, chXRunningSum, chXReadingList, chXPointer, chXInterval) * CH6Enabled * curveInput(rightX_raw, curveX, 63);
      couchY = smoothControl(rightY_raw, chYRunningSum, chYReadingList, chYPointer, chYInterval) * CH6Enabled * curveInput(rightX_raw, curveY, 128);
      leftPower = couchY + couchX;
      rightPower = couchY - couchX;
      setMotors(leftPower, rightPower);
    //} else {
      /*
      int couchX = smoothControl(joyX_raw, chXRunningSum, chXReadingList, chXPointer, chXInterval);
      int couchY = smoothControl(joyY_raw, chYRunningSum, chYReadingList, chYPointer, chYInterval);
      int leftPower = couchY + couchX;
      int rightPower = couchY - couchX;
      setMotors(leftPower * speedMultiplier, rightPower * speedMultiplier);
      Serial.print(couchX);
      Serial.print(',');
      Serial.println(couchY);
      */
    //  setMotors(0, 0);
    //}
    
    frameCounter = 0;
    
    //Serial.print(',');
    //Serial.print(rightX_raw);
    //Serial.print(',');
    //Serial.print(rightY_raw);
    //Serial.print(',');
    Serial.print(leftPower);
    Serial.print(',');
    Serial.println(rightPower);
  }
}

void waitForStartCondition() {
  // wait for push up
  /*
  while(map(smoothControl(analogRead(joyY_pin)-joyYOff, ch1RunningSum, ch1ReadingList, ch1Pointer, ch1Interval), 0, 1024, -255, 255) < 200) {
    delay(1);
  }
  // wait for release
  while(map(smoothControl(analogRead(joyY_pin), ch1RunningSum, ch1ReadingList, ch1Pointer, ch1Interval), 0, 1024, -255, 255) > 25) {
    delay(1);
  } */
}

int readcontrol(int CH) {
 int wavepeak = pulseIn(CH, HIGH, 25000);
 if(wavepeak == 0){
   wavepeak = pulseIn(CH, HIGH, 25000);
 }
 return wavepeak;
}

int smoothControl(int reading, float &runningSum, int valList[], int &indexPointer, int integrationInterval) {
  runningSum -= valList[indexPointer];
  runningSum += reading;
  valList[indexPointer] = reading;

  indexPointer++;
  indexPointer = indexPointer % integrationInterval;

  return (runningSum / integrationInterval);
}

float curveInput(float reading, float curveExp, float maximum) {
  reading = reading / maximum;
  //return(pow(reading, curveExp));
  return 1;
}

/**
 * set signed output to motors within a range of -255 to 255
 */
void setMotors(int left, int right) {
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);
  Serial.print(left);
  Serial.print(',');
  Serial.println(right);

  leftMotor.setSpeed(left);
  rightMotor.setSpeed(-right);
  
  //digitalWrite(leftMotorDirection, left > 0);
  //digitalWrite(leftMotorDirectionReverse, left < JOY_DEADZONE);
  //analogWrite(leftMotorMagnitude, abs(left));
  
  //digitalWrite(rightMotorDirection, right > 0);
  //digitalWrite(rightMotorDirectionReverse, right < JOY_DEADZONE);
  //analogWrite(rightMotorMagnitude, abs(right));
}
