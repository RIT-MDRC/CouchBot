#include "CytronMotorDriver.h" 

//define RC pins (channels can be changed in controller settings)
#define CH1 8 //right x
#define CH2 2 //right y
#define CH3 3 //left y (not springy)
#define CH4 4 //left x
#define CH5 7 //switch a
#define CH6 12 //switch c (3-position)
#define CH7 13 //switch f
#define CH8 9 //switch h (springy)

//define thumbpad pins
#define joyX_pin A3
#define joyY_pin A2
#define joyButt_pin 

//pulse-width modulation
#define PWM_VCC 13
#define PWM_GND 12

//initialize motors
CytronMD leftMotor(PWM_DIR, 5, 10);
CytronMD rightMotor(PWM_DIR, 6, 11);

//acceleration constants
//clock frequency is ~6hz
const float accelX = 18.0;
const float accelXZero = 30.0;
const float accelYFwd = 20.0;
const float accelYRev = 14;
const float accelYZero = 36;

const float speedMultiplier = 1;
const float accelMultiplier = 6;

int targetX = 0;
int targetY = 0;
int couchX = 0;
int couchY = 0;

const int chXInterval = 5; // the window to integrate over. Increase this value to increase smoothing
float chXRunningSum = 0;  // this is integrated
int chXReadingList[chXInterval]; 
int chXPointer = 0;

const int chYInterval = 5; // the window to integrate over. Increase this value to increase smoothing
float chYRunningSum = 0;  // this is integrated
int chYReadingList[chYInterval]; 
int chYPointer = 0;

int joyYOff;
int joyXOff;

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

  Serial.begin(115200);

  joyXOff = analogRead(joyX_pin) - 1024/2;
  joyYOff = analogRead(joyY_pin) - 1024/2;
  
  //waitForStartCondition();
}

void loop() {
  // this might be done better with delay(ms);
  //read raw values
  int joyX_raw = analogRead(joyX_pin) - joyXOff; //thumbpad
  int joyY_raw = analogRead(joyY_pin) - joyYOff;
  
  int rightX_raw = readcontrol(CH1); //remote control
  int rightY_raw = readcontrol(CH2);
  int leftY_raw = readcontrol(CH3);
  float leftX_raw = readcontrol(CH4);

  //update switch values
  bool CH5_val = updateSwitch(CH5, 2); // rc override
  float CH6_val = updateSwitch(CH6, 3); // speed control / e-stop
  bool CH7_val = updateSwitch(CH7, 2); // siren
  bool CH8_val = updateSwitch(CH8, 2); // horn
  bool ebrake = (CH6_val == 0);

  //map control values
  joyX_raw = map(joyX_raw, 0, 1023, -127, 127);
  joyY_raw = map(joyY_raw, 0, 1023, -255, 255) * (CH5_val + 1);

  rightX_raw = (map(rightX_raw, 1024, 2047, -255, 255)+24);
  rightY_raw = (map(rightY_raw, 1024, 2047, 255, -255)-24) * (CH5_val);
  leftY_raw = (map(leftY_raw, 1024, 2047, 255, 0)-24);
  leftX_raw = (map(leftX_raw, 1024, 2047, 255, -255)-12) * (CH5_val);

  //RC override check
  if(CH5_val == true){
    targetX = rightX_raw * speedMultiplier * CH6_val;
    targetY = rightY_raw * speedMultiplier * CH6_val;
  } else {
    targetX = joyX_raw * speedMultiplier * CH6_val;
    targetY = joyY_raw * speedMultiplier * CH6_val;
  }

  if(couchY >= 0){
    if(targetY > couchY){
      couchY += calculateAccel(targetY, couchY, accelYFwd * CH6_val * accelMultiplier);
    } else { couchY += calculateAccel(targetY, couchY, accelYZero * CH6_val * accelMultiplier * -1);
    }
  } else if(targetY < couchY){
      couchY += calculateAccel(targetY, couchY, accelYRev * CH6_val * accelMultiplier * -1);
    } else { couchY += calculateAccel(targetY, couchY, accelYZero * CH6_val * accelMultiplier);
    }

  if(couchX >= 0){
    if(targetX > couchX){
      couchX += calculateAccel(targetX, couchX, accelX * CH6_val * accelMultiplier);
    } else { couchX += calculateAccel(targetX, couchX, accelXZero * CH6_val * accelMultiplier * -1);
    }
  } else if(targetY < couchY){
      couchX += calculateAccel(targetX, couchX, accelX * CH6_val * accelMultiplier * -1);
    } else { couchX += calculateAccel(targetX, couchX, accelXZero * CH6_val * accelMultiplier);
    }
  
  couchX = smoothControl(couchX, chXRunningSum, chXReadingList, chXPointer, chXInterval);
  couchY = smoothControl(couchY, chYRunningSum, chYReadingList, chYPointer, chYInterval);
  Serial.print(couchX);
  Serial.print(",");
  Serial.println(couchY);

  int leftPower = couchY + couchX;
  int rightPower = couchY - couchX;
  setMotors(leftPower, rightPower);

  
}

int readcontrol(int CH) {
 int wavepeak = pulseIn(CH, HIGH, 25000);
 if(wavepeak == 0){
   wavepeak = pulseIn(CH, HIGH, 25000);
 }
 return wavepeak;
}

void setMotors(int left, int right) {
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);
  leftMotor.setSpeed(left);
  rightMotor.setSpeed(-right);
}

float updateSwitch(int channel, int positions){
  if(positions == 2){
    if(896 < readcontrol(channel) && readcontrol(channel) < 1152){return(1);} else {return(0);}}
  else {
    if(896 < readcontrol(channel) && readcontrol(channel) < 1279){return(1.0);} // high speed
    else if (1280 < readcontrol(channel) && readcontrol(channel) < 1791){return(0.6);} // low speed
    else {return(0);}
    }
}

int smoothControl(int reading, float &runningSum, int valList[], int &indexPointer, int integrationInterval) {
  runningSum -= valList[indexPointer];
  runningSum += reading;
  valList[indexPointer] = reading;

  indexPointer++;
  indexPointer = indexPointer % integrationInterval;

  return (runningSum / integrationInterval);
}

int calculateAccel(int targetSpeed, int couchSpeed, float accel){
  if(abs(targetSpeed - couchSpeed) > abs(accel)){
    return(accel);
    } else { return(targetSpeed - couchSpeed) / 1.25;}
}
