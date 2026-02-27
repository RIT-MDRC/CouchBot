#pragma once

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

// RC Transmitter
#define SBUS_SIG GPIO_NUM_3

// Fun stuffs
#define HORN_TRG GPIO_NUM_12
#define BUZZER GPIO_NUM_13

// Encoders
#define R_ENC_A GPIO_NUM_18
#define R_ENC_B GPIO_NUM_19
#define L_ENC_A GPIO_NUM_2
#define L_ENC_B GPIO_NUM_4

// Motor Controller
#define IN1 GPIO_NUM_27
#define IN2 GPIO_NUM_26
#define AN1 GPIO_NUM_33
#define AN2 GPIO_NUM_32


// RC Joystick ranges
#define JOY_MAX 1695
#define JOY_MIN 306

// =============================