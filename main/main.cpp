#include <stdio.h>
#include <chrono>
#include <algorithm>
#include <thread>
#include "driver/gpio.h"
#include "driver/uart.h"

// ESP-CPP components
#include "logger.hpp"
#include "task.hpp"

#include "util.hpp"
#include "config.hpp"
#include "CytronMotorDriver.hpp"
#include "rxsbus.hpp"

using namespace std::chrono_literals;

CytronMD rightMotor(MODE::PWM_DIR, LEDC_CHANNEL_0, IN1, AN1);
CytronMD leftMotor(MODE::PWM_DIR, LEDC_CHANNEL_1, IN2, AN2);

int leftMotorSetpoint = 0;
int rightMotorSetpoint = 0;
float drive = 0.0;
float rot = 0.0;

bool remote_stop = true;
bool drive_enabled = false;
bool stop_tone_played = false;
bool enable_tone_played = false;

// TODO:
// - Encoder interrupts
// - PID motor loop

RXSBUS rc_controller(uart_port_t::UART_NUM_2, 16);

volatile uint16_t rc_channels[16];
volatile bool failsafe = true;
volatile bool lost_packets = true;

espp::Logger logger({.tag="COUCH", .level=espp::Logger::Verbosity::DEBUG});

void update_setpoints() {
  // Not sure how safe this line is
  // Joysticks go below value -> setpoints stay at last value?
  // if ((abs(ly) < JOY_MIN) or (abs(ry) < JOY_MIN) or (abs(rx) < JOY_MIN)) return;

  // Map and reverse ranges (UP on joysticks is lower pwm value)
  float throttle = map_range<int>(rc_channels[2], JOY_MIN, JOY_MAX, 255*THROTTLE_MAX, 0)/255.0;
  float turn_accel = map_range<int>(rc_channels[6], JOY_MIN, JOY_MAX, 255*TURN_MAX, 0)/255.0;

  // DOWN -100 | 100 UP
  float driveSetpoint = map_range<int>(rc_channels[1], JOY_MIN, JOY_MAX, 255, -255)/255.0;
  // Add deadzone and joystick curve (generated using best fit curve in excel)
  // (0.0463 + (1.53*driveSetpoint)+(-0.621 * pow(driveSetpoint, 2))
  driveSetpoint = deadzone(throttle * driveSetpoint, DRIVE_DEADZONE);

  // LEFT 100 | -100 RIGHT
  float rotSetpoint = map_range<int>(rc_channels[0], JOY_MIN, JOY_MAX, 255, -255)/255.0;
  rotSetpoint = deadzone(rotSetpoint * turn_accel, ROT_DEADZONE);

  // Drive acceleration curves
  if ((driveSetpoint-drive) * signum(drive) < 0) {
    // Drive decceleration curve
    if (drive < driveSetpoint) {
      drive = std::min<float>(drive + DECEL, driveSetpoint);
    } else if (drive > driveSetpoint) {
      drive = std::max<float>(drive - DECEL, driveSetpoint);
    }
  } else {
    // Drive acceleration curve
    if (drive < driveSetpoint) {
      drive = std::min<float>(drive + ACCEL, driveSetpoint);
    } else if (drive > driveSetpoint) {
      drive = std::max<float>(drive - ACCEL, driveSetpoint);
    }
  }

  // Rotation acceleration curves
  if (rot < rotSetpoint) {
    rot = std::min<float>(rot + TURN_ACCEL, rotSetpoint);
  } else if (rot > rotSetpoint) {
    rot = std::max<float>(rot - TURN_ACCEL, rotSetpoint);
  }

  rightMotorSetpoint = -1 * ((drive * 255) - (rot * 255));
  leftMotorSetpoint = 1 * ((drive * 255) + (rot * 255));  

  // logger.debug("failsafe: {} | ch1: {} | ch2: {} | ch3: {} | left: {} | right: {} | throttle: {} | turn_accel: {} | rot: {}",
  //   failsafe, rc_channels[0], rc_channels[1], rc_channels[2], leftMotorSetpoint, rightMotorSetpoint, throttle, turn_accel, rotSetpoint
  // );
}

extern "C" void app_main(void)
{

  // Create task for reading SBUS values from reciever
  espp::Task::configure_task_watchdog(1000ms, true);
  auto sbus_periodic = [](std::mutex &m, std::condition_variable &cv) {
    rc_controller.read(&rc_channels[0], &failsafe, &lost_packets);
    // std::unique_lock<std::mutex> lk(m);
    // cv.wait_for(lk, 500ms);
    return false;
  };

  auto sbus_task = espp::Task({.callback = sbus_periodic,
                          .task_config = {.name = "RXSBUS"},
                          .log_level = espp::Logger::Verbosity::DEBUG});
  sbus_task.start();
  sbus_task.start_watchdog();

  // ===== MAIN LOOP =====
  while (true) {

    // Update setpoints if controller connected
    if (!failsafe) {
      update_setpoints();
    }

    // Remote e-stop switch (Ch5, Switch F)
    remote_stop = rc_channels[4] <= 500;

    if (drive_enabled && (failsafe || remote_stop)) {
      drive_enabled = false;
      logger.info("DRIVE DISABLED");
    } else if (
      !drive_enabled
      && !(failsafe || remote_stop)
      && drive == 0.0 && rot == 0.0 // Safety feature: drive stick must be in center position (zero) to enable drive
    ) {
      drive_enabled = true;
      logger.info("DRIVE ENABLED");
    };

    if (drive_enabled) {
      leftMotor.setSpeed(leftMotorSetpoint);
      rightMotor.setSpeed(rightMotorSetpoint);
    } else {
      // E-Stop/No Connection routine

      // Full stop
      leftMotorSetpoint = 0;
      rightMotorSetpoint = 0;
      leftMotor.setSpeed(0);
      rightMotor.setSpeed(0);

      logger.warn("Drive DISABLED | Remote Connection: {} | Remote stop enabled: {} | Stick Zeroed: {}", !failsafe, remote_stop, drive == 0.0 && rot == 0.0);
    }

    std::this_thread::sleep_for(10ms);
  }
}