#pragma once
#include <memory.h>
#include "driver/gpio.h"
#include "led.hpp"

enum MODE {
  PWM_DIR,
  PWM_PWM,
};

class CytronMD
{
  public:
    CytronMD(MODE mode, ledc_channel_t channel,  gpio_num_t in_pin, gpio_num_t an_pin);
    void setSpeed(int16_t speed);

private:
    // std::vector<espp::Led::ChannelConfig> led_channels;
    // espp::Led _ledc;
    ledc_channel_t _channel;

  protected:
    MODE _mode;
  	gpio_num_t _dir_pin;
    gpio_num_t _an_pin;
};