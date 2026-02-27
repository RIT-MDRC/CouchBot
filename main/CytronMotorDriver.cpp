#include "CytronMotorDriver.hpp"

CytronMD::CytronMD(MODE mode, ledc_channel_t channel, gpio_num_t in_pin, gpio_num_t an_pin)
      : _channel(channel)
{
      // Prepare and then apply the LEDC PWM timer configuration
      ledc_timer_config_t ledc_timer = {
            .speed_mode       = LEDC_LOW_SPEED_MODE,
            .duty_resolution  = LEDC_TIMER_12_BIT,
            .timer_num        = LEDC_TIMER_2,
            .freq_hz          = 4000,  // Set output frequency at 4 kHz
            .clk_cfg          = LEDC_AUTO_CLK,
      };
      ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

      ledc_channel_config_t ledc_channel = {
            .gpio_num       = (int) an_pin,
            .speed_mode     = LEDC_LOW_SPEED_MODE,
            .channel        = channel,
            .timer_sel      = LEDC_TIMER_2,
            .duty           = 0, // Set duty to 0%
            .hpoint         = 0
      };
      ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
      
      _mode = mode;
      _dir_pin = in_pin;
      _an_pin = an_pin;

      gpio_config_t io_conf = {};
      io_conf.intr_type = GPIO_INTR_DISABLE;
      io_conf.mode = GPIO_MODE_OUTPUT;
      io_conf.pin_bit_mask = (1ULL<<in_pin);
      io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
      io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    
      gpio_config(&io_conf);
      gpio_set_level(in_pin, 0);
}

void CytronMD::setSpeed(int16_t speed)
{
      // Make sure the speed is within the limit.
      if (speed > 255)
      {
            speed = 255;
      }
      else if (speed < -255)
      {
            speed = -255;
      }

      // Set the speed and direction.
      switch (_mode)
      {
      case PWM_DIR:
            if (speed >= 0)
            {
                  ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, _channel, ((speed/255.0) * 4096)));
                  gpio_set_level(_dir_pin, 0);
            }
            else
            {
                  ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, _channel, ((-speed/255.0) * 4096)));
                  gpio_set_level(_dir_pin, 1);
            }
            break;

      //! UNIMPLEMENTED
      case PWM_PWM:
            if (speed >= 0)
            {
            }
            else
            {
            }
            break;
      }

      // Update duty to apply the new value
      ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, _channel));
}
