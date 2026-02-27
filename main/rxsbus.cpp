#include "rxsbus.hpp"
#include "driver/uart.h"

RXSBUS::RXSBUS(uart_port_t channel, int rx_pin) {
  _channel = channel;
  
  uart_config_t uart_config = {
    .baud_rate = 100000,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_DEFAULT
  };

  ESP_ERROR_CHECK(uart_driver_install(_channel, SBUS_BUF_SIZE*2, 0, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_param_config(_channel, &uart_config));
  ESP_ERROR_CHECK(uart_set_pin(_channel, UART_PIN_NO_CHANGE, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  ESP_ERROR_CHECK(uart_set_line_inverse(_channel, uart_signal_inv_t::UART_SIGNAL_RXD_INV));
}

bool RXSBUS::_parse() {

  int len = 1;
  
  while (len > 0){
    // Search for starting bit
    len = uart_read_bytes(_channel, _payload, 1, 20/portTICK_PERIOD_MS);
    if (len > 0 && _payload[0] == 0x0F) {
      // read up to known packet length
      len = uart_read_bytes(_channel, _payload, 24, 20/portTICK_PERIOD_MS);

      // Check for end bit/footer
      if (len == 24 && _payload[23] == 0x00) {
        return true;
      }
    }
  }

  // Return false for partial or invalid packet
  return false;
}

bool RXSBUS::read(volatile uint16_t *channels, volatile bool *failsafe, volatile bool *lost_packets) {

  if (_parse()) {
    if (channels) {
      channels[0]  = (uint16_t) ((_payload[0]    |_payload[1] <<8)                     & 0x07FF);
      channels[1]  = (uint16_t) ((_payload[1]>>3 |_payload[2] <<5)                     & 0x07FF);
      channels[2]  = (uint16_t) ((_payload[2]>>6 |_payload[3] <<2 |_payload[4]<<10)  	 & 0x07FF);
      channels[3]  = (uint16_t) ((_payload[4]>>1 |_payload[5] <<7)                     & 0x07FF);
      channels[4]  = (uint16_t) ((_payload[5]>>4 |_payload[6] <<4)                     & 0x07FF);
      channels[5]  = (uint16_t) ((_payload[6]>>7 |_payload[7] <<1 |_payload[8]<<9)   	 & 0x07FF);
      channels[6]  = (uint16_t) ((_payload[8]>>2 |_payload[9] <<6)                     & 0x07FF);
      channels[7]  = (uint16_t) ((_payload[9]>>5 |_payload[10]<<3)                     & 0x07FF);
      channels[8]  = (uint16_t) ((_payload[11]   |_payload[12]<<8)                     & 0x07FF);
      channels[9]  = (uint16_t) ((_payload[12]>>3|_payload[13]<<5)                     & 0x07FF);
      channels[10] = (uint16_t) ((_payload[13]>>6|_payload[14]<<2 |_payload[15]<<10) 	 & 0x07FF);
      channels[11] = (uint16_t) ((_payload[15]>>1|_payload[16]<<7)                     & 0x07FF);
      channels[12] = (uint16_t) ((_payload[16]>>4|_payload[17]<<4)                     & 0x07FF);
      channels[13] = (uint16_t) ((_payload[17]>>7|_payload[18]<<1 |_payload[19]<<9)  	 & 0x07FF);
      channels[14] = (uint16_t) ((_payload[19]>>2|_payload[20]<<6)                     & 0x07FF);
      channels[15] = (uint16_t) ((_payload[20]>>5|_payload[21]<<3)                     & 0x07FF);
    }

    if (failsafe) {
      *failsafe = _payload[22] & 0x08;
    }

    if (lost_packets) {
      *lost_packets = _payload[22] & 0x04;
    }

    // Read success
    return true;
  }
  // Read fail, bad packets
  return false;
}