#pragma once
#include <stdio.h>
#include "driver/uart.h"
#include "logger.hpp"

#define SBUS_BUF_SIZE 1024

class RXSBUS {
    public:
        RXSBUS(uart_port_t channel, int rx_pin);
        bool read(volatile uint16_t *channels, volatile bool *failsafe, volatile bool *lost_packets);

    private:
        bool _parse();
        uint8_t _payload[24];
        uart_port_t _channel;
};