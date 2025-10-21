#ifndef LR1121_MODEM_HAL_CONTEXT_H
#define LR1121_MODEM_HAL_CONTEXT_H

#include <Arduino.h>
#include <SPI.h>

struct lr1121_modem_hal_context_t {
    SPIClass *spi;
    uint8_t cs_pin;
    uint8_t busy_pin;
    uint8_t int_pin; 
    uint8_t reset_pin;
    uint8_t dio7_pin;
    uint8_t dio8_pin;
};

#endif // LR1121_MODEM_HAL_CONTEXT_H
