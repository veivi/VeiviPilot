#include <AP_HAL/AP_HAL.h>
#include "Serial.h"

extern const AP_HAL::HAL& hal;

void serialOut(uint8_t c)
{
  hal.uartA->write(c);
}

void serialFlush()
{
  // hal.uartA->flush();
}

