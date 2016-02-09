#include <AP_HAL/AP_HAL.h>
#include "Serial.h"

extern const AP_HAL::HAL& hal;

#define BUF_SIZE (1<<6)

char outputBuf[BUF_SIZE];
uint8_t bufPtr;

void serialFlush()
{
  for(uint8_t i = 0; i < bufPtr; i++)
    hal.uartA->write(outputBuf[i]);
    
  bufPtr = 0;
}

void serialOut(const uint8_t c)
{
  if(bufPtr > BUF_SIZE-1)
    serialFlush();

  outputBuf[bufPtr++] = c;
}
