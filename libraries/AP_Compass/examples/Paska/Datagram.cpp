#include <stdint.h>
#include "Serial.h"

extern "C" {
#include "CRC16.h"
}

uint16_t crcState;

void outputBreak()
{ 
  serialOut((const uint8_t) 0x00);
  serialOut((const uint8_t) 0x00);
}

void datagramStart(void)
{
  outputBreak();
  serialOut((const uint8_t) 0x01);
  crcState = 0xFFFF;
}

void datagramOutput(const uint8_t c)
{
  serialOut(c);
    
  if(c == 0x00)
    serialOut((const uint8_t) 0xFF);

  crcState = crc16_update(crcState, c);
}

void datagramEnd(void)
{
  uint16_t buf = crcState;
  datagramOutput(buf>>8);
  datagramOutput(buf & 0xFF);
  outputBreak();
}


