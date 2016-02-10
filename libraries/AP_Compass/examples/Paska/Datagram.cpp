#include <stdint.h>
#include "Serial.h"
#include "Datagram.h"

extern "C" {
#include "CRC16.h"
}

uint16_t crcState;

void outputBreak()
{ 
  serialOut((const uint8_t) 0x00);
  serialOut((const uint8_t) 0x00);
}

void datagramStart(uint8_t dg)
{
  outputBreak();
  crcState = 0xFFFF;
  datagramOut((const uint8_t) dg);
}

void datagramOut(const uint8_t c)
{
  serialOut(c);
    
  if(c == 0x00)
    serialOut((const uint8_t) 0xFF);

  crcState = crc16_update(crcState, c);
}

void datagramOut(const uint8_t *data, int l)
{
  while(l-- > 0)
    datagramOut(*data++);
}

void datagramEnd(void)
{
  uint16_t buf = crcState;
  datagramOut(buf>>8);
  datagramOut(buf & 0xFF);
  outputBreak();
}


