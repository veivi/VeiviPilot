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

void datagramTxStart(uint8_t dg)
{
  outputBreak();
  crcState = 0xFFFF;
  datagramTxOutByte((const uint8_t) dg);
}

void datagramTxOutByte(const uint8_t c)
{
  serialOut(c);
    
  if(c == 0x00)
    serialOut((const uint8_t) 0xFF);

  crcState = crc16_update(crcState, c);
}

void datagramTxOut(const uint8_t *data, int l)
{
  while(l-- > 0)
    datagramTxOutByte(*data++);
}

void datagramTxEnd(void)
{
  uint16_t buf = crcState;
  datagramTxOut((const uint8_t*) &buf, sizeof(buf));
  outputBreak();
}


