#include "CRC16.h"

uint16_t crc16_update(uint16_t crc, uint8_t a)
{
  int i = 0;
  
  crc ^= a;
  
  for (i = 0; i < 8; ++i) {
    if (crc & 1)
      crc = (crc >> 1) ^ 0xA001U;
    else
      crc = (crc >> 1);
  }
  
  return crc;
}

uint16_t crc16(uint16_t initial, const uint8_t *data, int len)
{
  uint16_t crc = initial;
  int i = 0;
  
  for(i = 0; i < len; i++)
    crc = crc16_update(crc, data[i]);
    
  return crc;
}

