#ifndef CRC16_H
#define CRC16_H

#include <stdint.h>

uint16_t crc16_update(uint16_t crc, uint8_t a);
uint16_t crc16(uint16_t initial, const uint8_t *data, int len);

#endif



