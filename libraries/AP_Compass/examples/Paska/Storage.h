#include <stdint.h>
#include "NewI2C.h"

#define EXT_EEPROM_LATENCY 6000

extern long writeBytesCum;
extern bool eepromWarn, eepromFailed;

void waitEEPROM(uint32_t addr);
void writeEEPROM(uint32_t addr, const uint8_t *data, int bytes);
bool readEEPROM(uint32_t addr, uint8_t *data, int size);
void cacheFlush(void);
void cacheWrite(uint32_t addr, const uint8_t *value, int size);
void cacheRead(uint32_t addr, uint8_t *value, int size);

