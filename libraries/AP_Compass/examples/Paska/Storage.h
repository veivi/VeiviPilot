#include <stdint.h>
#include "NewI2C.h"
#include "Status.h"

#define EXT_EEPROM_LATENCY 6000

extern uint32_t writeBytesCum;

void waitEEPROM(uint32_t addr);
void writeEEPROM(uint32_t addr, const uint8_t *data, int bytes);
bool readEEPROM(uint32_t addr, uint8_t *data, int size);
void cacheFlush(void);
void cacheWrite(uint32_t addr, const uint8_t *value, int size);
void cacheRead(uint32_t addr, uint8_t *value, int32_t size);
int32_t cacheReadIndirect(uint32_t addr, uint8_t **value, int32_t size);

