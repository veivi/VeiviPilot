#include "Storage.h"
#include "NewI2C.h"
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

#define EXT_EEPROM_PAGE (1L<<6)
#define PAGE_MASK ~(EXT_EEPROM_PAGE-1)
#define EEPROM_I2C_ADDR 80

extern NewI2C I2c;

static int eepromFailCount = 0;
static uint32_t lastWriteTime;
uint8_t cacheData[EXT_EEPROM_PAGE];
bool cacheFlag[EXT_EEPROM_PAGE];
bool cacheValid, cacheModified;
uint32_t cacheTag;
int logBytesCum;
float logBandWidth;

bool eepromWarn = false, eepromFailed = false;

void waitEEPROM(uint32_t addr);
void writeEEPROM(uint32_t addr, const uint8_t *data, int bytes);
bool readEEPROM(uint32_t addr, uint8_t *data, int size);
void cacheFlush(void);
void cacheWrite(uint32_t addr, const uint8_t *value, int size);
void cacheRead(uint32_t addr, uint8_t *value, int size);

void waitEEPROM(uint32_t addr)
{
  if(hal.scheduler->micros() - lastWriteTime > EXT_EEPROM_LATENCY)
    // We're cool
    return;
    
  // Write latency not met, wait for acknowledge

  handleFailure("EEPROM wait",
     I2c.wait((uint8_t) (EEPROM_I2C_ADDR
     			+ (uint8_t) ((addr>>16) & 0x7))) != 0, 
                  	&eepromWarn, &eepromFailed, &eepromFailCount);
}

void writeEEPROM(uint32_t addr, const uint8_t *data, int bytes) 
{
  if(eepromFailed)
    return;
    
  waitEEPROM(addr);
  bool fail = I2c.write(  (uint8_t) EEPROM_I2C_ADDR + (uint8_t) ((addr>>16) & 0x7), 
                             (uint16_t) (addr & 0xFFFFL), 
                             data, bytes) != 0;
  handleFailure("EEPROM write", fail, &eepromWarn, &eepromFailed, &eepromFailCount);
  lastWriteTime = hal.scheduler->micros();
}
 
bool readEEPROM(uint32_t addr, uint8_t *data, int size) 
{
  if(eepromFailed)
    return true;
    
  waitEEPROM(addr);

  bool fail = I2c.read((uint8_t) EEPROM_I2C_ADDR + (uint8_t) ((addr>>16) & 0x7), (uint16_t) (addr & 0xFFFFL), data, size) != 0;
  
  return handleFailure("EEPROM read", fail, &eepromWarn, &eepromFailed, &eepromFailCount);

}

void cacheFlush(void)
{
  if(!cacheModified)
    return;
    
  int i = 0;
  
  do {
    while(!cacheFlag[i] && i < EXT_EEPROM_PAGE)
      i++;
    
    if(i < EXT_EEPROM_PAGE) {
      int l = 0;
      
      while(cacheFlag[i+l] && i+l < EXT_EEPROM_PAGE) {
        cacheFlag[i+l] = false;
        l++;
      }

      writeEEPROM(cacheTag + i, &cacheData[i], l);
        
      i += l;
    }
  } while(i < EXT_EEPROM_PAGE);
  
  cacheModified = false;
}

#define CACHE_TAG(a) ((a) & PAGE_MASK)

static bool cacheHit(uint32_t addr)
{
  return CACHE_TAG(addr) == cacheTag;
}

static void cacheAlloc(uint32_t addr)
{
  cacheFlush();
  cacheTag = CACHE_TAG(addr);
  cacheValid = false;
}

static void cacheWriteLine(uint32_t addr, const uint8_t *value, int size)
{  
  if(!cacheHit(addr))
    cacheAlloc(addr);
    
  for(int i = 0; i < size; i++) {
    cacheData[(addr & ~PAGE_MASK) + i] = value[i];
    cacheFlag[(addr & ~PAGE_MASK) + i] = true;
  }
  
  cacheValid = false;
  cacheModified = true;
  logBytesCum += size;
}

void cacheWrite(uint32_t addr, const uint8_t *value, int size)
{
  if(CACHE_TAG(addr) != CACHE_TAG(addr+size-1)) {
    uint32_t split = CACHE_TAG(addr+size-1);
    
    cacheWriteLine(addr, value, split - addr);
    cacheWriteLine(split, &value[split - addr], size - (split - addr));
  } else
    cacheWriteLine(addr, value, size);
}

void cacheReadLine(uint32_t addr, uint8_t *value, int size) 
{
  if(cacheModified || !cacheHit(addr))
    cacheAlloc(addr);
  
  if(!cacheValid) {
    if(readEEPROM(cacheTag, cacheData, EXT_EEPROM_PAGE)) {
      for(int i = 0; i < EXT_EEPROM_PAGE; i++)
        cacheData[i] = 0xFF;
    }
    cacheValid = true;
  }
  
  for(int i = 0; i < size; i++)
    value[i] = cacheData[(addr & ~PAGE_MASK) + i];  
}

void cacheRead(uint32_t addr, uint8_t *value, int size) 
{
  if(CACHE_TAG(addr) != CACHE_TAG(addr+size-1)) {
    uint32_t split = CACHE_TAG(addr+size-1);
    
    cacheReadLine(addr, value, split - addr);
    cacheReadLine(split, &value[split - addr], size - (split - addr));
  } else
    cacheReadLine(addr, value, size);
}
