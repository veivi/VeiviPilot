#include <stdarg.h>
#include <AP_HAL/AP_HAL.h>
#include "Logging.h"
#include "NVState.h"
#include "Filter.h"
#include "Datagram.h"

extern "C" {
#include "CRC16.h"
}

extern const AP_HAL::HAL& hal;

typedef enum { invalid_c, find_stamp_c, find_start_c, ready_c, stop_c, run_c, failed_c } logState_t;

logState_t logState;
int32_t logPtr, logLen, logSize;
uint16_t logEndStamp;
bool logEnabled = false;
long logBytesCum;

#define logOffset stateRecord.logPartition

bool logReady(void)
{
  if(logState == stop_c || logState == run_c)
    return true;

  consoleNoteLn_P(PSTR("Log not ready"));
  return false;
}

static void logWrite(int32_t index, const uint16_t *value, int count)
{
  if(logSize < 1)
    return;

  if(index+count > logSize) {
    int32_t p = logSize - index;
    cacheWrite(logOffset + index*sizeof(uint16_t), (const uint8_t*) value, p*sizeof(uint16_t));
    cacheWrite(logOffset, (const uint8_t*) &value[p], (count-p)*sizeof(uint16_t));
  } else
    cacheWrite(logOffset + index*sizeof(uint16_t), (const uint8_t*) value, count*sizeof(uint16_t));
}

static void logWrite(int32_t index, const uint16_t value)
{
  logWrite(index, &value, 1);
}

uint16_t logRead(int32_t index)
{
  if(logSize < 1)
    return 0xFFFF;
    
  uint16_t entry = 0;
  
  cacheRead(logOffset + index*sizeof(entry), (uint8_t*) &entry, sizeof(entry));
  
  return entry;
}

static void logCommit(int count)
{
  if(!logReady())
    return;
    
  logPtr = logIndex(count);
  logBytesCum += count*sizeof(uint16_t);

  logLen += count;
  
  if(logLen > logSize)
    logLen = logSize;
}

bool logDirty = false;

static void logEnter(const uint16_t *value, int count)
{
  if(!logReady())
    return;
    
  logWrite(logIndex(0), value, count);
  uint16_t buffer[2] = { ENTRY_TOKEN(t_stamp), logEndStamp };
  logWrite(logIndex(count), buffer, 2);
  logCommit(count);
  
  logDirty = true;
}

static void logEnter(uint16_t value)
{
  logEnter(&value, 1);
}

void logClear(void)
{
  if(!logReady())
    return;
    
  consoleNoteLn_P(PSTR("Log being CLEARED"));
    
  logEnter(ENTRY_TOKEN(t_start));

  logLen = 0;
  stateRecord.logStamp++;
  storeNVState();
}
  
static int prevCh = -1;

static void logWithCh(int ch, uint16_t value)
{
  if(logState != run_c)
    return;

  value = ENTRY_VALUE(value);    // Constrain to valid range
  
  if(!logChannels[ch].tick && value == logChannels[ch].value)
    // Repeat value, not stored
    
    return;
            
  if(ch == prevCh) {
    // Same channel as previous, store as delta
    
    logEnter(ENTRY_TOKEN(t_delta) | (DELTA_MASK & ((value - logChannels[ch].value)>>1)));
    
  } else if(prevCh > -1 && ch == prevCh + 1) {
    // Channel prediction good, just store value
    
    logEnter(value);
  } else {
    // Set channel first
    
    uint16_t buffer[2] = { ENTRY_TOKEN(t_channel + ch), value };
    logEnter(buffer, 2);
  }
    
  logChannels[ch].value = value;
  prevCh = ch;
}

void logGeneric(int ch, float value)
{
  float small = logChannels[ch].small, large = logChannels[ch].large;
  
  logWithCh(ch, (uint16_t) ((float) VALUE_MASK*((value-small)/(large-small))));
}

void logMark(void)
{
  logEnter(ENTRY_TOKEN(t_mark));
}

void logEnable()
{
  if(logEnabled)
    return;
    
  logEnabled = true;
  
  for(int i = 0; i < lc_channels; i++)
    logChannels[i].value = TOKEN_MASK;
  
  prevCh = -1;  
  
  consoleNoteLn_P(PSTR("Logging ENABLED"));
}

void logDisable()
{
  if(!logEnabled)
    return;
    
  logMark();
  logEnabled = false;
  
  consoleNoteLn_P(PSTR("Logging DISABLED"));
}

void logDumpBinary(void)
{
  struct LogInfo info = { stateRecord.logStamp };
  strncpy(info.name, paramRecord.name, NAME_LEN);

  datagramTxStart(DG_LOGINFO);    
  datagramTxOut((const uint8_t*) &info, sizeof(info));
  datagramTxEnd();
      
  for(int32_t i = 0; i < logLen; i++) {
    if((i & ((1UL<<9)-1)) == 0) {
      datagramTxEnd();
      datagramTxStart(DG_LOGDATA);
    }
      
    const uint16_t buf = logRead(logIndex(-logLen+i));
    datagramTxOut((uint8_t*) &buf, sizeof(buf));
  }

  datagramTxEnd();

  // Finish with an empty block
  
  datagramTxStart(DG_LOGDATA);
  datagramTxEnd();
}

bool logInit(uint32_t maxDuration)
{
  uint32_t current = hal.scheduler->micros();
  static int32_t endPtr = -1, startPtr = -1, searchPtr = 0;
  static bool endFound = false;
  uint32_t eepromSize = 0;
  uint8_t dummy;
  
  switch(logState) {
  case invalid_c:
    logSize = 0;
    
    while(!readEEPROM(eepromSize, &dummy, 1))
      eepromSize += 1<<10;
    
    if(!readEEPROM(eepromSize-1, &dummy, 1)) {
      logSize = (eepromSize - logOffset)/sizeof(uint16_t);

      consoleNote_P(PSTR("Inferred log size = "));
      consolePrint(logSize/(1<<10));
      consolePrint("k+");
      consolePrint(logSize%(1<<10));
      consolePrintLn(" entries");
      
      logState = find_stamp_c;
      logLen = -1;

        return false;
    } else {
      consoleNoteLn_P(PSTR("Log EEPROM failed"));
      logState = failed_c;
    }
    break;

  case find_stamp_c:
    while(searchPtr < logSize) {
      if(searchPtr % (1<<12) == 0) {
	consoleNote_P(PSTR("  Searching for log STAMP at "));
	consolePrint(searchPtr);
	consolePrintLn("...");
      }
        
      if(logRead(searchPtr) == ENTRY_TOKEN(t_start)) {
	startPtr = searchPtr;
      } else if(logRead(searchPtr) == ENTRY_TOKEN(t_stamp)) {
        uint16_t stamp = logRead(logIndex(searchPtr+1));

        if(endFound && stamp != ENTRY_VALUE(logEndStamp+1))
          break;
            
        endPtr = searchPtr;
        endFound = true;
        logEndStamp = stamp;

        searchPtr++;
      }
      
      searchPtr++;
      
      if(hal.scheduler->micros() - current > maxDuration)
        // Stop for now
        return false;
    }

    if(endFound && endPtr < logSize) {
      logPtr = endPtr;
    
      if(startPtr < 0) {
	// Don't know the log length yet
	consoleNoteLn_P(PSTR("Log STAMP found, looking for START"));
	logState = find_start_c;
      }	else {
	logState = ready_c;
       }
    } else {
      consoleNoteLn_P(PSTR("*** The log is corrupt and is being cleared ***"));
      logEndStamp = 0;
      logPtr = logSize-1;
      logLen = 0;
      logClear();
      logState = ready_c;
    }
    break;

  case find_start_c:
    while(searchPtr < logSize) {
      if(searchPtr % (1<<12) == 0) {
       	consoleNote_P(PSTR("  Searching for log START at "));
	consolePrint(searchPtr);
	consolePrintLn("...");
      }
	
      if(logRead(searchPtr) == ENTRY_TOKEN(t_start))
	startPtr = searchPtr;
      
      searchPtr++;
      
      if(hal.scheduler->micros() - current > maxDuration)
        // Stop for now
        return false;
    }

    logState = ready_c;
    break;

  case ready_c:
    if(startPtr < 0)
      logLen = logSize;
    else
      logLen = (logSize + endPtr - startPtr - 1) % logSize;
    
    consoleNote_P(PSTR("Log ready, length = "));
    consolePrintLn(logLen);
    logState = stop_c;
    return true;
  }

  return false;
}

void logSave(void (*logStartCB)())
{
  switch(logState) {
    case stop_c:
      if(logEnabled) {
        consoleNoteLn_P(PSTR("Logging STARTED"));
      
        logState = run_c;
    
        for(int i = 0; i < 4; i++)
          logMark();

	(*logStartCB)();
      }
      break;
      
    case run_c:
      if(!logEnabled) {
        consoleNoteLn_P(PSTR("Logging STOPPED"));
        logState = stop_c;
      } else if(logDirty) {
        logDirty = false;
        logCommit(2);
        logEndStamp = ENTRY_VALUE(logEndStamp + 1);
      }
      break;
  }
}
