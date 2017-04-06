#include <stdarg.h>
#include <AP_HAL/AP_HAL.h>
#include "Logging.h"
#include "NVState.h"
#include "Math.h"

extern "C" {
#include "Datagram.h"
#include "CRC16.h"
}

extern const AP_HAL::HAL& hal;

typedef enum { invalid_c, find_stamp_c, find_start_c, ready_c, stop_c, run_c, failed_c } logState_t;

logState_t logState;
int32_t logPtr, logLen, logSize;
uint16_t logEndStamp;
bool logEnabled = false;
long logBytesCum;

#define logOffset nvState.logPartition

bool logReady(bool verbose)
{
  if(logState == stop_c || logState == run_c)
    return true;

  if(verbose)
    consoleNoteLn_P(PSTR("Log not ready"));
  return false;
}

bool logReady(void)
{
  return logReady(true);
}

uint32_t logAddr(int32_t index)
{
  return logOffset + index*sizeof(uint16_t);
}

static int uncommitted = 0;
static uint16_t nextStamp;

static void logWrite(int32_t index, const uint16_t *value, int count)
{
  if(logSize < 1)
    return;

  if(index+count > logSize) {
    int32_t p = logSize - index;
    cacheWrite(logAddr(index), (const uint8_t*) value, p*sizeof(uint16_t));
    cacheWrite(logAddr(0), (const uint8_t*) &value[p], (count-p)*sizeof(uint16_t));
  } else
    cacheWrite(logAddr(index), (const uint8_t*) value, count*sizeof(uint16_t));

  uncommitted = count;
}

uint16_t logRead(int32_t index)
{
  if(logSize < 1)
    return 0xFFFF;
    
  uint16_t entry = 0;
  
  cacheRead(logAddr(index), (uint8_t*) &entry, sizeof(entry));
  
  return entry;
}

static void logCommit(void)
{
  if(!uncommitted)
    return;
    
  logPtr = logIndex(uncommitted);
  logBytesCum += uncommitted*sizeof(uint16_t);

  logLen += uncommitted;
  
  if(logLen > logSize)
    logLen = logSize;

  uncommitted = 0;
}

static void logEnter(const uint16_t *value, int count)
{
  logWrite(logPtr, value, count);
  logCommit();

  nextStamp = ENTRY_VALUE(logEndStamp+1);
  uint16_t buffer[] = { ENTRY_TOKEN(t_stamp), nextStamp };

  logWrite(logPtr, buffer, sizeof(buffer) / sizeof(uint16_t));
}

static void logEnter(uint16_t value)
{
  logEnter(&value, 1);
}

void logClear(void)
{
  bool wasNotEmpty = logLen > 0;
  
  consoleNoteLn_P(PSTR("Log being CLEARED"));
  
  logEnter(ENTRY_TOKEN(t_start + (nvState.testNum & 0xFF)));
  
  logLen = 0;
    
  if(wasNotEmpty) {
    nvState.logStamp++;
    storeNVState();
    consoleNote_P(PSTR("Log STAMP incremented to "));
    consolePrintLn(nvState.logStamp);
  }    
}

void logInit()
{
}

void logTestSet(uint16_t ch)
{
  nvState.testNum = ch;
  
  logClear();
  storeNVState();

  consoleNote_P(PSTR("Test channel set to "));
  consolePrintLn(nvState.testNum);
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
    
    logEnter(ENTRY_TOKEN(t_delta)
	     | (DELTA_MASK & ((value - logChannels[ch].value)>>1)));
    
  } else if(prevCh > -1 && ch == prevCh + 1) {
    // Channel prediction good, just store value
    
    logEnter(value);
  } else {
    // Set channel first
    
    uint16_t buffer[] = { ENTRY_TOKEN(t_channel + ch), value };
    logEnter(buffer, sizeof(buffer)/sizeof(uint16_t));
  }
    
  logChannels[ch].value = value;
  prevCh = ch;
}

void logGeneric(int ch, float value)
{
  float small = logChannels[ch].small, large = logChannels[ch].large;
  
  logWithCh(ch, (uint16_t)
	    clamp((float) VALUE_MASK*((value-small)/(large-small)),
		  0, VALUE_MASK));
}

void logMark(void)
{
  if(!logEnabled)
    return;
  
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
  
  for(int i = 0; i < 3; i++)
    logMark();
  
  consoleNoteLn_P(PSTR("Logging ENABLED"));
}

void logDisable()
{
  if(!logEnabled)
    return;
    
  logEnabled = false;
  
  consoleNoteLn_P(PSTR("Logging DISABLED"));
}

void logDumpBinary(void)
{
  if(!logReady())
    return;
  
  struct LogInfo info = { nvState.logStamp, nvState.testNum, logLen };
  strncpy(info.name, vpParam.name, NAME_LEN);

  datagramTxStart(DG_LOGINFO);    
  datagramTxOut((const uint8_t*) &info, sizeof(info));
  datagramTxEnd();

  consoleNoteLn_P(PSTR("PARAMETER RECORD"));
  printParams();
  
  consolePrintLn("");
  consoleNote_P(PSTR("TEST CH = "));
  consolePrintLn(nvState.testNum);
  consolePrintLn("");

  int32_t total = 0, block = 0;

  datagramTxStart(DG_LOGDATA);
      
  while(total < logLen) {
    uint8_t *buffer = NULL;

    int good = cacheReadIndirect(logAddr(logIndex(total-logLen)),
				 &buffer, sizeof(uint16_t)*(logLen-total));

    datagramTxOut(buffer, good);

    block += good;
    
    if(block > 1024-1) {
      datagramTxEnd();
      datagramTxStart(DG_LOGDATA);
      block = 0;
    }

    total += good/sizeof(uint16_t);
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
    
    while(!readEEPROM(eepromSize+(1<<10)-1, &dummy, 1))
      eepromSize += 1<<10;
    
    if(!readEEPROM(eepromSize-1, &dummy, 1)) {
      consoleNote_P(PSTR("EEPROM size = "));
      consolePrint(eepromSize/(1<<10));
      consolePrintLn("k bytes");
      
      logSize = (eepromSize - logOffset)/sizeof(uint16_t);

      consoleNote_P(PSTR("Inferred log size = "));
      consolePrint(logSize/(1<<10));
      consolePrint("k + ");
      consolePrint(logSize%(1<<10));
      consolePrintLn(" entries");
      
      logState = find_stamp_c;
      logPtr = 0;
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

      uint16_t entry = logRead(searchPtr);
      
      if(entry >= ENTRY_TOKEN(t_start) && entry <= ENTRY_TOKEN(t_start+BYTE_MASK)) {
	startPtr = searchPtr;
	nvState.testNum = entry & BYTE_MASK;
      } else if(entry == ENTRY_TOKEN(t_stamp)) {
        uint16_t stamp = logRead(logIndex(searchPtr+1));

        if(endFound && stamp != ENTRY_VALUE(logEndStamp+1))
          break;
            
        endPtr = searchPtr;
        logEndStamp = stamp;
        endFound = true;

        searchPtr++;
      }
      
      searchPtr++;
      
      if(hal.scheduler->micros() - current > maxDuration)
        // Stop for now
        return false;
    }

    if(endFound) {
      logPtr = logIndex(endPtr+2);
      
      if(startPtr < 0) {
	// Don't know the log length yet
	consoleNoteLn_P(PSTR("Log STAMP found, looking for START"));
	logState = find_start_c;
      }	else {
	logState = ready_c;
       }
    } else {
      consoleNoteLn_P(PSTR("Log appears corrupted"));
      consoleNoteLn_P(PSTR("Log being INITIALIZED"));
  
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

      uint16_t entry = logRead(searchPtr);
      
      if(entry >= ENTRY_TOKEN(t_start) && entry <= ENTRY_TOKEN(t_start+BYTE_MASK)) {
	startPtr = searchPtr;
	nvState.testNum = entry & BYTE_MASK;
      }
      
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

    consoleNote_P(PSTR("LOG READY, PTR = "));
    consolePrintLn(logPtr);
    consoleNote_P(PSTR("  TEST = "));
    consolePrintLn(nvState.testNum);
    consoleNote_P(PSTR("  LENGTH = "));
    consolePrintLn(logLen);
    consoleNote_P(PSTR("  STAMP = "));
    consolePrintLn(logEndStamp);
    
    logState = stop_c;
    
    datagramTxStart(DG_READY);
    datagramTxEnd();
    return true;

  default:
    return true;
  }

  return false;
}

void logSave(void (*logStartCB)())
{
  if(logState == stop_c && logEnabled) {
    logState = run_c;
    
    for(int i = 0; i < 4; i++)
      logMark();

    (*logStartCB)();

    consoleNoteLn_P(PSTR("Logging STARTED"));
      
  } else if(logState == run_c) {

    if(uncommitted > 0) {
      logCommit();
      cacheFlush();
      logEndStamp = nextStamp;
    }
    
    if(!logEnabled) {
      logState = stop_c;
      consoleNoteLn_P(PSTR("Logging STOPPED"));
    }
  }
}
