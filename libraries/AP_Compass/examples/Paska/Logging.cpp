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

#define logIndex(i) ((logPtr + logSize + (i)) % logSize)

typedef enum { invalid_c, init_c, stop_c, run_c, failed_c } logState_t;

logState_t logState;
int32_t logPtr, logLen, logSize;
uint16_t logEndStamp;
bool logEnabled = false;
long logBytesCum;

#define logOffset stateRecord.logPartition

static bool logReady(void)
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
  
  logLen = -1L;
}

static void logWrite(int32_t index, const uint16_t value)
{
  logWrite(index, &value, 1);
}

static uint16_t logRead(int32_t index)
{
  if(logSize < 1)
    return 0xFFFF;
    
  uint16_t entry = 0;
  
  cacheRead(logOffset + index*sizeof(entry), (uint8_t*) &entry, sizeof(entry));
  
  return entry;
}

static void logCommit(int bytes)
{
  if(!logReady())
    return;
    
  logPtr = logIndex(bytes);

  logBytesCum += bytes;
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

static int col = 0;
static bool first = false, tick = false;

static void logOutputInit(void)
{
  col = 20;
  first = true;
  tick = false;
}

long valueCount;

static void logOutputValue(float v)
{
  float av = absVal(v);
  
  if(!first) {
    consolePrint(',');
    col++;
  }

  if(col > 72) {
    float progress = (float) valueCount / logLen / lc_channels;
    consolePrint(" // ");
    consolePrint(100.0*progress, 0);
    consolePrintLn("%");
    col = 0;
  }
  
  if(av < 0.001) {
    col++;
    consolePrint("0");
  } else if(absVal(av - 1.0) < 0.001){
    consolePrint(v < 0.0 ? "-1" : "1");
    col += v < 0.0 ? 2 : 1;
  } else {
    int decimals = av < 1 ? 3 : av < 10 ? 2 : av < 100 ? 1 : 0;
    consolePrint(v, decimals);
    col += 3 + (v < 0.0 ? 1 : 0) + (decimals > 0 ? 1 : 0) + (av < 1.0 ? 1 : 0)
      + (av >= 1000.0 ? 1 : 0) + (av >= 10000.0 ? 1 : 0);
  }

  first = false;
}

static void logOutputString(const char *s)
{  
  if(!first) {
    consolePrint(",");
    col++;
  }

  if(col > 72) {
    consolePrintLn("");
    col = 0;
  }

  consolePrint("\"");
  consolePrint(s);
  consolePrint("\"");
  
  col += strlen(s) + 2;

  first = false;
}

static void logOutputVariableName(int stamp, const char *name)
{  
  if(!first) {
    consolePrint(";");
    col++;
  }

  if(col > 72) {
    consolePrintLn("");
    col = 0;
  }

  consolePrint("fdr_");
  consolePrint(stamp);
  consolePrint("_");
  consolePrint(name);
  
  col += 4 + 3 + 1 + strlen(name);

  first = false;
}

static void logOutputValue(float small, float large)
{
  logOutputValue(tick ? small : large);
  tick = !tick;
}

void logDump(int ch)
{
  if(!logReady())
    return;
  
  if(ch < 0) {
    valueCount = 0;
    
    for(ch = 0; ch < lc_channels; ch++)
      logDump(ch);

    consolePrint("fdr_");
    consolePrint(stateRecord.logStamp);
    consolePrint("_matrix = [ ");

    logOutputInit();

    for(ch = 0; ch < lc_channels; ch++)
      logOutputVariableName(stateRecord.logStamp, logChannels[ch].name);
    
    consolePrint(" ]\n");

    consoleNoteLn_P(PSTR("FLIGHT DATA RECORD WITH INDEX"));
    
    consolePrint("fdr_");
    consolePrint(stateRecord.logStamp);
    consolePrint(" = { fdr_");
    consolePrint(stateRecord.logStamp);
    consolePrint("_matrix, ");
    
    logOutputInit();

    for(ch = 0; ch < lc_channels; ch++)
      logOutputString(logChannels[ch].name);
    
    consolePrintLn(" }");
    
    return;
  }
  
  int32_t len = logLen;

  if(len < 0) {
    consoleNote_P(PSTR("Looking for log start..."));
    
    len = 0;
    
    while(len < logSize-1 && logRead(logIndex(-len-1)) != ENTRY_TOKEN(t_start)) {
      if(len % 5000 == 0)
        consolePrint(".");
      len++;
    }
        
    consolePrint(" found, log length = ");
    consolePrintLn(len);
  
    logLen = len;
  }
  
  consoleNote_P(PSTR("CHANNEL "));
  consolePrint(ch);
  consolePrint(" (");
  consolePrint(logChannels[ch].name);
  consolePrintLn(") DATA");
  
  consolePrint("fdr_");
  consolePrint(stateRecord.logStamp);
  consolePrint("_");
  consolePrint(logChannels[ch].name);
  consolePrint(" = [ ");

  float small = logChannels[ch].small, large = logChannels[ch].large;
  int currentCh = -1, nextCh = -1;
  uint16_t valueRaw = 0;
  bool valueValid = false;
  float value = 0.0;

  logOutputInit(); // Initialize
    
  for(int32_t i = 0; i < logLen; i++) {    
    valueCount++;
  
    uint16_t entry = logRead(logIndex(-logLen+i));

    if(ENTRY_IS_TOKEN(entry) && ENTRY_VALUE(entry) < t_delta) {
      LogToken_t token = (LogToken_t) ENTRY_VALUE(entry);
      
      switch(token) {
        case t_stamp:
          // End marker, a count follows, ignore both
          i++;
          break;
          
        case t_start:
          break;
          
        case t_mark:
          // Mark
                      
          for(int j = 0; j < 10; j++)
            logOutputValue(small, large); 
          break;
          
        default:
          if(token >= t_channel && token < t_channel+lc_channels) {
            // Valid channel id
      
            nextCh = token - t_channel;
          } else {
            // Invalid token
      
            consolePrint(" // *** Invalid entry ");
            consolePrintLn(entry);
            break;
          }
      }
    } else {
      // A log value entry

      if(!ENTRY_IS_TOKEN(entry))
        currentCh = nextCh;
      
      if(logChannels[currentCh].tick) {
        if(valueValid)
          logOutputValue(value);
        else
          logOutputValue(small, large);
      }

      if(currentCh == ch) {
        if(ENTRY_IS_TOKEN(entry)) {
          // Delta value
                  
          valueRaw = ENTRY_VALUE(valueRaw + ((ENTRY_VALUE(entry) - t_delta) << 1));
                    
        } else {
          // Absolute value
                    
          valueRaw = entry;
        }
        
        // value = (float) valueRaw;
	value = small + (float) valueRaw / (float) VALUE_MASK * (large - small);
        valueValid = true;
      }

      if(currentCh > -1)
        nextCh = currentCh + 1;
    }
  }

  consolePrintLn(" ]");
}

void logDumpBinary(void)
{
  for(int32_t i = 0; i < logSize; i++) {
    if((i & ((1UL<<9)-1)) == 0) {
      datagramEnd();
      datagramStart();
    }
      
    const uint16_t buf = logRead(i);
    datagramOutput(buf & 0xFF);
    datagramOutput(buf>>8);
  }

  datagramEnd();

  consoleNoteLn_P(PSTR("LOG DUMP COMPLETED"));
}

bool logInit(uint32_t maxDuration)
{
  uint32_t current = hal.scheduler->micros();
  static int32_t endPtr = -1, searchPtr = 0;
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
      
      logState = init_c;
      logLen = -1;

        return false;
    } else {
      consoleNoteLn_P(PSTR("Log EEPROM failed"));
      logState = failed_c;
    }
    break;

  case init_c:
    while(searchPtr < logSize) {
      if(logRead(searchPtr) == ENTRY_TOKEN(t_stamp)) {
        uint16_t stamp = logRead(logIndex(searchPtr+1));

        if(stamp % 100 == 0) {
          consoleNote_P(PSTR("  Searching for log end at "));
          consolePrint(searchPtr);
          consolePrintLn("...");
        }
        
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

    logState = stop_c;
      
    if(endFound && endPtr < logSize) {
      logPtr = endPtr;
    
      consoleNoteLn_P(PSTR("LOG READY"));
    } else {
      consoleNoteLn_P(PSTR("*** The log is corrupt and is being cleared ***"));
      logEndStamp = 0;
      logPtr = logSize-1;  
      logClear();
    }
    break;
  }

  return true;
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
