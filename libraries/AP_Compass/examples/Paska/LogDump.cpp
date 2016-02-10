#include <math.h>
#include <string.h>
#include "Logging.h"
#include "NVState.h"

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
  float av = fabs(v);
  
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
  } else if(fabs(av - 1.0) < 0.001){
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
        consolePrint("."); {
	consoleFlush();
      }
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
