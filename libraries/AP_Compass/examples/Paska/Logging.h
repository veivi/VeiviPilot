#ifndef LOGGING_H
#define LOGGING_H

#include <stdint.h>
#include "Storage.h"
#include "Console.h"
#include "Log.h"

extern bool logEnabled;
extern long logBytesCum;

bool logInit(uint32_t);
void logClear(void);
void logDump(int ch);
void logDumpBinary(void);

void logGeneric(int ch, float value);
void logMark();

void logEnable();
void logDisable();

void logSave(void (*logStartCB)());
  
#endif
