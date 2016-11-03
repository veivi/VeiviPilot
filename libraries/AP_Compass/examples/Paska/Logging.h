#ifndef LOGGING_H
#define LOGGING_H

#include <stdint.h>
#include "Storage.h"
#include "Console.h"

extern "C" {
#include "Log.h"
}

extern long logBytesCum;
extern int32_t logPtr, logLen, logSize;

#define logIndex(i) ((logPtr + logSize + (i)) % logSize)

bool logReady(bool verbose);
bool logReady(void);
bool logInit(uint32_t);
uint16_t logRead(int32_t index);
void logClear();
void logInit();
void logTestSet(uint16_t);
void logDumpBinary(void);

void logGeneric(int ch, float value);
void logMark();

void logEnable();
void logDisable();

void logSave(void (*logStartCB)());
  
#endif
