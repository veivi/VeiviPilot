#include "NVState.h"
#include "Console.h"
#include "Storage.h"

extern "C" {
#include "CRC16.h"
}

// NV store layout

struct NVStateRecord stateRecord;
struct ParamRecord paramRecord;

#define stateOffset 0U
#define paramOffset stateRecord.paramPartition

const struct ParamRecord paramDefaults = {
  0,
  12,
  0x40, 0x50, 
  0,
  0, 0, 0,
  0, -45.0/90,
  0, 45.0/90,
  0, -15.0/90, -15.0/90,
  0, 45.0/90,
  0, 45.0/90,
  0, 1, 2, 3, -1, 6, -1,
  -3.0/360,  12.0/360,
  1.0, 0.25, 10.0, 
  1.3, 0.25, 
  2.0, 1.0, 0.33 };

const struct NVStateRecord stateDefaults = { 0, 128, 1024, 400, 0, false, 0 };

static uint16_t crc16OfRecord(uint16_t initial, const uint8_t *record, int size)
{
  return crc16(initial, &record[sizeof(uint16_t)], size - sizeof(uint16_t));
}

static uint16_t paramRecordCrc(struct ParamRecord *record)
{
  return crc16OfRecord(0xFFFF, (uint8_t*) record, sizeof(*record));
}

static uint16_t stateRecordCrc(struct NVStateRecord *record)
{
  return crc16OfRecord(0xFFFF, (uint8_t*) record, sizeof(*record));
}

void defaultParams(void)
{
  paramRecord = paramDefaults;
}

void setModel(int model)
{
  const int maxModels_c =
    (stateRecord.logPartition - stateRecord.paramPartition)
    / sizeof(paramRecord);
  
  if(model < 0)
    model = 0;
  else if(model > maxModels_c - 1)
    model = maxModels_c - 1;
  
  stateRecord.model = model;
  cacheRead(paramOffset + sizeof(paramRecord)*model,
	    (uint8_t*) &paramRecord, sizeof(paramRecord));

  consoleNote_P(PSTR("MODEL "));
  consolePrintLn(model);
  
  consoleNote_P(PSTR("  Model record CRC = "));
  consolePrint(paramRecord.crc);
    
  if(paramRecordCrc(&paramRecord) != paramRecord.crc) {
    consolePrintLn_P(PSTR(" CORRUPT, using defaults")); 
    paramRecord = paramDefaults;
  } else
    consolePrintLn_P(PSTR(" OK"));  
}
  
void storeParams(void)
{
  paramRecord.crc = paramRecordCrc(&paramRecord);
  consoleNote_P(PSTR("Model record CRC = "));
  consolePrintLn(paramRecord.crc);
  cacheWrite(paramOffset + sizeof(paramRecord)*stateRecord.model,
  	     (const uint8_t*) &paramRecord, sizeof(paramRecord));
  cacheFlush();
  consoleNoteLn_P(PSTR("  Stored"));
}

void readNVState(void)
{
  cacheRead(stateOffset, (uint8_t*) &stateRecord, sizeof(stateRecord));
  
  consoleNote_P(PSTR("  State record CRC = "));
  consolePrint(stateRecord.crc);
    
  if(stateRecord.crc != stateRecordCrc(&stateRecord)) {
    consolePrintLn_P(PSTR(" CORRUPT, using defaults")); 
    defaultParams();
  } else
    consolePrintLn_P(PSTR(" OK"));
}

void storeNVState(void)
{
  if(sizeof(stateRecord) < paramOffset - stateOffset) {
    stateRecord.crc = stateRecordCrc(&stateRecord);
    cacheWrite(stateOffset, (const uint8_t*) &stateRecord, sizeof(stateRecord));
    cacheFlush();
  } else
    consoleNoteLn_P(PSTR("PANIC : State record exceeds partition size"));
}

