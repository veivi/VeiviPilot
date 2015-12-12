#include "NVState.h"
#include "Console.h"
#include "Storage.h"

// NV store layout

struct NVStateRecord stateRecord;
struct ParamRecord paramRecord;

#define stateOffset 0
#define paramOffset (stateRecord.paramPartition+stateRecord.model*sizeof(paramRecord))

const struct ParamRecord paramDefaults = {
  0,
  12,
  0x40, 0x50, 
  0,
  0, 0,
  0, -45.0/90,
  0, 45.0/90,
  0.5, 0.5, -0.3,
  -0.5, -0.5,
  0, 1, 2, 3, -1, -1,
  -3.0/360,  12.0/360,
  0.51, 4.64, 0.014, 10.0, 
  0.48, 4.36, 0.013
};

const struct NVStateRecord stateDefaults = { 0, 128, 1024, 400, 0, false, 0 };

static uint16_t crc16_update(uint16_t crc, uint8_t a)
{
  crc ^= a;
  for (int i = 0; i < 8; ++i) {
    if (crc & 1)
      crc = (crc >> 1) ^ 0xA001U;
    else
      crc = (crc >> 1);
  }
  
  return crc;
}

static uint16_t crc16(uint16_t initial, const uint8_t *data, int len)
{
  uint16_t crc = initial;
  
  for(int i = 0; i < len; i++)
    crc = crc16_update(crc, data[i]);
    
  return crc;
}

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

void setModel(int model)
{
  const int maxModels_c =
    (stateRecord.logPartition - stateRecord.paramPartition) / sizeof(paramRecord);
  
  if(model < 0)
    model = 0;
  else if(model > maxModels_c - 1)
    model = maxModels_c - 1;
  
  stateRecord.model = model;
  cacheRead(paramOffset, (uint8_t*) &paramRecord, sizeof(paramRecord));

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
  cacheWrite(paramOffset, (const uint8_t*) &paramRecord, sizeof(struct ParamRecord));
  cacheFlush();
}

void readNVState(void)
{
  cacheRead(stateOffset, (uint8_t*) &stateRecord, sizeof(stateRecord));
  
  consoleNote_P(PSTR("  State record CRC = "));
  consolePrint(stateRecord.crc);
    
  if(stateRecord.crc != stateRecordCrc(&stateRecord)) {
    consolePrintLn_P(PSTR(" CORRUPT, using defaults")); 
    stateRecord = stateDefaults;
  } else
    consolePrintLn_P(PSTR(" OK"));

}

void storeNVState(void)
{
  stateRecord.crc = stateRecordCrc(&stateRecord);
  cacheWrite(stateOffset, (const uint8_t*) &stateRecord, sizeof(stateRecord));
  cacheFlush();
}

