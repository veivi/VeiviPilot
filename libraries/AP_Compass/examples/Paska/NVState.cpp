#include <string.h>
#include "NVState.h"
#include "Console.h"
#include "Storage.h"
#include "Datagram.h"
#include "Command.h"

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
  "Unnamed",
  12,
  0x40, 0x50, 
  0,
  0, 0, 0,
  0, -45.0/90,
  0, 45.0/90,
  0, -15.0/90, -15.0/90,
  0, 45.0/90,
  0, 45.0/90,
  0, 1, 2, -1, -1, -1, -1,
  -3.0/360,  12.0/360,
  1.0, 0.25, 10.0, 
  1.3, 0.25, 
  2.0, 1.0, 0.1,
  0.1, 0.25,
  0.0, 0.0,
  0.0,
  true,
  12, 25 };

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

  printParams();
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

void printParams()
{
  consoleNote_P(PSTR("  NAME \""));
  consolePrint(paramRecord.name);
  consolePrintLn_P(PSTR("\""));
  
  consoleNote_P(PSTR("  AS5048B ref = "));
  consolePrintLn(paramRecord.alphaRef);
  consoleNoteLn_P(PSTR("  Autostick/pusher"));
  consoleNote_P(PSTR("    Inner Ku = "));
  consolePrint(paramRecord.i_Ku, 4);
  consolePrint_P(PSTR(" Tu = "));
  consolePrint(paramRecord.i_Tu, 4);
  consolePrint_P(PSTR(" Outer P = "));
  consolePrintLn(paramRecord.o_P, 4);
  consoleNoteLn_P(PSTR("  AutoAlpha feedforward Ax+B"));
  consoleNote_P(PSTR("    A = "));
  consolePrint(paramRecord.ff_A, 4);
  consolePrint_P(PSTR(" B = "));
  consolePrintLn(paramRecord.ff_B, 4);
  consoleNoteLn_P(PSTR("  Stabilizer"));
  consoleNote_P(PSTR("    Ku = "));
  consolePrint(paramRecord.s_Ku, 4);
  consolePrint_P(PSTR(" Tu = "));
  consolePrintLn(paramRecord.s_Tu, 4);
  consoleNote_P(PSTR("    Weak leveling limit angle = "));
  consolePrintLn(paramRecord.wl_Limit, 4);
  consoleNote_P(PSTR("    Using "));
  consolePrint_P(paramRecord.c_PID ? PSTR("PID") : PSTR("PI"));
  consolePrintLn_P(PSTR(" controller"));
  consoleNoteLn_P(PSTR("  Auto rudder"));
  consoleNote_P(PSTR("    Ku = "));
  consolePrint(paramRecord.r_Ku, 4);
  consolePrint_P(PSTR(" Tu = "));
  consolePrintLn(paramRecord.r_Tu, 4);
  consoleNote_P(PSTR("  Yaw damper P = "));
  consolePrint(paramRecord.yd_P, 4);
  consolePrint_P(PSTR(" Tau = "));
  consolePrintLn(paramRecord.yd_Tau, 4);
  consoleNote_P(PSTR("  Alpha min = "));
  consolePrint(paramRecord.alphaMin*360);
  consolePrint_P(PSTR("  max = "));
  consolePrintLn(paramRecord.alphaMax*360);
  consoleNote_P(PSTR("  IAS low = "));
  consolePrint(paramRecord.ias_Low);
  consolePrint_P(PSTR("  high = "));
  consolePrintLn(paramRecord.ias_High);
  consoleNoteLn_P(PSTR("  Elevator"));
  consoleNote_P(PSTR("    deflection = "));
  consolePrint(paramRecord.elevDefl*90);
  consolePrint_P(PSTR(" neutral = "));
  consolePrint(paramRecord.elevNeutral*90);
  consolePrint_P(PSTR(" center = "));
  consolePrintLn(paramRecord.elevZero*90);
  consoleNoteLn_P(PSTR("  Aileron"));
  consoleNote_P(PSTR("    deflection = "));
  consolePrint(paramRecord.aileDefl*90);
  consolePrint_P(PSTR(" neutral = "));
  consolePrint(paramRecord.aileNeutral*90);
  consolePrint_P(PSTR(" center = "));
  consolePrintLn(paramRecord.aileZero*90);
  consoleNoteLn_P(PSTR("  Rudder"));
  consoleNote_P(PSTR("    deflection = "));
  consolePrint(paramRecord.rudderDefl*90);
  consolePrint_P(PSTR(" neutral = "));
  consolePrint(paramRecord.rudderNeutral*90);
  consolePrint_P(PSTR(" aile mix = "));
  consolePrint(paramRecord.r_Mix);
  consolePrint_P(PSTR(" center = "));
  consolePrintLn(paramRecord.rudderZero*90);
  consoleNoteLn_P(PSTR("  Flap"));
  consoleNote_P(PSTR("    step = "));
  consolePrint(paramRecord.flapStep*90);
  consolePrint_P(PSTR(" neutral = "));
  consolePrint(paramRecord.flapNeutral*90);
  consolePrint_P(PSTR(" ("));
  consolePrint(paramRecord.flap2Neutral*90);
  consolePrintLn_P(PSTR(")"));
  consoleNoteLn_P(PSTR("  Servo channels"));
  consoleNote_P(PSTR("    A = "));
  consolePrint(paramRecord.servoAile);
  consolePrint_P(PSTR("  E = "));
  consolePrint(paramRecord.servoElev);
  consolePrint_P(PSTR("  R = "));
  consolePrint(paramRecord.servoRudder);
  consolePrint_P(PSTR("  F = ("));
  consolePrint(paramRecord.servoFlap);
  consolePrint_P(PSTR(", "));
  consolePrint(paramRecord.servoFlap2);
  consolePrint_P(PSTR(")  G = "));
  consolePrint(paramRecord.servoGear);
  consolePrint_P(PSTR("  B = "));
  consolePrintLn(paramRecord.servoBrake);
}

static void dumpParamEntry(const Command *e)
{
  consolePrint(e->name);

  for(int i = 0; e->var[i]; i++) {
    consolePrint(" ");
    switch(e->varType) {
    case e_string:
      consolePrint((const char*) e->var[i]);
      break;
      
    case e_uint16:
      consolePrint(*((uint16_t*) e->var[i]));
      break;
      
    case e_int8:
      consolePrint(*((int8_t*) e->var[i]));
      break;
      
    case e_float:
      consolePrint(*((float*) e->var[i]), 3);
      break;

    case e_angle90:
      consolePrint(*((float*) e->var[i])*90);
      break;

    case e_angle360:
      consolePrint(*((float*) e->var[i])*360);
      break;
    }
  }

  consolePrintLn("");
} 

void dumpParams()
{
  datagramTxStart(DG_PARAMS);
  datagramTxOut((const uint8_t*) paramRecord.name, strlen(paramRecord.name));
  datagramTxEnd();
  
  consoleNoteLn_P(PSTR("Param backup"));
  consoleNoteLn("");
  consoleNote_P(PSTR("MODEL "));
  consolePrint(stateRecord.model);
  consolePrint_P(PSTR(" "));
  consolePrintLn(paramRecord.name);
  consoleNoteLn("");
  consolePrintLn("");

  consolePrint("model ");
  consolePrintLn(stateRecord.model);

  int i = 0;
  
  while(1) {
    struct Command cache;
    memcpy_P(&cache, &commands[i++], sizeof(cache));
    if(cache.token == c_invalid)
      break;
    if(cache.var[0])
      dumpParamEntry(&cache);    
  }

  consolePrintLn_P(PSTR("store"));

  datagramTxStart(DG_PARAMS);
  datagramTxEnd();  
}
