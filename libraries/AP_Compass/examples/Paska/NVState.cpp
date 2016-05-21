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
  .crc = 0,
  { .name = "Unnamed" },
  .i2c_clkDiv = 12,
  .i2c_5048B = 0x40, .i2c_24L256 = 0x50, 
  .alphaRef = 0,
  .aileNeutral = 0, .aileDefl = -45.0/90,
  .elevNeutral = 0, .elevDefl = 45.0/90,
  .flapNeutral = 0, .flap2Neutral = -15.0/90, .flapStep = -15.0/90,
  .rudderNeutral = 0, .rudderDefl = 45.0/90,
  .brakeNeutral = 0, .brakeDefl = 45.0/90,
  .servoAile = 0, .servoElev = 1, .servoRudder = 2, .servoFlap = -1, .servoFlap2 = -1, .servoGear = -1, .servoBrake = -1,
  .alphaMin = -3.0/360, .alphaMax = 12.0/360,
  .i_Ku = 1.0, .i_Tu = 0.25, .o_P = 10.0, 
  .s_Ku_fast = 1.3, .s_Tu_fast = 0.25, 
  .s_Ku_slow = 1.3, .s_Tu_slow = 0.25, 
  .yd_P = 0, .yd_Tau = 1.0, .r_Mix = 0.1,
  .r_Ku = 0.1, .r_Tu = 0.25,
  .ff_A = 0.0, .ff_B = 0.0,
  .wl_Limit = 0.0,
  .c_PID = true,
  .ias_Low = 12, .ias_High = 25,
  .servoRate = 60/0.09
};

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
  consoleNote_P(PSTR("    (Fast) Ku = "));
  consolePrint(paramRecord.s_Ku_fast, 4);
  consolePrint_P(PSTR(" Tu = "));
  consolePrintLn(paramRecord.s_Tu_fast, 4);
  consoleNote_P(PSTR("    (Slow) Ku = "));
  consolePrint(paramRecord.s_Ku_slow, 4);
  consolePrint_P(PSTR(" Tu = "));
  consolePrintLn(paramRecord.s_Tu_slow, 4);
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
  consolePrintLn(paramRecord.yd_P, 4);
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
  consolePrintLn(paramRecord.elevNeutral*90);
  //  consolePrint_P(PSTR(" center = "));
  //  consolePrintLn(paramRecord.elevZero*90);
  consoleNoteLn_P(PSTR("  Aileron"));
  consoleNote_P(PSTR("    deflection = "));
  consolePrint(paramRecord.aileDefl*90);
  consolePrint_P(PSTR(" neutral = "));
  consolePrintLn(paramRecord.aileNeutral*90);
  //  consolePrint_P(PSTR(" center = "));
  //  consolePrintLn(paramRecord.aileZero*90);
  consoleNoteLn_P(PSTR("  Rudder"));
  consoleNote_P(PSTR("    deflection = "));
  consolePrint(paramRecord.rudderDefl*90);
  consolePrint_P(PSTR(" neutral = "));
  consolePrint(paramRecord.rudderNeutral*90);
  consolePrint_P(PSTR(" aile mix = "));
  consolePrintLn(paramRecord.r_Mix);
  //  consolePrint_P(PSTR(" center = "));
  // consolePrintLn(paramRecord.rudderZero*90);
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
  consoleNote_P(PSTR("  Servo rate = "));
  consolePrintLn(paramRecord.servoRate);
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
