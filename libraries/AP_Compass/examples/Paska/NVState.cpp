#include <string.h>
#include "NVState.h"
#include "Console.h"
#include "Storage.h"
#include "Command.h"
#include "Status.h"
#include "Filter.h"

extern "C" {
#include "CRC16.h"
#include "Datagram.h"
}

// NV store layout

struct NVStateRecord nvState;
struct ParamRecord vpParam;

#define stateOffset 0U
#define paramOffset nvState.paramPartition
#define dataOffset nvState.dataPartition

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
  .cL_A = 0.05, .alphaMax = 12.0/RADIAN,
  .i_Ku_C = 100, .i_Tu = 0.25, .o_P = 0.3, 
  .s_Ku_C = 400, .s_Tu = 0.25, 
  .r_Mix = 0.1,
  .p_Ku_C = 100, .p_Tu = 1.0,
  .ff_A = 0.0, .ff_B = 0.0,
  .wl_Limit = 0.0,
  .cL_max= 0.25,
  .roll_C = 0.1,
  .cL_B = 0.6,
  .servoRate = 60/0.09,
  .takeoffTrim = 0.25
};

const struct NVStateRecord stateDefaults = {
  .crc = 0,
  .paramPartition = 128,
  .dataPartition = 2048,
  .logPartition = 4096,
  .logStamp = 400,
  .model = 0 };

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
  vpParam = paramDefaults;
}

void defaultState(void)
{
  nvState = stateDefaults;
}

int maxModels(void)
{
  return (nvState.dataPartition - nvState.paramPartition) / sizeof(vpParam);
}

bool setModel(int model)
{
  if(model < 0)
    model = 0;
  else if(model > maxModels() - 1)
    model = maxModels() - 1;
  
  nvState.model = model;
  cacheRead(paramOffset + sizeof(vpParam)*model,
	    (uint8_t*) &vpParam, sizeof(vpParam));

  consoleNote_P(PSTR("MODEL "));
  consolePrintLn(model);
  consoleNote_P(PSTR("  Model record CRC = "));
  consolePrint(vpParam.crc);

  bool isGood = true;
  
  if(paramRecordCrc(&vpParam) != vpParam.crc) {
    consolePrintLn_P(PSTR(" CORRUPT, using defaults")); 
    vpParam = paramDefaults;
    isGood = false;
  } else
    consolePrintLn_P(PSTR(" OK"));

  printParams();

  return isGood;
}
  
void storeParams(void)
{
  vpParam.crc = paramRecordCrc(&vpParam);
  consoleNote_P(PSTR("Model record CRC = "));
  consolePrintLn(vpParam.crc);
  cacheWrite(paramOffset + sizeof(vpParam)*nvState.model,
  	     (const uint8_t*) &vpParam, sizeof(vpParam));
  cacheFlush();
  consoleNoteLn_P(PSTR("  Stored"));
}

void readData(uint8_t *data, int size)
{
  cacheRead(dataOffset, data, size);
}

void storeData(const uint8_t *data, int size)
{
  cacheWrite(dataOffset, data, size);
  cacheFlush();
}

void readNVState(void)
{
  cacheRead(stateOffset, (uint8_t*) &nvState, sizeof(nvState));
  
  consoleNote_P(PSTR("  State record CRC = "));
  consolePrint(nvState.crc);
    
  if(nvState.crc != stateRecordCrc(&nvState)) {
    consolePrintLn_P(PSTR(" CORRUPT, using defaults")); 
    defaultState();
  } else
    consolePrintLn_P(PSTR(" OK"));
}

void storeNVState(void)
{
  if(sizeof(nvState) < paramOffset - stateOffset) {
    nvState.crc = stateRecordCrc(&nvState);
    cacheWrite(stateOffset, (const uint8_t*) &nvState, sizeof(nvState));
    cacheFlush();
  } else
    consoleNoteLn_P(PSTR("PANIC : State record exceeds partition size"));
}

void printParams()
{
  consoleNote_P(PSTR("  NAME \""));
  consolePrint(vpParam.name);
  consolePrintLn_P(PSTR("\""));
  
  consoleNote_P(PSTR("  AS5048B ref = "));
  consolePrintLn(vpParam.alphaRef);
  consoleNoteLn_P(PSTR("  Alpha Hold"));
  consoleNote_P(PSTR("    Inner Ku*IAS^1.5 = "));
  consolePrint(vpParam.i_Ku_C, 4);
  consolePrint_P(PSTR(" Tu = "));
  consolePrint(vpParam.i_Tu, 4);
  consolePrint_P(PSTR(" Outer P = "));
  consolePrintLn(vpParam.o_P, 4);
  consoleNoteLn_P(PSTR("  Alpha feedforward A+Bx"));
  consoleNote_P(PSTR("    "));
  consolePrint(vpParam.ff_A, 5);
  consolePrint_P(PSTR(" + "));
  consolePrint(vpParam.ff_B, 5);
  consolePrint_P(PSTR(" x  (alpha range = "));
  consolePrint(alphaFromElev(-1.0)*RADIAN);
  consolePrint_P(PSTR("..."));
  consolePrint(alphaFromElev(1.0)*RADIAN);
  consolePrintLn_P(PSTR(")"));
  consoleNoteLn_P(PSTR("  Pusher"));
  consoleNote_P(PSTR("    Ku*IAS^1.5 = "));
  consolePrint(vpParam.p_Ku_C, 4);
  consolePrint_P(PSTR(" Tu = "));
  consolePrintLn(vpParam.p_Tu, 4);
  consoleNoteLn_P(PSTR("  Stabilizer"));
  consoleNote_P(PSTR("    Ku*IAS^1.5 = "));
  consolePrint(vpParam.s_Ku_C, 4);
  consolePrint_P(PSTR(" Tu = "));
  consolePrintLn(vpParam.s_Tu, 4);
  consoleNote_P(PSTR("    Weak leveling limit angle = "));
  consolePrintLn(vpParam.wl_Limit*RADIAN, 4);
  consoleNote_P(PSTR("  Stall alpha = "));
  consolePrint(vpParam.alphaMax*RADIAN);
  consolePrint_P(PSTR(" IAS = "));
  consolePrintLn(stallIAS());
  consoleNote_P(PSTR(" Coeff of lift (A, B, max) = "));
  consolePrint(vpParam.cL_A);
  consolePrint_P(PSTR(", "));
  consolePrint(vpParam.cL_B);
  consolePrint_P(PSTR(", "));
  consolePrintLn(vpParam.cL_max);
  consoleNote_P(PSTR("  Roll rate K = "));
  consolePrintLn(vpParam.roll_C);
  consoleNoteLn_P(PSTR("  Elevator"));
  consoleNote_P(PSTR("    deflection = "));
  consolePrint(vpParam.elevDefl*90);
  consolePrint_P(PSTR(" neutral = "));
  consolePrint(vpParam.elevNeutral*90);
  consolePrint_P(PSTR(" trim%(takeoff) = "));
  consolePrintLn(vpParam.takeoffTrim*100);
  consoleNoteLn_P(PSTR("  Aileron"));
  consoleNote_P(PSTR("    deflection = "));
  consolePrint(vpParam.aileDefl*90);
  consolePrint_P(PSTR(" neutral = "));
  consolePrintLn(vpParam.aileNeutral*90);
  consoleNoteLn_P(PSTR("  Rudder"));
  consoleNote_P(PSTR("    deflection = "));
  consolePrint(vpParam.rudderDefl*90);
  consolePrint_P(PSTR(" neutral = "));
  consolePrint(vpParam.rudderNeutral*90);
  consolePrint_P(PSTR(" aile mix = "));
  consolePrintLn(vpParam.r_Mix);
  consoleNoteLn_P(PSTR("  Flap"));
  consoleNote_P(PSTR("    step = "));
  consolePrint(vpParam.flapStep*90);
  consolePrint_P(PSTR(" neutral = "));
  consolePrint(vpParam.flapNeutral*90);
  consolePrint_P(PSTR(" ("));
  consolePrint(vpParam.flap2Neutral*90);
  consolePrintLn_P(PSTR(")"));
  consoleNoteLn_P(PSTR("  Servo channels"));
  consoleNote_P(PSTR("    A = "));
  consolePrint(vpParam.servoAile);
  consolePrint_P(PSTR("  E = "));
  consolePrint(vpParam.servoElev);
  consolePrint_P(PSTR("  R = "));
  consolePrint(vpParam.servoRudder);
  consolePrint_P(PSTR("  F = ("));
  consolePrint(vpParam.servoFlap);
  consolePrint_P(PSTR(", "));
  consolePrint(vpParam.servoFlap2);
  consolePrint_P(PSTR(")  G = "));
  consolePrint(vpParam.servoGear);
  consolePrint_P(PSTR("  B = "));
  consolePrintLn(vpParam.servoBrake);
  consoleNote_P(PSTR("  Servo rate = "));
  consolePrintLn(vpParam.servoRate);

  consoleNoteLn_P(PSTR("CoL(norm) curve"));
  
  for(float aR = -0.25; aR <= 1.2; aR += 0.075) {
    consoleNote("");
    consolePrint(vpParam.alphaMax*RADIAN*aR);
    consoleTab(10);

    const int col0 = 20, col1 = 78;
    int x = col0 + (col1-col0) * coeffOfLift(vpParam.alphaMax*aR)/vpParam.cL_max;

    if(x < col0) {
      consoleTab(x);
      consolePrint("*");
      consoleTab(col0);
      consolePrint("|");
      consoleTab(col1);
      consolePrintLn("|");
    } else if(x > col0) {
      consoleTab(col0);
      consolePrint("|");
      consoleTab(x);
      if(x < col1) {
	consolePrint("*");
	consoleTab(col1);
	consolePrintLn("|");
      } else
	consolePrintLn("*");
    } else {
      consoleTab(col0);
      consolePrint("*");
      consoleTab(col1);
      consolePrintLn("|");
    }
  }
}

static void backupParamEntry(const Command *e)
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

    case e_percent:
      consolePrint(*((float*) e->var[i])*100);
      break;

    case e_angle:
      consolePrint(*((float*) e->var[i])*RADIAN);
      break;

    case e_angle90:
      consolePrint(*((float*) e->var[i])*90);
      break;
    }
  }

  consolePrintLn("");
} 

const prog_char_t *updateDescription = NULL;

void backupParams()
{
  datagramTxStart(DG_PARAMS);
  datagramTxOut((const uint8_t*) vpParam.name, strlen(vpParam.name));
  datagramTxEnd();
  
  consoleNoteLn_P(PSTR("Param backup"));
  consoleNoteLn("");
  if(updateDescription) {
    consoleNote("  APPLIED UPDATE : ");
    consolePrintLn_P(updateDescription);
  }
  consoleNote_P(PSTR("MODEL "));
  consolePrint(nvState.model);
  consolePrint_P(PSTR(" "));
  consolePrintLn(vpParam.name);
  consoleNoteLn("");
  consolePrintLn("");

  consolePrint("model ");
  consolePrintLn(nvState.model);

  int i = 0;
  
  while(1) {
    struct Command cache;
    memcpy_P(&cache, &commands[i++], sizeof(cache));
    if(cache.token == c_invalid)
      break;
    if(cache.var[0])
      backupParamEntry(&cache);    
  }

  consolePrintLn_P(PSTR("store"));

  datagramTxStart(DG_PARAMS);
  datagramTxEnd();  
}

float elevFromAlpha(float x)
{
  return vpParam.ff_A + vpParam.ff_B*x;
}

float alphaFromElev(float x)
{
  return (x - vpParam.ff_A)/vpParam.ff_B;
}
