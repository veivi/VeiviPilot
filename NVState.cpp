#include <string.h>
#include "NVState.h"
#include "Console.h"
#include "Storage.h"
#include "Command.h"
#include "Status.h"
#include "Math.h"

extern "C" {
#include "CRC16.h"
#include "Datagram.h"
}

// NV store layout

struct NVStateRecord nvState;
struct ParamRecord vpParam;
struct DerivedParams vpDerived;

#define stateOffset 0U
#define paramOffset nvState.paramPartition
#define dataOffset nvState.dataPartition

const struct ParamRecord paramDefaults = {
  .crc = 0,
  { .name = "Invalid" },
  .i2c_clkDiv = 12,
  .i2c_5048B = 0x40, .i2c_24L256 = 0x50, 
  .alphaRef = 0,
  .aileNeutral = 0, .aileDefl = -45.0/90,
  .elevNeutral = 0, .elevDefl = 45.0/90,
  .flapNeutral = 0, .flap2Neutral = -15.0/90, .flapStep = -15.0/90,
  .rudderNeutral = 0, .rudderDefl = 45.0/90,
  .steerNeutral = 0, .steerDefl = 45.0/90,
  .brakeNeutral = 0, .brakeDefl = 45.0/90,
  .servoAile = 0, .servoElev = 1, .servoRudder = 2, .servoFlap = -1, .servoFlap2 = -1, .servoGear = -1, .servoBrake = -1, .servoSteer = -1, .servoThrottle = -1,
  .cL_A = 0.05, .alphaMax = 12.0/RADIAN,
  .i_Ku_C = 100, .i_Tu = 0.25, .o_P = 0.3, 
  .s_Ku_C = 400, .s_Tu = 0.25, 
  .r_Mix = 0.1,
  .p_Ku_C = 100, .p_Tu = 1.0,
  .at_Ku = 1, .at_Tu = 2.5,
  .ff_A = 0.0, .ff_B = 0.0, .ff_C = 0.0,
  .maxPitch = 45/RADIAN,
  .cL_max= 0.25,
  .roll_C = 0.1,
  .cL_B = 0.6,
  .servoRate = 60/0.09,
  .takeoffTrim = 0.25,
  .weight = 0,
  .thrust = 0,
  .thresholdMargin = 0.15,
  .glideSlope = 3.0/RADIAN,
  .offset = -0.4/RADIAN,
  .elevon = 0,
  .veeTail = 0,
  .virtualOnly = true
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

bool setModel(int model, bool verbose)
{
  if(model < 0)
    model = 0;
  else if(model > maxModels() - 1)
    model = maxModels() - 1;
  
  nvState.model = model;
  cacheRead(paramOffset + sizeof(vpParam)*model,
	    (uint8_t*) &vpParam, sizeof(vpParam));

  if(verbose) {
    consoleNote_P(PSTR("MODEL "));
    consolePrintLn(model);
    consoleNote_P(PSTR("  Model record CRC = "));
    consolePrint(vpParam.crc);
  }
  
  bool isGood = true;
  
  if(paramRecordCrc(&vpParam) != vpParam.crc) {
    if(verbose)
      consolePrintLn_P(PSTR(" CORRUPT, using defaults")); 
    vpParam = paramDefaults;
    isGood = false;
  } else if(verbose)
    consolePrintLn_P(PSTR(" OK"));

  if(verbose)
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

void deleteModel(int model)
{
  
  cacheWrite(paramOffset + sizeof(struct ParamRecord)*model,
  	     NULL, sizeof(struct ParamRecord));
  cacheFlush();
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
  deriveParams();
  
  consoleNote_P(PSTR("  NAME \""));
  consolePrint(vpParam.name);
  consolePrintLn_P(PSTR("\""));
  
  consoleNote_P(PSTR("  Weight = "));
  consolePrint(vpParam.weight, 3);
  consolePrint_P(PSTR(" kg  thrust = "));
  consolePrint(vpParam.thrust, 3);
  consolePrint_P(PSTR(" kg ("));
  consolePrint(vpParam.thrust/vpParam.weight*100, 0);
  consolePrintLn_P(PSTR("% of weight)"));
  consoleNote_P(PSTR("  AS5048B ref = "));
  consolePrintLn(vpParam.alphaRef);
  consoleNoteLn_P(PSTR("  Alpha Hold"));
  consoleNote_P(PSTR("    Inner Ku*IAS^1.5 = "));
  consolePrint(vpParam.i_Ku_C, 4);
  consolePrint_P(PSTR(" Tu = "));
  consolePrint(vpParam.i_Tu, 4);
  consolePrint_P(PSTR(" Outer P = "));
  consolePrintLn(vpParam.o_P, 4);
  consoleNoteLn_P(PSTR("  Alpha feedforward A+Bx+Cx^2"));
  consoleNote_P(PSTR("    "));
  consolePrint(vpParam.ff_A, 5);
  consolePrint_P(PSTR(" + "));
  consolePrint(vpParam.ff_B, 5);
  consolePrint_P(PSTR(" x + "));
  consolePrint(vpParam.ff_C, 5);
  consolePrint_P(PSTR(" x^2  (eff alpha range = "));
  consolePrint(elevPredictInverse(-1.0)*RADIAN);
  consolePrint_P(PSTR(" ... "));
  consolePrint(elevPredictInverse(1.0)*RADIAN);
  consolePrintLn_P(PSTR(")"));
  consoleNoteLn_P(PSTR("  Pusher"));
  consoleNote_P(PSTR("    Ku*IAS^0.5 = "));
  consolePrint(vpParam.p_Ku_C, 4);
  consolePrint_P(PSTR(" Tu = "));
  consolePrintLn(vpParam.p_Tu, 4);
  consoleNoteLn_P(PSTR("  Stabilizer"));
  consoleNote_P(PSTR("    Ku*IAS^1.5 = "));
  consolePrint(vpParam.s_Ku_C, 4);
  consolePrint_P(PSTR(" Tu = "));
  consolePrintLn(vpParam.s_Tu, 4);
  consoleNoteLn_P(PSTR("  Autothrottle"));
  consoleNote_P(PSTR("    Ku = "));
  consolePrint(vpParam.at_Ku, 4);
  consolePrint_P(PSTR(" Tu = "));
  consolePrintLn(vpParam.at_Tu, 4);
  consoleNote_P(PSTR("  Climb pitch(max) = "));
  consolePrint(vpParam.maxPitch*RADIAN, 2);
  consolePrint_P(PSTR("  Glide slope = "));
  consolePrintLn(vpParam.glideSlope*RADIAN, 2);
  consoleNote_P(PSTR("  Alpha range = "));
  consolePrint(vpDerived.zeroLiftAlpha*RADIAN);
  consolePrint_P(PSTR(" ... "));
  consolePrint(vpParam.alphaMax*RADIAN);
  consolePrint_P(PSTR(", stall IAS = "));
  consolePrintLn(vpDerived.stallIAS);
  consoleNote_P(PSTR("  Threshold margin(%) = "));
  consolePrintLn(vpParam.thresholdMargin*100);
  consoleNote_P(PSTR("    Derived alpha(threshold, shake, push) = "));
  consolePrint(vpDerived.thresholdAlpha*RADIAN);
  consolePrint_P(PSTR(", "));
  consolePrint(vpDerived.shakerAlpha*RADIAN);
  consolePrint_P(PSTR(", "));
  consolePrintLn(vpDerived.pusherAlpha*RADIAN);
  consoleNoteLn_P(PSTR("  Coeff of lift A + Bx"));
  consoleNote_P(PSTR("    "));
  consolePrint(vpParam.cL_A, 4);
  consolePrint_P(PSTR(" + "));
  consolePrint(vpParam.cL_B, 4);
  consolePrint_P(PSTR(" x  (max = "));
  consolePrint(vpParam.cL_max, 4);
  consolePrintLn(")");
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
  consoleNoteLn_P(PSTR("  Steering"));
  consoleNote_P(PSTR("    deflection = "));
  consolePrint(vpParam.steerDefl*90);
  consolePrint_P(PSTR(" neutral = "));
  consolePrintLn(vpParam.steerNeutral*90);
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
  consolePrint(vpParam.servoBrake);
  consolePrint_P(PSTR("  S = "));
  consolePrintLn(vpParam.servoSteer);
  consoleNote_P(PSTR("  Servo rate = "));
  consolePrintLn(vpParam.servoRate);
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
      consolePrint(*((float*) e->var[i]), 4);
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

void deriveParams()
{
  // Zero lift alpha
  
  vpDerived.zeroLiftAlpha = -vpParam.cL_A/vpParam.cL_B;

  // Stall IAS
  
  vpDerived.stallIAS = dynamicPressureInverse(G/vpParam.cL_max);
  
  //
  // Effective alpha limits
  //
  
  vpDerived.thresholdAlpha =
    coeffOfLiftInverse(vpParam.cL_max/square(1 + vpParam.thresholdMargin));
  vpDerived.shakerAlpha =
    coeffOfLiftInverse(vpParam.cL_max/square(1 + vpParam.thresholdMargin/2));
  vpDerived.pusherAlpha = vpParam.alphaMax/(1 + vpParam.thresholdMargin/4);

  //
  // Feedforward apex
  //

  if(vpParam.ff_C != 0.0) {
    vpDerived.apexAlpha = -vpParam.ff_B / (2*vpParam.ff_C);
    vpDerived.apexElev = vpParam.ff_A + vpParam.ff_B*vpDerived.apexAlpha + vpParam.ff_C*square(vpDerived.apexAlpha);
  } else {
    vpDerived.apexAlpha = vpDerived.apexElev = 0.0;
  }
}

