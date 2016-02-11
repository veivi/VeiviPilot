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
  consoleNoteLn_P(PSTR("  Stabilizer"));
  consoleNote_P(PSTR("    Ku = "));
  consolePrint(paramRecord.s_Ku, 4);
  consolePrint_P(PSTR(" Tu = "));
  consolePrintLn(paramRecord.s_Tu, 4);
  consoleNoteLn_P(PSTR("  Auto rudder"));
  consoleNote_P(PSTR("    PID Ku = "));
  consolePrint(paramRecord.r_Ku, 4);
  consolePrint_P(PSTR(" Tu = "));
  consolePrintLn(paramRecord.i_Tu, 4);
  consoleNote_P(PSTR("    Yaw damper P = "));
  consolePrintLn(paramRecord.yd_P, 4);
  consoleNote_P(PSTR("  Alpha min = "));
  consolePrint(paramRecord.alphaMin*360);
  consolePrint_P(PSTR("  max = "));
  consolePrintLn(paramRecord.alphaMax*360);
  consoleNoteLn_P(PSTR("  Elevator"));
  consoleNote_P(PSTR("    deflection = "));
  consolePrint(paramRecord.elevDefl*90);
  consolePrint_P(PSTR(" neutral = "));
  consolePrintLn(paramRecord.elevNeutral*90);
  consoleNoteLn_P(PSTR("  Aileron"));
  consoleNote_P(PSTR("    deflection = "));
  consolePrint(paramRecord.aileDefl*90);
  consolePrint_P(PSTR(" neutral = "));
  consolePrintLn(paramRecord.aileNeutral*90);
  consoleNoteLn_P(PSTR("  Rudder"));
  consoleNote_P(PSTR("    deflection = "));
  consolePrint(paramRecord.rudderDefl*90);
  consolePrint_P(PSTR(" neutral = "));
  consolePrintLn(paramRecord.rudderNeutral*90);
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

#define MAX_PARAMS 4

typedef enum { e_int8, e_uint16, e_angle90, e_angle360, e_float, e_string } entry_t;

struct BackupEntry {
  const char *name;
  entry_t type;
  int numParams;
  void *param[MAX_PARAMS];
};

const struct BackupEntry entry[] PROGMEM = {
  { "model", e_uint16, 1, &stateRecord.model },
  { "name", e_string, 1, &paramRecord.name },
  { "min", e_angle360, 1, &paramRecord.alphaMin },
  { "max", e_angle360, 1, &paramRecord.alphaMax },
  { "5048b_ref", e_uint16, 1, &paramRecord.alphaRef },
  { "inner_pid_zn", e_float, 2, &paramRecord.i_Ku, &paramRecord.i_Tu },
  { "outer_p", e_float, 1, &paramRecord.o_P },
  { "stabilizer_pid_zn", e_float, 2, &paramRecord.s_Ku, &paramRecord.s_Tu },
  { "rudder_pid_zn", e_float, 2, &paramRecord.r_Ku, &paramRecord.r_Tu },
  { "yd_p", e_float, 1, &paramRecord.yd_P },
  { "edefl", e_angle90, 1, &paramRecord.elevDefl },
  { "eneutral", e_angle90, 1, &paramRecord.elevNeutral },
  { "ezero", e_angle90, 1, &paramRecord.elevZero },
  { "eservo", e_int8, 1, &paramRecord.servoElev },
  { "adefl", e_angle90, 1, &paramRecord.aileDefl },
  { "aneutral", e_angle90, 1, &paramRecord.aileNeutral },
  { "azero", e_angle90, 1, &paramRecord.aileZero },
  { "aservo", e_int8, 1, &paramRecord.servoAile },
  { "rdefl", e_angle90, 1, &paramRecord.rudderDefl },
  { "rneutral", e_angle90, 1, &paramRecord.rudderNeutral },
  { "rzero", e_angle90, 1, &paramRecord.rudderZero },
  { "rservo", e_int8, 1, &paramRecord.servoRudder },
  { "fstep", e_angle90, 1, &paramRecord.flapStep },
  { "fneutral", e_angle90, 2, &paramRecord.flapNeutral, &paramRecord.flap2Neutral },
  { "fservo", e_int8, 2, &paramRecord.servoFlap, &paramRecord.servoFlap2 },
  { "bdefl", e_angle90, 1, &paramRecord.brakeDefl },
  { "bneutral", e_angle90, 1, &paramRecord.brakeNeutral },
  { "bservo", e_int8, 1, &paramRecord.servoBrake },
  { "gservo", e_int8, 1, &paramRecord.servoGear }
};

static void dumpParamEntry(const struct BackupEntry *e)
{
  consolePrint(e->name);

  for(int i = 0; i < e->numParams; i++) {
    consolePrint(" ");
    switch(e->type) {
    case e_string:
      consolePrint((const char*) e->param[i]);
      break;
      
    case e_uint16:
      consolePrint(*((uint16_t*) e->param[i]));
      break;
      
    case e_int8:
      consolePrint(*((int8_t*) e->param[i]));
      break;
      
    case e_float:
      consolePrint(*((float*) e->param[i]), 3);
      break;

    case e_angle90:
      consolePrint(*((float*) e->param[i])*90);
      break;

    case e_angle360:
      consolePrint(*((float*) e->param[i])*360);
      break;
}
  }

  consolePrintLn("");
} 

void dumpParams()
{
  consoleNoteLn_P(PSTR("Param backup"));
  consoleNoteLn("");
  consoleNote_P(PSTR("MODEL "));
  consolePrint(stateRecord.model);
  consolePrint_P(PSTR(" "));
  consolePrintLn(paramRecord.name);
  consoleNoteLn("");
  consolePrintLn("");

  for(int i = 0; i < sizeof(entry)/sizeof(struct BackupEntry); i++) {
    struct BackupEntry buf;
    memcpy_P(&buf, &entry[i], sizeof(buf));
    dumpParamEntry(&buf);    
  }

  consolePrintLn_P(PSTR("store"));
  consolePrintLn("");
}
