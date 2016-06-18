#ifndef NVSTATE_H
#define NVSTATE_H

#include <stdint.h>

#define MAX_MODELS   4
#define NAME_LEN     8
#define MAX_CH       8

// Parameters and non-volatile state

struct ParamRecord {
  uint16_t crc;
  char name[NAME_LEN+1];
  uint8_t i2c_clkDiv;
  uint8_t i2c_5048B, i2c_24L256;
  uint16_t alphaRef;
  float aileNeutral, aileDefl;
  float elevNeutral, elevDefl;
  float flapNeutral, flap2Neutral, flapStep;
  float rudderNeutral, rudderDefl;
  float brakeNeutral, brakeDefl;
  int8_t servoAile, servoElev, servoRudder, servoFlap, servoFlap2, servoGear, servoBrake;
  float alphaMin, alphaMax;
  float i_KuDp, i_Tu, o_P_per_IAS;
  float s_KuDp, s_Tu;
  float r_Mix, r_Ku, r_Tu;
  float ff_A, ff_B;
  float wl_Limit;
  bool c_PID;
  float ias_Low, ias_High;
  float servoRate;
  };

struct NVStateRecord {
  uint16_t crc;
  uint16_t paramPartition, logPartition;
  uint16_t logStamp;
  int model;
  bool logRPM;
  int testChannel;
  int32_t rxCenter[MAX_CH], rxMin[MAX_CH], rxMax[MAX_CH];
};

extern struct ParamRecord paramRecord;
extern struct NVStateRecord stateRecord;

void defaultParams(void);
void setModel(int model);
void storeParams(void);
void readNVState(void);
void storeNVState(void);
void printParams(void);
void dumpParams(void);

#endif
