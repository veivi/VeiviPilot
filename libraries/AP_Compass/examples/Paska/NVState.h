#ifndef NVSTATE_H
#define NVSTATE_H

#include <stdint.h>

#define MAX_MODELS   4
#define NAME_LEN     8

// Parameters and non-volatile state

struct ParamRecord {
  uint16_t crc;
  char name[NAME_LEN+1];
  uint8_t i2c_clkDiv;
  uint8_t i2c_5048B, i2c_24L256;
  uint16_t alphaRef;
  float elevZero, aileZero, rudderZero;
  float aileNeutral, aileDefl;
  float elevNeutral, elevDefl;
  float flapNeutral, flap2Neutral, flapStep;
  float rudderNeutral, rudderDefl;
  float brakeNeutral, brakeDefl;
  int8_t servoAile, servoElev, servoRudder, servoFlap, servoFlap2, servoGear, servoBrake;
  float alphaMin, alphaMax;
  float i_Ku, i_Tu, o_P;
  float s_Ku_fast, s_Tu_fast;
  float s_Ku_slow, s_Tu_slow;
  float yd_P, yd_Tau, r_Mix;
  float r_Ku, r_Tu;
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
