#ifndef NVSTATE_H
#define NVSTATE_H

#include <stdint.h>

#define MAX_MODELS   4

// Parameters and non-volatile state

struct ParamRecord {
  uint16_t crc;
  uint8_t i2c_clkDiv;
  uint8_t i2c_5048B, i2c_24L256;
  uint16_t alphaRef;
  float elevZero, aileZero;
  float aileNeutral, aileDefl;
  float elevNeutral, elevDefl;
  float flapNeutral, flap2Neutral, flapStep;
  float brakeNeutral, brakeDefl;
  int8_t servoAile, servoElev, servoFlap, servoFlap2, servoGear, servoBrake;
  float alphaMin, alphaMax;
  float i_Kp, i_Ki, i_Kd, o_P;
  float s_Kp, s_Ki, s_Kd;
};

struct NVStateRecord {
  uint16_t crc;
  int paramPartition, logPartition;
  int logStamp;
  int model;
  bool logRPM;
  int testChannel;
};

extern struct ParamRecord paramRecord;
extern struct NVStateRecord stateRecord;

void setModel(int model);
void storeParams(void);
void readNVState(void);
void storeNVState(void);

#endif
