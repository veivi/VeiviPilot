#ifndef NVSTATE_H
#define NVSTATE_H

#include <stdint.h>
#include <AP_ProgMem/AP_ProgMem.h>

//
// Threshold speed margin (IAS)
//

const float thresholdMargin_c = 15/100.0;

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
  float cL_A, alphaMax;
  float i_Ku_C, i_Tu, o_P;
  float s_Ku_C, s_Tu;
  float r_Mix;
  float p_Ku_C, p_Tu;
  float ff_A, ff_B;
  float maxPitch;
  float cL_max;
  float roll_C;
  float cL_B;
  float servoRate;
  float takeoffTrim;
  };

struct DerivedParams {
  float stallIAS, zeroLiftAlpha;
  float thresholdAlpha, shakerAlpha;
};

struct NVStateRecord {
  uint16_t crc;
  uint16_t paramPartition, dataPartition, logPartition;
  uint16_t logStamp;
  uint16_t model;
  uint16_t testNum;
  int32_t rxCenter[MAX_CH], rxMin[MAX_CH], rxMax[MAX_CH];
};

extern struct ParamRecord vpParam;
extern struct NVStateRecord nvState;
extern struct DerivedParams vpDerived;
extern const prog_char_t *updateDescription;

void defaultParams(void);
bool setModel(int model);
void storeParams(void);
void readNVState(void);
void storeNVState(void);
void printParams(void);
void deriveParams();
void backupParams(void);
int maxModels(void);
void readData(uint8_t *data, int size);
void storeData(const uint8_t *data, int size);
  
float elevFromAlpha(float x);
float alphaFromElev(float x);

#endif
