#ifndef RXINPUT_H
#define RXINPUT_H

#include <math.h>
#include "InputOutput.h"

struct RxInputRecord {
  struct PinDescriptor pin;
  bool freqOnly, alive;
  int32_t pulseMin, pulseMax, pulseCenter;
  int32_t pulseStart;
  int32_t pulseCount;
  int32_t pulseWidthAcc;
  int32_t pulseWidthLast;
};
  
struct SwitchRecord {
  struct RxInputRecord *input;
  int8_t state;
  float prevValue;
};

void rxInputInit(struct RxInputRecord *record);
bool inputValid(struct RxInputRecord *record);
float inputValue(struct RxInputRecord *record);
int8_t readSwitch(struct SwitchRecord *record);

#define NULLZONE 0.05

float applyNullZone(float value, bool *pilotInput);
float applyNullZone(float value);

extern bool pciWarn;

// pin change int callback

extern "C" void rxInterrupt_callback(uint8_t num);

#endif

