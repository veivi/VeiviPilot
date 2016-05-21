#ifndef RXINPUT_H
#define RXINPUT_H

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
  
void rxInputInit(struct RxInputRecord *record);
bool inputValid(struct RxInputRecord *record);
float inputValue(struct RxInputRecord *record);
// float decodePWM(float pulse);

extern bool pciWarn;

// pin change int callback

extern "C" void rxInterrupt_callback(uint8_t num);

#endif

