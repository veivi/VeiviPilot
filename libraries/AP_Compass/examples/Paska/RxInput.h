#ifndef RXINPUT_H
#define RXINPUT_H

#include "InputOutput.h"

struct RxInputRecord {
  struct PinDescriptor pin;
  bool freqOnly, alive;
  uint32_t pulseStart;
  uint32_t pulseCount;
  uint32_t pulseWidthAcc;
};
  
void rxInputInit(const struct RxInputRecord *inputs);
bool inputValid(struct RxInputRecord *record);
float inputValue(struct RxInputRecord *record);
float decodePWM(float pulse);

#endif

