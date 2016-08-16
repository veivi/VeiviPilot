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

#define INERTIA 3

class ButtonInputChannel {
 public:
  bool active();
  float value();
  void input(float v);

 private:
  float filter[INERTIA], filterOutput;
  uint8_t filterPtr;
  bool stable;
};

void rxInputInit(struct RxInputRecord *record);
bool inputValid(struct RxInputRecord *record);
float inputValue(struct RxInputRecord *record);
int8_t readSwitch(struct SwitchRecord *record);

#define NULLZONE 0.05

float applyNullZone(float value, bool *pilotInput);
float applyNullZone(float value);

// float decodePWM(float pulse);

extern bool pciWarn;

// pin change int callback

extern "C" void rxInterrupt_callback(uint8_t num);

#endif

