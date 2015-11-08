#ifndef RXINPUT_H
#define RXINPUT_H

#include <stdint.h>

//

struct RxInputRecord {
  uint8_t pin, index;
  bool freqOnly, alive;
  uint32_t pulseStart;
  uint32_t pulseCount;
  uint32_t pulseWidthAcc;
};
  
bool inputValid(struct RxInputRecord *record);
float inputValue(struct RxInputRecord *record);
float decodePWM(float pulse);

#endif

