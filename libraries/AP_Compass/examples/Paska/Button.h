#ifndef BUTTON_H
#define BUTTON_H

#include <stdint.h>

class Button {
public:
  Button(float activeValue);
  void input(float);
  bool singlePulse();
  bool doublePulse();
  bool state();
  bool active();

private:
  float activeValue;
  bool inertia, filterOutput, statePrev, stateLazy, stateActive;
  uint32_t pulseStart;
  bool pulseArmed;
  uint8_t count;
  bool pulseDouble, pulseSingle;
};

#endif
