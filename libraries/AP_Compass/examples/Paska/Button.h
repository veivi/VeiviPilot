#ifndef BUTTON_H
#define BUTTON_H

#include <stdint.h>

class Button {
public:
  void input(bool);
  bool singlePulse();
  bool doublePulse();
  bool state();
  bool active();

private:
  bool statePrev, stateLazy, stateActive;
  uint32_t pulseStart;
  bool pulseArmed;
  uint8_t count;
  bool pulseDouble, pulseSingle;
};

#endif
