#ifndef BUTTON_H
#define BUTTON_H

#include <stdint.h>

class Button {
public:
  Button(float activeValue);
  void reset();
  void input(float);
  bool singlePulse();
  bool doublePulse();
  bool depressed();
  bool state();
  bool lazy();

private:
  float activeValue;
  bool statePrev, stateLazy;
  bool pulseArmed, inputState, inertiaState;
  uint32_t transition;
  uint8_t count;
  bool pulseDouble, pulseSingle, buttonPress;
};

#endif
