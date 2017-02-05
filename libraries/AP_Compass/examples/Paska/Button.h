#ifndef BUTTON_H
#define BUTTON_H

#include <stdint.h>

#define INERTIA 2

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
  uint8_t inertiaCount;
  bool inputState;
  bool statePrev, stateLazy, pulseArmed;
  uint32_t transition;
  uint8_t count;
  bool pulseDouble, pulseSingle, buttonPress;
};

#endif
