#include <stdint.h>
#include "Filter.h"
#include "Console.h"

class Controller {
public:
  Controller();
  void setPID(float P, float I, float D);
  void setZieglerNicholsPID(float Ku, float Tu);
  void setZieglerNicholsPI(float Ku, float Tu);
  void reset(float value, float err);
  void input(float err, float d);
  void limit(float, float);

  float output(void);
protected:
  float I, D, prevD, delta, prevErr, Kp, Ki, Kd;
  float rangeMin, rangeMax;
};

class UnbiasedController : public Controller {
 public:
  void input(float err, float d);
};

const float gainTweak_c = 0.80;
