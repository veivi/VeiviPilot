#include <stdint.h>
#include "Filter.h"
#include "Console.h"

class Controller {
public:
  Controller();
  void setPID(float P, float I, float D);
  float getP(void);
  float getI(void);
  float getD(void);
  float getKu(void);
  float getTu(void);
  void setZieglerNicholsPID(float Ku, float Tu);
  void setZieglerNicholsPI(float Ku, float Tu);
  void getZieglerNicholsPID(float *Ku, float *Tu);
  void reset(float value, float err);
  void input(float err, float d);
  void limit(float, float);

  float output(void);
private:
  float I, D, prevD, delta, prevErr, Kp, Ki, Kd;
  float rangeMin, rangeMax;
};

float znGain(float kP, float kI, float kD);
