#include "Controller.h"
#include "Math.h"

Controller::Controller() {
  rangeMin = -1;
  rangeMax = 1;
}

void Controller::setPID(float kP, float kI, float kD)
{
  Kp = kP; 
  Ki = kI;
  Kd = kD;
}

void Controller::setZieglerNicholsPID(float Ku, float Tu) {
  Ku *= gainTweak_c;
  Kp = 0.6*Ku; 
  Ki = 2*Kp/Tu; 
  Kd = Kp*Tu/8;
}

void Controller::setZieglerNicholsPI(float Ku, float Tu)
{
  Ku *= gainTweak_c;
  Kp = 0.45*Ku; 
  Ki = 1.2*Kp/Tu; 
  Kd = 0;
}

void Controller::reset(float value, float err) {
  prevErr = err;
  I = value - Kp*err;
}

void Controller::limit(float a, float b) {
  rangeMin = a;
  rangeMax = b;
}

void Controller::input(float err, float d) {
  prevD = d;
  D = err - prevErr;
  prevErr = err;
  
  delta = d;

  I = clamp(I + Ki*err*delta, rangeMin - Kp*err, rangeMax - Kp*err);
}

void UnbiasedController::input(float err, float d) {
  Controller::input(err, d);
  
  if(Ki == 0.0)
    I = 0.0;
}

float Controller::output(void) {
  const float diffLimit = (rangeMax-rangeMin)/6,
    diffTerm = clamp(Kd*(D+prevD)/2/delta, -diffLimit, diffLimit);
  return clamp(Kp*prevErr + I + diffTerm, rangeMin, rangeMax);
}
