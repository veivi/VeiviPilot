#include "Controller.h"

Controller::Controller() {
  rangeMin = -1;
  rangeMax = 1;
}

void Controller::setPID(float kP, float kI, float kD) {
    Kp = kP; 
    Ki = kI;
    Kd = kD;
  }

float Controller::getP(void)
{
  return Kp;
}

float Controller::getI(void)
{
  return Ki;
}

float Controller::getD(void)
{
  return Kd;
}

void Controller::setZieglerNicholsPID(float Ku, float Tu) {
  Kp = 0.6*Ku; 
  Ki = 2*Kp/Tu; 
  Kd = Kp*Tu/8;
}

void Controller::setZieglerNicholsPI(float Ku, float Tu)
{
  Kp = 0.45*Ku; 
  Ki = 1.2*Kp/Tu; 
  Kd = 0;
}

float Controller::getKu(void)
{
  float k = 0;
  
  if(Kd != 0.0)
    k = Kp/0.6; 
  else
    k = Kp/0.45;

  return k;
}

float Controller::getTu(void)
{
  if(Kd != 0.0)
    return 2*Kp/Ki;
  else
    return 1.2*Kp/Ki; 
}

void Controller::getZieglerNicholsPID(float *Ku, float *Tu)
{
  if(Ku)
    *Ku = getKu();
  if(Tu)
    *Tu = getTu();
}

float znGain(float kP, float kI, float kD)
{
  if(kD != 0.0)
    return kP/0.6; 
  else
    return kP/0.45; 
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

float Controller::output(void) {
  const float diffLimit = 0.5,
    diffTerm = clamp(Kd*(D+prevD)/2/delta, -diffLimit, diffLimit);
  return clamp(Kp*prevErr + I + diffTerm, rangeMin, rangeMax);
}
