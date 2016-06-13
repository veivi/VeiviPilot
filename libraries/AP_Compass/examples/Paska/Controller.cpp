#include "Controller.h"

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

void Controller::input(float err, float d) {
  if(d < 1.0e-3) {
    warn = true;
    consoleNoteLn_P(PSTR("Controller delta less than 1 ms, iteration ignored"));
    return;
  }

  D = err - prevErr;
  prevErr = err;
  
  errorFilter.input(D);
  delta = d;
  
  // float range = 1.0 - Kp*err;
  // I = clamp(I + Ki*err*delta, -range, range);
  // I = clamp(I + Ki*err*delta, -1.0, 1.0);
    
  if(abs(Ki) > 0.05)
    I = clamp(I + Ki*err*delta, -1.0 - Kp*err, 1.0 - Kp*err);
  else
    I = 0.0;
}

float Controller::output(void) {
  float diffTerm = 0.0, diffLimit = 0.15;
  if(Kd != 0.0)
    // diffTerm = clamp(Kd*errorFilter.output()/delta, -diffLimit, diffLimit);
    diffTerm = clamp(Kd*D/delta, -diffLimit, diffLimit);
  return clamp(Kp*prevErr + I + diffTerm, -1, 1);
}
