#include "Controller.h"

void Controller::setPID(float P, float I, float D) {
    Kp = P; 
    Ki = I;
    Kd = D;
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
  Ki = 2.0*Kp/Tu; 
  Kd = Kp*Tu/8.0;
}

void Controller::setZieglerNicholsPI(float Ku, float Tu) {
  Kp = 0.45*Ku; 
  Ki = 1.2*Kp/Tu; 
  Kd = 0;
}

void Controller::getZieglerNicholsPID(float *Ku, float *Tu) {
  if(Kd != 0.0) {
    *Ku = Kp / 0.6; 
    *Tu = Kd/Kp*8.0;
  } else {
    *Ku = Kp/0.45; 
    *Tu = 1.2*Kp/Ki; 
  }
}

void Controller::reset(float value, float err) {
  prevErr = err;
  I = value - Kp*err;
}

void Controller::input(float err, float d) {
  if(d < 0.001) {
    warn = true;
    consolePrintLn("Controller delta less than 1 ms, iteration ignored");
    return;
  }
    
  errorFilter.input(err - prevErr);
  prevErr = err;
  delta = d;
  
  // float range = 1.0 - Kp*err;
  // I = clamp(I + Ki*err*delta, -range, range);
  // I = clamp(I + Ki*err*delta, -1.0, 1.0);
    
  if(abs(Ki) > 0.1)
    I = clamp(I + Ki*err*delta, -1.0 - Kp*err, 1.0 - Kp*err);
  else
    I = 0.0;
}

float Controller::output(void) {
  float diffTerm = 0.0, diffLimit = 0.15;
  if(Kd != 0.0)
    diffTerm = clamp(Kd*errorFilter.output()/delta, -diffLimit, diffLimit);
  return clamp(Kp*prevErr + I + diffTerm, -1, 1);
}
