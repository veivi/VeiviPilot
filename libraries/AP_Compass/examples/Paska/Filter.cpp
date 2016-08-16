#include "Filter.h"
#include "Console.h"
#include <math.h>

uint8_t population(uint16_t a)
{
  uint8_t v = 0;
  
  for(uint8_t i = 0; i < 16; i++)
    if(a & (1U<<i))
      v++;
  
  return v;
}

float sign(float x)
{
  return x < 0.0 ? -1.0 : 1.0;
}

float clamp(float value, float a, float b)
{
  if(a > b) {
    // Swap limits
    float t = a;
    a = b;
    b = t;
  }

  if(value > a && value < b)
    return value;  
  else if(value <= a)
    return a;
  else if(value >= b)
    return b;

  // All comparisons failed, must be NaN or some such
  
  return 0.0;
}

float mixValue(float mixRatio, float a, float b)
{
  if(mixRatio < 0)
    return a;
  else if(mixRatio > 1)
    return b;
  else
    return (1.0 - mixRatio)*a + mixRatio*b;
}

void Median3Filter::input(float v)
{
  memory[ptr++] = v;
  if(ptr > MedianWindow_c-1) ptr = 0;
}

float Median3Filter::output(void)
{
  return max(min(memory[0],memory[1]), min(max(memory[0],memory[1]),memory[2]));
}

Damper::Damper(void)
{
  avg = 0;
  setTau(1);
}

Damper::Damper(float t)
{
  avg = 0;
  setTau(t);
}

void Damper::input(float v)
{
  avg = mixValue(tau, avg, v);
}

void Damper::reset(float v)
{
  avg = v;
}

float Damper::output(void)
{
  return avg;
}

void Damper::setTau(float tauValue)
{
  tau = 1.0/(1+tauValue);
}

void Derivator::input(float v, float dt)
{
  prev = value;
  value = v;
  delta = dt;
}

float Derivator::output(void)
{
  return (value - prev) / delta;
}

void RateLimiter::setRate(float r)
{
  maxRate = r;
}  

void RateLimiter::reset(float v)
{
  state = v;
}  

float RateLimiter::input(float v, float dt)
{
  state += clamp(v - state, -maxRate*dt, maxRate*dt);
  return output();
}

float RateLimiter::output(void)
{
  return state;
}

void RunningAvgFilter::setWindow(int a) 
{
  if(a < 1 || a > windowLenMax)
     a = windowLenMax;

   sum = 0.0;
   windowLen = a;

   for(int i = 0; i < windowLen; i++)
     memory[i] = 0.0;
}

float RunningAvgFilter::output() 
{ 
  return (float) sum / windowLen;
}

float RunningAvgFilter::input(float v) 
{ 
    ptr = (ptr + 1) % windowLen;

    sum -= memory[ptr];
    sum += memory[ptr] = v;
    
    return output();
}
  
void DelayLine::setDelay(int a) 
{
  if(a < 1 || a > windowLenMax)
     a = windowLenMax;

   delay = a;

   for(int i = 0; i < windowLenMax; i++)
     memory[i] = 0.0;
}

float DelayLine::output() 
{ 
  return memory[(ptr+windowLenMax-delay)%windowLenMax];
}

float DelayLine::input(float v) 
{
  ptr = (ptr + 1) % windowLenMax;
  memory[ptr] = v;
    
  return output();
}
  
float AlphaBuffer::output(void) { 
  if(length > 0) {
    value = sum / length;
    sum = 0.0;
    length = 0;
  } else if(!warn) {
    consoleNoteLn_P(PSTR("Alpha/IAS buffer starved"));
    warn = true;
  }  
  return value;
}

void AlphaBuffer::input(float v) { 
  length++;
  sum += v;
}
