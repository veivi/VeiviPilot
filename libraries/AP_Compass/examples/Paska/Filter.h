#ifndef FILTER_H
#define FILTER_H

#include <stdint.h>
#include <stdlib.h>
#include <AP_Math/AP_Math.h>

// #define max(a,b) ((a) > (b) ? (a) : (b))
// #define min(a,b) ((a) < (b) ? (a) : (b))
// #define abs(a) ((a) < 0 ? -(a) : (a))

const int windowLenMax = 1<<5;

float sign(float x);
float clamp(float value, float a, float b);
float mixValue(float mixRatio, float a, float b);
uint8_t population(uint16_t a);
float randomNum(float small, float large);
float quantize(float value, float *state, int numSteps);

class RunningAvgFilter {
 public:
  RunningAvgFilter(int w);
  RunningAvgFilter(void);
  void setWindow(int w);
  float input(float v);
  float output();
    
 private:
  float memory[windowLenMax], sum;
  int windowLen;
  int ptr;
};

class DelayLine {
  public:
    void setDelay(int l);
    float input(float v);
    float output();
    
  private:
    float memory[windowLenMax];
    int delay;
    int ptr;
};

class Damper {
 public:
  Damper(void);
  Damper(float tau);
  Damper(float tau, float initState);
  
  void reset(float v);
  void setTau(float tau);
  float input(float v);
  float output();
    
 private:
  float tau, avg;
};

class Derivator {
  public:
  void input(float v, float dt);
  float output();
    
  private:
  float value, prev, delta;
};

class RateLimiter {
  public:
  float input(float v, float dt);
  float output();
  void setRate(float v);
  void reset(float v);
    
  private:
  float maxRate;
  float state;
};

const int MedianWindow_c = 3;
  
class Median3Filter {  
  public:
    void input(float v);
    float output();
    
  private:
    float memory[MedianWindow_c];
    int ptr;
};

class AlphaBuffer {
public:
  float output(void);
  void input(float v);
  bool warn;
  
private:
  float sum, value;
  int length;
};

#endif
