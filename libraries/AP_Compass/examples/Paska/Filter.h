#ifndef FILTER_H
#define FILTER_H

#include <stdint.h>
#include <stdlib.h>

const int windowLenMax = 1<<5;

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

const int DelayMax = 4;

class DelayLine {
  public:
    void setDelay(int l);
    float input(float v);
    float output();
    
  private:
    float memory[DelayMax];
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
