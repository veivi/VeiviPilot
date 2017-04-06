#include "Filter.h"
#include "Console.h"
#include "NVState.h"
#include <math.h>

const float airDensity_c = 1.225;

float dynamicPressure(float ias)
{
    return airDensity_c * square(ias) / 2;
}

float dynamicPressureInverse(float pressure)
{
    return sqrtf(2 * pressure / airDensity_c);
}

float coeffOfLift(float aoa)
{
  const float i = (vpParam.cL_max - vpParam.cL_A)/vpParam.cL_B,
    d = 2/(PI-2)*(vpParam.alphaMax - i),
    w = d + vpParam.alphaMax - i;

  if(i > vpParam.alphaMax || aoa < vpParam.alphaMax - w)
    return MIN(vpParam.cL_A + aoa*vpParam.cL_B, vpParam.cL_max);
  else
    return vpParam.cL_A
      + vpParam.cL_B*(i - d*(1 - sin(PI/2*(aoa - vpParam.alphaMax + w)/w)));
}

float coeffOfLiftInverse(float target)
{
  float left = vpDerived.zeroLiftAlpha, right = vpParam.alphaMax;
  float center, approx;

  int i = 0;
  
  do {
    center = (left+right)/2;
    approx = coeffOfLift(center);
    
    if(approx > target)
      right = center;
    else
      left = center;

    if(i++ > 1<<5) {
      consoleNote_P(PSTR("Inverse cL not defined for "));
      consolePrintLn(target);
      return 0/0.0;
    }
  } while(fabs(approx - target) > 0.0003);

  return center;
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

float randomNum(float small, float large)
{
  return small + (large-small)*(float) ((rand()>>3) & 0xFFF) / 0x1000;
}

uint32_t randomUInt32()
{
  uint32_t buffer = 0;
  for(unsigned int i = 0; i < sizeof(buffer); i++)
    buffer = (buffer<<8) | (uint32_t) randomNum(0, 1<<8);
  return buffer;
}

float quantize(float value, float *state, int numSteps)
{
  if((int) ((value-1.0/numSteps/2)*numSteps) > *state)
    *state = (value-1.0/numSteps/2)*numSteps;
  else if((int) ((value+1.0/numSteps/2)*numSteps) < *state)
    *state = (value+1.0/numSteps/2)*numSteps;

  return (float) *state / numSteps;
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

/*
static int compareFloat(const void *a, const void *b)
{
  if(*(float*)a < *(float*)b)
    return -1;
  else if(*(float*)a > *(float*)b)
    return 1;
  else return 0;    
}
*/

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

Damper::Damper(float t, float i)
{
  avg = i;
  setTau(t);
}

float Damper::input(float v)
{
  avg = mixValue(tau, avg, v);
  return output();
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

RunningAvgFilter::RunningAvgFilter(void)
{
  setWindow(-1);
}

RunningAvgFilter::RunningAvgFilter(int w)
{
  setWindow(w);
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
  if(a < 1 || a > DelayMax)
     a = DelayMax;

   delay = a;

   for(int i = 0; i < DelayMax; i++)
     memory[i] = 0.0;
}

float DelayLine::output() 
{ 
  return memory[(ptr+DelayMax-delay)%DelayMax];
}

float DelayLine::input(float v) 
{
  ptr = (ptr + 1) % DelayMax;
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

int Tabulator::index(float x)
{
  return TabulatorWindow_c * (x - store.domainA) / (store.domainB - store.domainA);
}

const int threshold_c = 3;

void Tabulator::datum(float x, float y)
{
  int i = index(x);
    
  if(i < 0 || i > TabulatorWindow_c-1)
    return;

  store.count[i]++;
  store.sum[i] += y;
  
  if(store.count[i] > threshold_c) {
    const float est = store.sum[i]/store.count[i];
    store.rangeA = MIN(store.rangeA, est);
    store.rangeB = MAX(store.rangeB, est);
    store.var[i] += square(y - est);
  }
}

float Tabulator::estimate(float x)
{
  int i = index(x);

  if(i < 0 || i > TabulatorWindow_c-1 || store.count[i] < threshold_c)
    return 0/0.0;

  return store.sum[i]/store.count[i];
}

void Tabulator::commit()
{
  if(committed)
    storeData((uint8_t*) &storeCommitted, sizeof(storeCommitted));

  storeCommitted = store;
  committed = true;
}

void Tabulator::invalidate()
{
  committed = false;
  readData((uint8_t*) &store, sizeof(store));
}

void Tabulator::clear()
{
  memset(&store.sum, 0, sizeof(store.sum));
  memset(&store.var, 0, sizeof(store.var));
  memset(&store.count, 0, sizeof(store.count));
  store.rangeA = store.rangeB = 0;
  storeData((uint8_t*) &store, sizeof(store));
  committed = false;
}

void Tabulator::setDomain(float a, float b)
{
  store.domainA = a;
  store.domainB = b;
  clear();
}

void Tabulator::report(float t)
{
  const int colLeft = 10, colRight = 78;
  const int col0 = colLeft - store.rangeA*(colRight-colLeft)/(store.rangeB - store.rangeA);
    

  float maxX = 0, maxV = -1e6;
  
  for(int i = 0; i < TabulatorWindow_c; i++) {
    consoleNote("");

    float xI = (store.domainA + (store.domainB-store.domainA)*(0.5+i)/TabulatorWindow_c)*RADIAN;
    consolePrint(xI);
    consoleTab(colLeft);

    if(store.count[i] > 2*threshold_c) {
      float e = store.sum[i]/store.count[i], v = sqrt(store.var[i]/(store.count[i]-threshold_c));
      int xc = colLeft + (colRight-colLeft) * (e - store.rangeA) / (store.rangeB - store.rangeA);
      int xl = colLeft + (colRight-colLeft) * (e - v - store.rangeA) / (store.rangeB - store.rangeA);
      int xr = colLeft + (colRight-colLeft) * (e + v - store.rangeA) / (store.rangeB - store.rangeA);

      if(e > maxV) {
	maxV = e;
	maxX = xI;
      }
      
      for(int x = colLeft; x <= colRight; x++) {
	if(x == xc)
	  consolePrint("*");
	else if(x == col0)
	  consolePrint("|");
	else if(x >= xl && x <= xr)
	  consolePrint("-");
	else
	  consolePrint(" ");
      }
    }

    consolePrintLn("");
  }
  
  consoleNote_P(PSTR("Max value = "));
  consolePrint(maxV, 4);
  consolePrint_P(PSTR(" at "));
  consolePrintLn(maxX, 2);
  
  consolePrintLn("");
  
  consolePrint_P(PSTR("col_max "));
  consolePrint(maxV, 4);
  consolePrint(" ");
  consolePrintLn(maxX/RADIAN, 4);
  
  consolePrintLn("");
  
  consolePrint("x = c(");

  int c = 0;
  
  for(int i = 0; i < TabulatorWindow_c; i++) {
    float v = sqrt(store.var[i]/(store.count[i]-threshold_c));

    if(store.count[i] > 2*threshold_c && v < t) {
      if(c++ > 0)
	consolePrint(", ");
      consolePrint(store.domainA + (store.domainB-store.domainA)*(i + 0.5)/TabulatorWindow_c, 4);
    }
  }
  
  consolePrintLn(")");
    
  consolePrint("y = c(");

  c = 0;

  for(int i = 0; i < TabulatorWindow_c; i++) {
    float e = store.sum[i]/store.count[i], v = sqrt(store.var[i]/(store.count[i]-threshold_c));

    if(store.count[i] > 2*threshold_c && v < t) {
      if(c++ > 0)
	consolePrint(", ");
      
      consolePrint(e, 4);
    }
  }
  
  consolePrintLn(")");  
}
