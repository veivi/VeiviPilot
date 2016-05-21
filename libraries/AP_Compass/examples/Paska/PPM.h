#ifndef PPM_H
#define PPM_H

#include "RxInput.h"

//

extern uint8_t ppmNumChannels;
extern uint32_t ppmFrames;
extern bool ppmWarnSlow, ppmWarnShort;

void ppmInputInit(struct RxInputRecord *inputs[], int num, const int32_t *min, const int32_t *center, const int32_t *max);

void calibStart();
void calibStop(int32_t *min, int32_t *center, int32_t *max);
  
#endif

