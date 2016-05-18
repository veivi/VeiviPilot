#ifndef PPM_H
#define PPM_H

#include "RxInput.h"

//

extern uint8_t ppmNumChannels;
extern uint32_t ppmFrames;
extern bool ppmWarnSlow, ppmWarnShort;

void ppmInputInit(struct RxInputRecord *inputs[], int num);

void calibStart();
void calibStop();
  
#endif

