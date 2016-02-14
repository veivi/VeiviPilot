#include "PWMOutput.h"
#include <avr/io.h>

#define PWM_HZ 50 // (100.141592654)
#define TIMER_HZ (16e6/8)

static const uint8_t outputModeMask[] = { 1<<COM1A1, 1<<COM1B1, 1<<COM1C1 };

void pwmTimerInit(const struct HWTimer *timer[], int num)
{
  for(int i = 0; i < num; i++) { 
    // WGM, prescaling
   
    *(timer[i]->TCCRA) = 1<<WGM11;
    *(timer[i]->TCCRB) = (1<<WGM13) | (1<<WGM12) | (1<<CS11);

   // PWM frequency
   
   *(timer[i]->ICR) = TIMER_HZ/PWM_HZ - 1;

   // Output set to 1.5 ms by default

   for(int j = 0; j < 3; j++)
     *(timer[i]->OCR[j]) = ~0U;
  }
}

void pwmEnable(const struct PWMOutput *output)
{
   *(output->timer->TCCRA) |= outputModeMask[output->pwmCh];
}

void pwmDisable(const struct PWMOutput *output)
{
   *(output->timer->TCCRA) &= ~outputModeMask[output->pwmCh];
}

void pwmOutputInit(const struct PWMOutput *output)
{
  configureOutput(&output->pin);
  pwmOutputWrite(output, 1500);
  pwmEnable(output);
}

void pwmOutputInitList(const struct PWMOutput output[], int num)
{
   for(int i = 0; i < num; i++)
      pwmOutputInit(&output[i]);
}

uint16_t constrain_period(uint16_t p) {
    if (p > RC_OUTPUT_MAX_PULSEWIDTH)
       return RC_OUTPUT_MAX_PULSEWIDTH;
    else if (p < RC_OUTPUT_MIN_PULSEWIDTH)
       return RC_OUTPUT_MIN_PULSEWIDTH;
    else return p;
}

void pwmOutputWrite(const struct PWMOutput *output, uint16_t value)
{
  if(!output)
    return;
  
   *(output->timer->OCR[output->pwmCh]) = constrain_period(value) << 1;
}

