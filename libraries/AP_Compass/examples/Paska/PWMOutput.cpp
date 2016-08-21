#include "PWMOutput.h"
#include <avr/io.h>

#define PWM_HZ 50
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
     *(timer[i]->OCR[j]) = 1500UL<<1; // ~0U;
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

void pwmOutputInit(struct PWMOutput *output)
{
  pwmDisable(output);
  setPinState(&output->pin, 0);
  configureOutput(&output->pin);
  output->active = false;
}

void pwmOutputInitList(struct PWMOutput output[], int num)
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

void pwmOutputWrite(struct PWMOutput *output, uint16_t value)
{
  if(!output)
    return;
  
  *(output->timer->OCR[output->pwmCh]) = constrain_period(value) << 1;

  if(!output->active) {
    configureOutput(&output->pin);
    pwmEnable(output);
    output->active = true;
  }
}

