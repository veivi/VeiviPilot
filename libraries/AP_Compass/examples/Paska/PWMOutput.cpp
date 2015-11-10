#include "PWMOutput.h"
#include <avr/io.h>

#define PWM_HZ 50
#define TIMER_HZ (16e6/8)

static const uint8_t outputModeMask[3] = { 1<<COM1A1, 1<<COM1B1, 1<<COM1C1 };

#define OUTPUT 1

void pwmTimerInit(struct HWTimer *timer)
{
   if(timer->initDone)
      return;
      
   // WGM, prescaling
   
   *(timer->TCCRA) = 1<<WGM11;
   *(timer->TCCRB) = (1<<WGM13) | (1<<WGM12) | (1<<CS11);

   // PWM frequency
   
   *(timer->ICR) = TIMER_HZ/PWM_HZ - 1;

   // Output set to nil by default

   for(int i = 0; i < 3; i++)
      *(timer->OCR[i]) = 0xFFFF;

   timer->initDone = true;
}

void pwmEnable(struct PWMOutput *output)
{
   *(output->timer->TCCRA) |= outputModeMask[output->pwmCh];
}

void pwmDisable(struct PWMOutput *output)
{
   *(output->timer->TCCRA) &= ~outputModeMask[output->pwmCh];
}

void pwmOutputInit(struct PWMOutput *output)
{
   pwmTimerInit(output->timer);
   pwmEnable(output);
   configureOutput(&output->pin);
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
   *(output->timer->OCR[output->pwmCh]) = constrain_period(value) << 1;
}

