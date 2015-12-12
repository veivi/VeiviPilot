#include "PPM.h"
#include "Interrupt.h"
#include "Filter.h"
#include <AP_HAL/AP_HAL.h>
#include <avr/interrupt.h>

extern const AP_HAL::HAL& hal;

#define AVR_RC_INPUT_MAX_CHANNELS 10
#define AVR_RC_INPUT_MIN_CHANNELS 6

/*
  mininum pulse width in microseconds to signal end of a PPM-SUM
  frame. This value is chosen to be smaller than the default 3000 sync
  pulse width for OpenLRSng. Note that this is the total pulse with
  (typically 300us low followed by a long high pulse)
 */

#define AVR_RC_INPUT_MIN_SYNC_PULSE_WIDTH 2700

static uint16_t _pulse_capt[AVR_RC_INPUT_MAX_CHANNELS];

uint8_t ppmNumChannels;
uint32_t ppmFrames;
bool ppmWarnShort, ppmWarnSlow;

static struct RxInputRecord **inputRecords;
static int numInputs;

static void handlePPMInput(const uint16_t *pulse, int numCh)
{
  static uint32_t prev;
  uint32_t current = hal.scheduler->micros(), cycle = current - prev;

  if(prev > 0 && cycle > 30000)
    ppmWarnSlow = true;

  prev = current;
  
  ppmNumChannels = numCh;
  ppmFrames++;

  for(int i = 0; i < numCh; i++) {
    if(inputRecords[i]) {
       inputRecords[i]->alive = true;
       inputRecords[i]->pulseCount++;
       inputRecords[i]->pulseWidthAcc += pulse[i]/2;              
     }
  }
}

extern "C" ISR(TIMER5_CAPT_vect)
{
  static uint16_t icr5_prev;
  static uint8_t  channel_ctr;

  const uint16_t icr5_current = ICR5;
  uint16_t pulse_width;
    
  if (icr5_current < icr5_prev) {
    pulse_width =  OCR5A + 1 + icr5_current - icr5_prev;
  } else {
    pulse_width = icr5_current - icr5_prev;
  }

  if (pulse_width > AVR_RC_INPUT_MIN_SYNC_PULSE_WIDTH*2) {
    // sync pulse
	
    if( channel_ctr < AVR_RC_INPUT_MIN_CHANNELS )
      ppmWarnShort = true;
    else
      handlePPMInput(_pulse_capt, channel_ctr);
 
    channel_ctr = 0;
  } else if (channel_ctr < numInputs)
    _pulse_capt[channel_ctr++] = pulse_width;

  icr5_prev = icr5_current;
}

void ppmInputInit(struct RxInputRecord *inputs[], int num)
{

  inputRecords = inputs;
  numInputs = min(num, AVR_RC_INPUT_MAX_CHANNELS);
  
  FORBID;
    
  TCCR5B |= (1<<ICES5) | (1<<CS51);
  TIMSK5 |= 1<<ICIE5;
     
  PERMIT;
}
