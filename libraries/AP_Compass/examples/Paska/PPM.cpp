#include "PPM.h"
#include "Interrupt.h"
#include "Math.h"
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

static bool calibrating;

void calibStart()
{
  calibrating = true;
  
  for(int i = 0; i < numInputs; i++)
    if(inputRecords[i]) {
      inputRecords[i]->pulseCenter
	= inputRecords[i]->pulseMin
	= inputRecords[i]->pulseMax = inputRecords[i]->pulseWidthLast;
    }
}
  
void calibStop(int32_t *min, int32_t *center, int32_t *max)
{
  if(calibrating) {
    for(uint8_t i = 0; i < numInputs; i++) {
      min[i] = inputRecords[i]->pulseMin;
      center[i] = inputRecords[i]->pulseCenter;
      max[i] = inputRecords[i]->pulseMax;
    }

    calibrating = false;
  }
}
  
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
       inputRecords[i]->pulseCount = 1;
       inputRecords[i]->pulseWidthAcc = inputRecords[i]->pulseWidthLast
	 = pulse[i]/2;              

       if(calibrating) {
	 inputRecords[i]->pulseMin = MIN(inputRecords[i]->pulseWidthLast, inputRecords[i]->pulseMin);
	 inputRecords[i]->pulseMax = MAX(inputRecords[i]->pulseWidthLast, inputRecords[i]->pulseMax);
       }
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

void ppmInputInit(struct RxInputRecord *inputs[], int num, const int32_t *min, const int32_t *center, const int32_t *max)
{
  inputRecords = inputs;
  numInputs = MIN(num, AVR_RC_INPUT_MAX_CHANNELS);
  
  for(uint8_t i = 0; i < num; i++) {
    inputs[i]->pulseMin = min[i];
    inputs[i]->pulseCenter = center[i];
    inputs[i]->pulseMax = max[i];
  }
  
  FORBID;
    
  TCCR5B |= (1<<ICES5) | (1<<CS51);
  TIMSK5 |= 1<<ICIE5;
     
  PERMIT;
}
