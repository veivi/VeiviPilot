#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_AVR/AP_HAL_AVR.h>
#include "Button.h"

extern AP_HAL::HAL& hal;

void Button :: input(bool newState)
{
  if(newState != statePrev) {
    pulseStart = hal.scheduler->micros();

    if(newState)
      pulseArmed = true;
    else if(pulseArmed) {
      if(count > 0) {
	pulseDouble = true;
	count = 0;
      } else
	count = 1;
      
      pulseArmed = false;
    }
    
    statePrev = newState;
  } else if(hal.scheduler->micros() - pulseStart > 1.0e6/3) {

    if(stateLazy != newState)
      stateActive = newState;
    
    stateLazy = newState;
    
    if(!newState && count > 0)
      pulseSingle = true;

    count = 0;
    pulseArmed = false;
  }
}

bool Button::state(void)
{
  return stateLazy;
}  

bool Button::active(void)
{
  bool value = stateActive;
  stateActive = false;
  return value;
}  

bool Button::singlePulse(void)
{
  bool value = pulseSingle;
  pulseSingle = false;
  return value;
}  

bool Button::doublePulse(void)
{
  bool value = pulseDouble;
  pulseDouble = false;
  return value;
}  

