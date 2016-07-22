#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_AVR/AP_HAL_AVR.h>
#include <math.h>
#include "Button.h"

extern AP_HAL::HAL& hal;

Button :: Button(float aValue)
{
  activeValue = aValue;
}

void Button :: input(float inputValue)
{
  bool inputState = fabs(inputValue - activeValue) < 0.05;

  if(inertia != inputState) {
    inertia = inputState;
    return;
  }
  
  if(inputState != statePrev) {
    pulseStart = hal.scheduler->micros();

    if(inputState)
      pulseArmed = true;
    else if(pulseArmed) {
      if(count > 0) {
	pulseDouble = true;
	count = 0;
      } else
	count = 1;
      
      pulseArmed = false;
    }
    
    statePrev = inputState;
  } else if(hal.scheduler->micros() - pulseStart > 1.0e6/2) {

    if(stateLazy != inputState)
      stateActive = inputState;
    
    stateLazy = inputState;
    
    if(!inputState && count > 0)
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

