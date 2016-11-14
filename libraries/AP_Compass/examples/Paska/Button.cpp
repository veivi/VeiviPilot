#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_AVR/AP_HAL_AVR.h>
#include <math.h>
#include "Button.h"

extern AP_HAL::HAL& hal;

Button :: Button(float aValue)
{
  activeValue = aValue;
}

void Button :: reset()
{
  count = 0;
  pulseSingle = pulseDouble = buttonPress = pulseArmed = false;
  transition = 0;
}

void Button :: input(float inputValue)
{
  inputState = fabsf(inputValue - activeValue) < 0.1;

  if(!inputState)
    inertiaState = false;

  else if(!inertiaState) {
    inertiaState = true;
    inputState = false;
  }
  
  if(inputState != statePrev) {
    transition = hal.scheduler->micros();

    if(inputState)
      pulseArmed = true;
      
    else {
      stateLazy = false;
      
      if(pulseArmed) {
	if(count > 0) {
	  pulseDouble = true;
	  pulseSingle = false;
	  count = 0;
	} else
	  count = 1;
      
	pulseArmed = false;
      }
    }
    
    statePrev = inputState;
  } else if(hal.scheduler->micros() - transition > 2.0e6/3) {

    if(stateLazy != inputState)
      buttonPress = inputState;
    
    stateLazy = inputState;
    
    if(!inputState && count > 0)
      pulseSingle = true;

    count = 0;
    pulseArmed = false;
  }
}

bool Button::state(void)
{
  return inputState && inertiaState;
}  

bool Button::lazy(void)
{
  return stateLazy;
}  

bool Button::depressed(void)
{
  bool value = buttonPress;
  buttonPress = false;
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
