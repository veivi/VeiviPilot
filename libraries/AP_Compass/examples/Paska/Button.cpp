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
  if(fabsf(inputValue - activeValue) > 0.05) {
    inertiaCount = 0;
    inputState = false;
  } else if(inertiaCount < INERTIA)
    inertiaCount++;
  else
    inputState = true;

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
  } else if(hal.scheduler->micros() - transition > 0.5e6) {

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
  return inputState;
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

