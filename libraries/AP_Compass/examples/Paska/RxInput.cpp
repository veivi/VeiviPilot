#include "RxInput.h"
#include "Interrupt.h"
#include "Console.h"
#include <avr/io.h>
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

struct RxInputRecord *rxInputIndex0[8], *rxInputIndex1[8], *rxInputIndex2[8];
struct RxInputRecord **rxInputIndexList[] = { rxInputIndex0, rxInputIndex1, rxInputIndex2 };
uint8_t log2Table[1<<8];
bool pciWarn;

bool inputValid(struct RxInputRecord *record)
{
  return record->pulseCount > 0;
}

float inputValue(struct RxInputRecord *record)
{
  FORBID;
  
  int32_t value = record->pulseWidthLast;
  
  PERMIT;
  
  if(value < record->pulseCenter) {
    if(record->pulseCenter-record->pulseMin < 100)
      return 0;
    else
      return (float) (value - record->pulseCenter)/(record->pulseCenter-record->pulseMin);
  } else {
    if(record->pulseMax-record->pulseCenter < 100)
      return 0;
    else
      return (float) (value - record->pulseCenter)/(record->pulseMax-record->pulseCenter);
    }
}

void rxInputInit(struct RxInputRecord *record)
{
  static bool initialized = false;

  if(!initialized) {
    for(int i = 1; i < (1<<8); i++) {
      int j = 7;
      while(((1<<j) & i) == 0 && j > 0)
	j--;
      log2Table[i] = j;
    }

    PCMSK0 = PCMSK1 = PCMSK2 = 0;

    initialized = true;
  }
  
  const struct PortDescriptor *port = &portTable[record->pin.port];

  if(port->mask) {
    FORBID;
    rxInputIndexList[port->pci][record->pin.index] = record;
    *port->mask |= 1<<record->pin.index;
    PCICR |= pcIntMask[port->pci];
    PERMIT;
  } else
      consoleNoteLn("PASKA PCI-PORTTI");
}

extern "C" void rxInterrupt_callback(uint8_t num)
{
  const struct PortDescriptor *port = &portTable[pcIntPort[num]];

  if(!port || !port->mask) {
    consoleNoteLn("PASKA PCI.");
    return;
  }

  static uint8_t prevState[3];
  uint8_t state = *port->pin, event = (state ^ prevState[num]) & *port->mask;

  prevState[num] = state;
  
  uint32_t current = hal.scheduler->micros();
  
  while(event) {
    uint8_t i = log2Table[event];
    uint8_t mask = 1U<<i;
  
    if(!rxInputIndexList[num][i]) {
      pciWarn = true;
    } else if(rxInputIndexList[num][i]->freqOnly) {
      rxInputIndexList[num][i]->pulseCount += (state & mask) ? 1 : 0;
    } else if(state & mask) {
      rxInputIndexList[num][i]->pulseStart = current;
    } else if(rxInputIndexList[num][i]->pulseStart > 0) {
      uint32_t width = current - rxInputIndexList[num][i]->pulseStart;
      rxInputIndexList[num][i]->pulseWidthAcc += width;
      rxInputIndexList[num][i]->pulseWidthLast = width;
      rxInputIndexList[num][i]->pulseCount++;      
      rxInputIndexList[num][i]->alive = true;
    }
    
    event &= ~mask;
  }
}

int8_t readSwitch(struct SwitchRecord *record)
{
  if(inputValid(record->input)) {
    const float value = inputValue(record->input),
      diff = fabsf(value - record->prevValue);
    
    record->prevValue = value;
  
    if(diff < 0.05) {
      if(fabs(value) < 1.0/3)
	record->state = 0;
      else
	record->state = value < 0.0 ? -1 : 1;
    }
  }

  return record->state;
}

float applyNullZone(float value, bool *pilotInput)
{
  if(pilotInput)
    *pilotInput = true;
  
  if(value < -NULLZONE)
    return (value + NULLZONE) / (1.0 - NULLZONE);
  else if(value > NULLZONE)
    return (value - NULLZONE) / (1.0 - NULLZONE);

  if(pilotInput)
    *pilotInput = false;
    
  return 0.0;
}

float applyNullZone(float value)
{
  return applyNullZone(value, NULL);
}

