#include "RxInput.h"
#include "Interrupt.h"
#include <avr/io.h>

const struct PortDescriptor ports[] = {
  [PortA] = { &PINA, &PORTA, &DDRA },
  [PortB] = { &PINB, &PORTB, &DDRB },
  [PortC] = { &PINC, &PORTC, &DDRC },
  [PortD] = { &PIND, &PORTD, &DDRD },
  [PortE] = { &PINE, &PORTE, &DDRE },
  [PortF] = { &PINF, &PORTF, &DDRF },
  [PortG] = { &PING, &PORTG, &DDRG },
  [PortH] = { &PINH, &PORTH, &DDRH },
  [PortK] = { &PINK, &PORTK, &DDRK },
  [PortL] = { &PINL, &PORTL, &DDRL }
};

float decodePWM(float pulse) {
  const float txRange = 0.81;
  return (pulse - 1500)/500.0/txRange;
}

void rxInputInit(const struct RxInputRecord *input)
{
  *(input->port->ddr) &= ~(1<<(input->index));
  *(input->port->port) |= 1<<(input->index);
}

bool inputValid(struct RxInputRecord *record)
{
  return record->pulseCount > 0;
}

float inputValue(struct RxInputRecord *record)
{
  FORBID;
  
  uint32_t count = record->pulseCount, acc = record->pulseWidthAcc;
  record->pulseWidthAcc = record->pulseCount = 0;
  
  PERMIT;
  
  return (float) acc / count;
}

