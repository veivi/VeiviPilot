#include <avr/io.h>
#include "InputOutput.h"

const struct PortDescriptor portTable[] = {
  [PortA] = { &PINA, &PORTA, &DDRA },
  [PortB] = { &PINB, &PORTB, &DDRB, &PCMSK0, 0 },
  [PortC] = { &PINC, &PORTC, &DDRC },
  [PortD] = { &PIND, &PORTD, &DDRD },
  [PortE] = { &PINE, &PORTE, &DDRE },
  [PortF] = { &PINF, &PORTF, &DDRF },
  [PortG] = { &PING, &PORTG, &DDRG },
  [PortH] = { &PINH, &PORTH, &DDRH },
  [PortK] = { &PINK, &PORTK, &DDRK, &PCMSK2, 2 },
  [PortL] = { &PINL, &PORTL, &DDRL }
};

const portName_t pcIntPort[] = {
  PortB, PortF, PortK
};

const uint8_t pcIntMask[] = {
  1<<PCIE0, 1<<PCIE1, 1<<PCIE2
};

void pinOutputEnable(const struct PinDescriptor *pin, bool output)
{
  if(output)
    *(portTable[pin->port].ddr) |= 1<<(pin->index);
  else
    *(portTable[pin->port].ddr) &= ~(1<<(pin->index));
}  

void setPinState(const struct PinDescriptor *pin, uint8_t state)
{
  if(state > 0)
    *(portTable[pin->port].port) |= 1<<(pin->index);
  else
    *(portTable[pin->port].port) &= ~(1<<(pin->index));
}

uint8_t getPinState(const struct PinDescriptor *pin)
{
  return (*(portTable[pin->port].pin)>>(pin->index)) & 1;
}

void configureInput(const struct PinDescriptor *pin, bool pullup)
{
  pinOutputEnable(pin, false);
  setPinState(pin, pullup ? 1 : 0);
}

void configureOutput(const struct PinDescriptor *pin)
{
  pinOutputEnable(pin, true);
}

