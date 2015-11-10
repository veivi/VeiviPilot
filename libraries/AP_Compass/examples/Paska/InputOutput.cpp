#include <avr/io.h>
#include "InputOutput.h"

const struct PortDescriptor portTable[] = {
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

void pinOutputEnable(struct PinDescriptor *pin, bool output)
{
  if(output)
    *(portTable[pin->port].ddr) |= 1<<(pin->index);
  else
    *(portTable[pin->port].ddr) &= ~(1<<(pin->index));
}  

void setPinState(struct PinDescriptor *pin, uint8_t state)
{
  if(state > 0)
    *(portTable[pin->port].port) |= 1<<(pin->index);
  else
    *(portTable[pin->port].port) &= ~(1<<(pin->index));
}

uint8_t getPinState(struct PinDescriptor *pin)
{
  return (*(portTable[pin->port].pin)>>(pin->index)) & 1;
}

void configureInput(struct PinDescriptor *pin, bool pullup)
{
  pinOutputEnable(pin, false);
  setPinState(pin, pullup ? 1 : 0);
}

void configureOutput(struct PinDescriptor *pin)
{
  pinOutputEnable(pin, true);
}

