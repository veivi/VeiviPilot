#ifndef INTERRUPT_H
#define INTERRUPT_H

#include <stdint.h>
#include <avr/interrupt.h>

extern uint8_t nestCount;

#define FORBID if(!nestCount++) cli()
#define PERMIT if(!--nestCount) sei()

#endif

