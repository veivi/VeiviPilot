#ifndef INTERRUPT_H
#define INTERRUPT_H

#include <stdint.h>

extern uint8_t nestCount;

#define FORBID if(!nestCount++) asm("cli")
#define PERMIT if(!--nestCount) asm("sei")

#endif

