#include "Interrupt.h"
#include "Console.h"
#include "RxInput.h"
#include <stdlib.h>

uint8_t nestCount = 0;
extern bool armed;

extern "C" ISR(BADISR_vect)
{
   sei();
   consoleNoteLn("PASKA KESKEYTYS.");

   if(!armed)
     abort();
}


extern "C" ISR(PCINT0_vect) { rxInterrupt_callback(0); }
extern "C" ISR(PCINT1_vect) { rxInterrupt_callback(1); }
extern "C" ISR(PCINT2_vect) { rxInterrupt_callback(2); }

