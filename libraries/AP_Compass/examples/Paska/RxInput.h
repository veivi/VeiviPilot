#ifndef RXINPUT_H
#define RXINPUT_H

#include <stdint.h>

//

typedef enum { PortA, PortB, PortC, PortD, PortE, PortF, PortG, PortH, PortK, PortL } portName_t;

struct PortDescriptor {
  volatile uint8_t *pin, *port, *ddr;
};

extern const struct PortDescriptor ports[];
  
struct RxInputRecord {
  const struct PortDescriptor *port;
  uint8_t index;
  bool freqOnly, alive;
  uint32_t pulseStart;
  uint32_t pulseCount;
  uint32_t pulseWidthAcc;
};
  
void rxInputInit(const struct RxInputRecord *inputs);
bool inputValid(struct RxInputRecord *record);
float inputValue(struct RxInputRecord *record);
float decodePWM(float pulse);

#endif

