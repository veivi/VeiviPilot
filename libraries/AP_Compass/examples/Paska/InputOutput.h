#ifndef INPUTOUTPUT_H
#define INPUTOUTPUT_H

#include <stdint.h>

typedef enum { PortA, PortB, PortC, PortD, PortE, PortF, PortG, PortH, PortK, PortL } portName_t;

struct PortDescriptor {
  volatile uint8_t *pin, *port, *ddr, *mask;
  uint8_t pci;
};

extern const struct PortDescriptor portTable[];
  
struct PinDescriptor {
  portName_t port;
  uint8_t index;
};

extern const portName_t pcIntPort[];
extern const uint8_t pcIntMask[];

void pinOutputEnable(struct PinDescriptor *pin, bool output);
void setPinState(struct PinDescriptor *pin, uint8_t state);
uint8_t getPinState(struct PinDescriptor *pin);  
void configureInput(struct PinDescriptor *pin, bool pullup);
void configureOutput(struct PinDescriptor *pin);

#endif

