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

void pinOutputEnable(const struct PinDescriptor *pin, bool output);
void setPinState(const struct PinDescriptor *pin, uint8_t state);
uint8_t getPinState(const struct PinDescriptor *pin);  
void configureInput(const struct PinDescriptor *pin, bool pullup);
void configureOutput(const struct PinDescriptor *pin);

#endif

