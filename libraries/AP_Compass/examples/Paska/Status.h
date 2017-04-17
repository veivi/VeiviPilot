//
// Global status record
//

#ifndef STATUS_H
#define STATUS_H

#include "NewI2C.h"

struct StatusRecord {
  bool armed;
  bool consoleLink;
  bool simulatorLink;
  bool silent;
  bool positiveIAS;
  bool fullStop;
  bool pitotBlocked;
  bool stall;
  bool weightOnWheels;
  bool aloft;
};

extern struct StatusRecord vpStatus;

//
//
//
extern I2CDevice alphaDevice, pitotDevice;

bool pitotFailed();
bool alphaFailed();

#endif
