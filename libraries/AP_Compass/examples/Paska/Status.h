//
// Global status record
//

#ifndef STATUS_H
#define STATUS_H

struct StatusRecord {
  bool armed;
  bool consoleLink;
  bool simulatorLink;
  bool silent;
  bool positiveIAS;
  bool fullStop;
  bool pitotBlocked;
  bool iasFailed = false, iasWarn = false;
  bool alphaFailed = false, alphaWarn = false;
  bool eepromFailed = false, eepromWarn = false;
};

extern struct StatusRecord vpStatus;

#endif
