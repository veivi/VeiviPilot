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
};

extern struct StatusRecord vpStatus;

#endif