//
// Global status record
//

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
};

extern struct StatusRecord vpStatus;
