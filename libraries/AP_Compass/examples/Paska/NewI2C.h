#ifndef NEWI2C_H
#define NEWI2C_H

#include <avr/io.h>

bool handleFailure(const char *name, bool fail, bool *warn, bool *failed, int *count);

class NewI2C
{
  public:
    void begin();
    void end();
    void timeOut(uint16_t);
    void setSpeed(uint8_t); 
    void pullup(uint8_t);
    uint8_t wait(uint8_t);
    uint8_t write(uint8_t, const uint8_t*, uint8_t);
    uint8_t write(uint8_t, uint8_t, const uint8_t*, uint8_t);
    uint8_t write(uint8_t, uint16_t, const uint8_t*, uint8_t);
    uint8_t write(uint8_t, const uint8_t*, uint8_t, const uint8_t*, uint8_t);
    uint8_t read(uint8_t, uint8_t*, uint8_t);
    uint8_t read(uint8_t, uint8_t, uint8_t*, uint8_t);
    uint8_t read(uint8_t, uint16_t, uint8_t*, uint8_t);
    uint8_t read(uint8_t, const uint8_t*, uint8_t, uint8_t*, uint8_t);


  private:
    uint8_t start();
    uint8_t transmitByte(uint8_t);
    uint8_t receiveByte(bool);
    uint8_t stop();
    void lockUp();
    uint8_t returnStatus;
    uint8_t nack;
    static uint16_t timeOutDelay;
};

class I2CDevice {
 public:
  I2CDevice(NewI2C *interface, uint8_t addr, const char *name);
  bool handleStatus(bool);
  bool hasFailed();
  bool status();

 private:
  bool warn, failed;
  int failCount;
  uint32_t failedAt, backoff;
  const char *name;
  //  NewI2C *interface;
  //  uint8_t addr;
};

#endif


