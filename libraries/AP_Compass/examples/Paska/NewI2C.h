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
    uint8_t write(uint8_t, uint8_t, const uint8_t*, int);
    uint8_t write(uint8_t, uint16_t, const uint8_t*, int);
    uint8_t write(uint8_t, const uint8_t*, int, const uint8_t*, int);
    uint8_t read(uint8_t, uint8_t, uint8_t*, int);
    uint8_t read(uint8_t, uint16_t, uint8_t*, int);
    uint8_t read(uint8_t, const uint8_t*, int, uint8_t*, int);


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

#endif


