#include <stdint.h>
#include <stdbool.h>

void datagramTxStart(uint8_t );
void datagramTxOutByte(const uint8_t c);
void datagramTxOut(const uint8_t *data, int l);
void datagramTxEnd(void);
bool datagramRxInputChar(const uint8_t c);

#define DG_HEARTBEAT     1
#define DG_CONSOLE       2
#define DG_LOGDATA       3
#define DG_LOGINFO       4
#define DG_PARAMS        5
#define DG_INITIALIZED   6
#define DG_READY         7
#define DG_SIMLINK       8

extern void datagramInterpreter(uint8_t t, const uint8_t *data, int size);
extern void datagramSerialOut(uint8_t);
extern int maxDatagramSize;
extern uint8_t datagramRxStore[];

struct SimLinkSensor {
  float alpha, alt, ias;
  float roll, pitch, heading;
  float rrate, prate, yrate;
  float accx, accy, accz;
};

struct SimLinkControl {
  float aileron, elevator, throttle, rudder;
};


  
