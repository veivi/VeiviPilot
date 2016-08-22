#include <stdint.h>
#include "Datagram.h"
#include "CRC16.h"

static uint16_t crcState;
static int datagramSize = 0;
static int datagrams, datagramsGood;

static void outputBreak()
{
  datagramSerialOut((const uint8_t) 0x00);
  datagramSerialOut((const uint8_t) 0x00);
  datagramSerialFlush();
}

void datagramTxOutByte(const uint8_t c)
{
  datagramSerialOut(c);
    
  if(c == 0x00)
    datagramSerialOut((const uint8_t) 0xFF);

  crcState = crc16_update(crcState, c);
}

void datagramTxOut(const uint8_t *data, int l)
{
  while(l-- > 0)
    datagramTxOutByte(*data++);
}

void datagramTxStart(uint8_t dg)
{
  outputBreak();
  crcState = 0xFFFF;
  datagramTxOutByte((const uint8_t) dg);
}

void datagramTxEnd(void)
{
  uint16_t buf = crcState;
  datagramTxOut((const uint8_t*) &buf, sizeof(buf));
  outputBreak();
}

static void storeByte(const uint8_t c)
{
    if(datagramSize < maxDatagramSize)
        datagramRxStore[datagramSize++] = c;
}
  
static bool datagramRxEnd(void)
{
    bool success = false;
    
    if(datagramSize > 2) {
        datagrams++;
        uint16_t crcReceived = *((uint16_t*) &datagramRxStore[datagramSize-2]);
        uint16_t crc = crc16(0xFFFF, datagramRxStore, datagramSize - 2);
        success = crc == crcReceived;
        
        if(success) {
            datagramsGood++;
	    datagramInterpreter(datagramRxStore[0], &datagramRxStore[1], datagramSize-3);
        } else
	  datagramRxError("CRC FAIL");
    } else
      datagramRxError("TOO SHORT");
    
    datagramSize = 0;
    return success;
}

bool datagramRxInputChar(const uint8_t c)
{
    static uint8_t prev;
    static bool busy = false;
    bool success = false;

    if(prev == 0x00 && c == 0x00) {
        if(busy)
            success = datagramRxEnd();
        
        busy = false;
    } else if(busy) {
        if(prev == 0x00)
            storeByte(0x00);
        else if(c != 0x00)
            storeByte(c);
    } else if(c != 0x00) {
        busy = true;
        datagramSize = 0;
        storeByte(c);
    }
    
    prev = c;
    
    return success;
}


