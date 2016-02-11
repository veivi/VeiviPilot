#include <AP_HAL/AP_HAL.h>
#define CONSOLE_PRIVATE_H
#include "Console.h"
#include "Serial.h"
#include "Datagram.h"
#include <stdarg.h>
#include <math.h>

extern const AP_HAL::HAL& hal;

static void newline(void);
bool talk = true;

#define BUF_SIZE (1<<6)

uint8_t outputBuf[BUF_SIZE];
uint8_t bufPtr;

void consoleFlush()
{
  datagramTxStart(DG_CONSOLE_OUT);
  datagramTxOut(outputBuf, bufPtr);
  datagramTxEnd();
  
  bufPtr = 0;
}

void consoleOut(const uint8_t c)
{
  if(bufPtr > BUF_SIZE-1)
    consoleFlush();

  outputBuf[bufPtr++] = c;
}

static void newline(void)
{
  consolePrint("\n");
  consoleFlush();
}

void consoleNote(const char *s)
{
  consolePrint("// ");
  consolePrint(s);
}

void consoleNoteLn(const char *s)
{
  consoleNote(s);
  newline();
}

void consoleNote_P(const prog_char_t *s)
{
  consolePrint("// ");
  consolePrint_P(s);
}

void consoleNoteLn_P(const prog_char_t *s)
{
  consoleNote_P(s);
  newline();
}

void consolevNotef(const char *s, va_list argp)
{
  consolePrint("// ");
  consolevPrintf(s, argp);
}

void consoleNotef(const char *s, ...)
{
  va_list argp;

  va_start(argp, s);
  consolevNotef(s, argp);
  va_end(argp);
}

void consoleNotefLn(const char *s, ...)
{
  va_list argp;

  va_start(argp, s);
  consolevNotef(s, argp);
  va_end(argp);
  
  newline();
}

void consolePrintf(const char *s, ...)
{
  va_list argp;

  va_start(argp, s);
  consolevPrintf(s, argp);
  va_end(argp);
}

void consolePrintfLn(const char *s, ...)
{
  va_list argp;

  va_start(argp, s);
  consolevPrintf(s, argp);
  va_end(argp);

  newline();
}

void consolevPrintf(const char *s, va_list argp)
{
  if(talk)
    // Sorry, a bit crap
    consolePrint(s);
}

void consolePrint(const char *s)
{
  if(!talk)
    return;

  while(*s)
    consoleOut(*s++);
}

void consolePrint_P(const prog_char_t *s)
{
  if(!talk)
    return;
  
  uint8_t c = 0;

  while((c = pgm_read_byte(s++)))
    consoleOut(c);
}

#define printDigit(d) consoleOut('0'+d)
// #define USE_PRINTF

void consolePrint(float v, int p)
{
  if(!talk)
    return;
     
#ifdef USE_PRINTF
  const char fmt[] = {'%', '.', '0'+p, 'f', '\0'};
  hal.console->printf(fmt, (double) v);
#else
  if(v < 0.0) {
    consoleOut('-');
    v = -v;
  }

  v += 0.5*pow(0.1, p);
  
  uint32_t i = (uint32_t) v;

  consolePrint(i);

  if(p > 0) {
    float f = v - (float) i;
  
    consolePrint('.');

    while(p > 0) {
      f *= 10.0;
      printDigit(((uint32_t) f) % 10);
      p--;
    }
  }
#endif  
}

void consolePrint(const char c)
{
  consoleOut(c);
}  

void consolePrint(float v)
{
  consolePrint(v, 2);
}

void consolePrint(double v, int p)
{
  consolePrint((float) v, p);
}

void consolePrint(double v)
{
  consolePrint(v, 2);
}

void consolePrint(int v)
{
  consolePrint((long) v);
}

void consolePrint(unsigned int v)
{
  consolePrint((unsigned long) v);
}

void consolePrint(long v)
{
  if(!talk)
    return;
  
  if(v < 0) {
    v = -v;
    consoleOut('-');
  }

  consolePrint((unsigned long) v);
}

void consolePrint(unsigned long v)
{
  if(!talk)
    return;
  
  uint8_t buf[20];
  int l = 0;

  while(v > 0 && l < 20) {
    buf[l++] = v % 10;
    v /= 10;
  }

  if(l > 0) {
    while(l > 0)
      printDigit(buf[--l]);
  } else
    printDigit(0);
}

void consolePrint(uint8_t v)
{
  consolePrint((unsigned int) v);
}

void consolePrintLn(const char *s)
{
  consolePrint(s);
  newline();
}

void consolePrintLn_P(const prog_char_t *s)
{
  consolePrint_P(s);
  newline();
}

void consolePrintLn(float v)
{
  consolePrint(v);
  newline();
}

void consolePrintLn(float v, int p)
{
  consolePrint(v, p);
  newline();
}

void consolePrintLn(double v)
{
  consolePrint(v);
  newline();
}

void consolePrintLn(double v, int p)
{
  consolePrint(v, p);
  newline();
}

void consolePrintLn(int v)
{
  consolePrint(v);
  newline();
}

void consolePrintLn(unsigned int v)
{
  consolePrint(v);
  newline();
}

void consolePrintLn(uint8_t v)
{
  consolePrint(v);
  newline();
}

void consolePrintLn(long v)
{
  consolePrint(v);
  newline();
}

void consolePrintLn(unsigned long v)
{
  consolePrint(v);
  newline();
}

