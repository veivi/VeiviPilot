#include <AP_HAL/AP_HAL.h>
#define CONSOLE_PRIVATE_H
#include "Console.h"
#include "Status.h"
#include <stdarg.h>
#include <math.h>

extern "C" {
#include "Datagram.h"
}

extern const AP_HAL::HAL& hal;

#define BUF_SIZE (1<<6)

static uint8_t outputBuf[BUF_SIZE];
static uint8_t bufPtr;
static int column;

void consoleFlush()
{
  if(bufPtr > 0) {
    datagramTxStart(DG_CONSOLE);
    datagramTxOut(outputBuf, bufPtr);
    datagramTxEnd();
  }
  
  bufPtr = 0;
}

void consoleOut(const uint8_t c)
{
  if(!vpStatus.consoleLink || vpStatus.silent)
    return;
  
  if(bufPtr > BUF_SIZE-1)
    consoleFlush();

  outputBuf[bufPtr++] = c;
  column++;
}

void consoleNL(void)
{
  consoleOut('\n');
  consoleFlush();
  column = 0;
}

void consoleCR(void)
{
  consoleOut('\r');
  consoleFlush();
  column = 0;
}

void consoleTab(int i)
{
  while(column < i)
    consoleOut(' ');
}

void consoleNote(const char *s)
{
  consolePrint("// ");
  consolePrint(s);
}

void consoleNoteLn(const char *s)
{
  consoleNote(s);
  consoleNL();
}

void consoleNote_P(const prog_char_t *s)
{
  consolePrint("// ");
  consolePrint_P(s);
}

void consoleNoteLn_P(const prog_char_t *s)
{
  consoleNote_P(s);
  consoleNL();
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
  
  consoleNL();
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

  consoleNL();
}

void consolevPrintf(const char *s, va_list argp)
{
  consolePrint(s);
}

void consolePrint(const char *s)
{
  while(*s)
    consoleOut(*s++);
}

void consolePrint(const char *s, int l)
{
  while(*s && l-- > 0)
    consoleOut(*s++);
}

void consolePrint_P(const prog_char_t *s)
{
  uint8_t c = 0;

  while((c = pgm_read_byte(s++)))
    consoleOut(c);
}

#define printDigit(d) consoleOut('0'+d)
// #define USE_PRINTF

void consolePrint(float v, int p)
{
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
  if(v < 0) {
    v = -v;
    consoleOut('-');
  }

  consolePrint((unsigned long) v);
}

void consolePrint(unsigned long v)
{
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
  consoleNL();
}

void consolePrintLn_P(const prog_char_t *s)
{
  consolePrint_P(s);
  consoleNL();
}

void consolePrintLn(float v)
{
  consolePrint(v);
  consoleNL();
}

void consolePrintLn(float v, int p)
{
  consolePrint(v, p);
  consoleNL();
}

void consolePrintLn(double v)
{
  consolePrint(v);
  consoleNL();
}

void consolePrintLn(double v, int p)
{
  consolePrint(v, p);
  consoleNL();
}

void consolePrintLn(int v)
{
  consolePrint(v);
  consoleNL();
}

void consolePrintLn(unsigned int v)
{
  consolePrint(v);
  consoleNL();
}

void consolePrintLn(uint8_t v)
{
  consolePrint(v);
  consoleNL();
}

void consolePrintLn(long v)
{
  consolePrint(v);
  consoleNL();
}

void consolePrintLn(unsigned long v)
{
  consolePrint(v);
  consoleNL();
}

