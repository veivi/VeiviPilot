#include <AP_HAL/AP_HAL.h>
#include "Console.h"
#include <stdarg.h>

extern const AP_HAL::HAL& hal;

static void newline(void);
bool talk = true;

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

void consoleNotef(const char *s, ...)
{
  va_list argp;

  va_start(argp, s);
  consolePrint("// ");
  consolevPrintf(s, argp);
  va_end(argp);
}

void consoleNotefLn(const char *s, ...)
{
  va_list argp;

  va_start(argp, s);
  consolePrint("// ");
  consolevPrintf(s, argp);
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

void consolevPrintf(const char *s, va_list argp)
{
  if(talk)
#ifdef ARDUINO
    Serial.print(s);
#else
    hal.console->vprintf(s, argp);
#endif
}

void consolePrint(const char *s)
{
  if(talk)
#ifdef ARDUINO
    Serial.print(s);
#else
    hal.console->printf("%s", s);
#endif
}

void consolePrint(float v, int p)
{
  if(talk) {
#ifdef ARDUINO
    Serial.print(v, p);
#else
    char fmt[10];
    strcpy(fmt, "%. f");
    fmt[2] = '0' + p;
    hal.console->printf(fmt, (double) v);
#endif
  }
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
#ifdef ARDUINO
    Serial.print(v);
#else
    hal.console->printf("%d", v);
#endif
}

void consolePrint(unsigned long v)
{
#ifdef ARDUINO
    Serial.print(v);
#else
    hal.console->printf("%d", v);
#endif
}

void consolePrint(uint8_t v)
{
  consolePrint((unsigned int) v);
}

void newline(void)
{
  consolePrint("\n");
}

void consolePrintLn(const char *s)
{
  consolePrint(s);
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

