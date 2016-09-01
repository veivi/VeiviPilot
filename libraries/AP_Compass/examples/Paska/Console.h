#include <stdint.h>
#include <stdarg.h>
#include <AP_ProgMem/AP_ProgMem.h>

void consoleFlush();
void consoleCR();
void consoleNL();
void consolePrint(const char c);
void consoleTab(int i);
void consoleNote_P(const prog_char_t *s);
void consoleNote(const char *s);
void consoleNoteLn_P(const prog_char_t *s);
void consoleNoteLn(const char *s);
void consolePrint_P(const prog_char_t *s);
void consolePrint(const char *s);
void consolePrint(const char *s, int);
void consolePrint(float v, int p);
void consolePrint(float v);
void consolePrint(double v, int p);
void consolePrint(double v);
void consolePrint(int v);
void consolePrint(unsigned int v);
void consolePrint(long v);
void consolePrint(unsigned long v);
void consolePrint(uint8_t v);
void consolePrintLn_P(const prog_char_t *s);
void consolePrintLn(const char *s);
void consolePrintLn(float v);
void consolePrintLn(float v, int p);
void consolePrintLn(double v);
void consolePrintLn(double v, int p);
void consolePrintLn(int v);
void consolePrintLn(unsigned int v);
void consolePrintLn(uint8_t v);
void consolePrintLn(long v);
void consolePrintLn(unsigned long v);

#ifdef CONSOLE_PRIVATE_H
void consolevNotef(const char *s, va_list argp);
void consoleNotef(const char *s, ...);
void consoleNotefLn(const char *s, ...);
void consolevPrintf(const char *s, va_list argp);
void consolePrintf(const char *s, ...);
void consolePrintfLn(const char *s, ...);
#endif

extern uint32_t currentTime;
