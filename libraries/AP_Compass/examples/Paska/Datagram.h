#include <stdint.h>

void datagramStart(uint8_t );
void datagramOut(const uint8_t c);
void datagramOut(const uint8_t *data, int l);
void datagramEnd(void);

#define DG_HEARTBEAT     1
#define DG_CONSOLE_OUT   2
#define DG_LOG           3
#define DG_STAMP         4


