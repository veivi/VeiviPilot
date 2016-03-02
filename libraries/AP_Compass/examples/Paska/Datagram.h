#include <stdint.h>

void datagramTxStart(uint8_t );
void datagramTxOutByte(const uint8_t c);
void datagramTxOut(const uint8_t *data, int l);
void datagramTxEnd(void);

#define DG_HEARTBEAT     1
#define DG_CONSOLE_OUT   2
#define DG_LOGDATA       3
#define DG_LOGINFO       4
#define DG_PARAMS        5
#define DG_READY         6


