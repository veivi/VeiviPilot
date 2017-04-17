#ifndef LOG_H
#define LOG_H

#include <stdint.h>
#include <stdbool.h>

#define NAME_LEN  8

struct LogInfo {
  uint16_t stamp;
  uint16_t test;
  int32_t length;
  char name[NAME_LEN];
};

typedef enum {  lc_alpha, 
                lc_dynpressure, 
		lc_accx,
		lc_accy,
		lc_accz,
                lc_roll, 
                lc_rollrate, 
                lc_pitch, 
                lc_pitchrate, 
                lc_heading, 
		lc_yawrate,
                lc_ailestick, 
                lc_elevstick,
		lc_thrstick,
		lc_rudstick,
                lc_aileron,
		lc_aileron_ff,
                lc_elevator,
		lc_elevator_ff,
		lc_rudder,
                lc_mode, 
                lc_target, 
                lc_target_pr, 
                lc_trim, 
                lc_gain, 
                lc_test, 
                lc_alt,
                lc_channels } ChannelId_t;

struct LogChannel {
  ChannelId_t ch;
  const char *name;
  float small, large;
  bool tick;
  uint16_t value;
};

#define TOKEN_MASK (1U<<15)
#define VALUE_MASK (TOKEN_MASK-1)
#define DELTA_MASK (VALUE_MASK>>1)
#define BYTE_MASK ((1<<8)-1)
#define ENTRY_TOKEN(t) (TOKEN_MASK | (t))
#define ENTRY_VALUE(v) (((uint16_t) v) & VALUE_MASK)
#define ENTRY_IS_TOKEN(e) ((e) & TOKEN_MASK)

typedef enum { t_stamp,
               t_mark, 
               t_channel = t_stamp + BYTE_MASK + 1,
               t_start = t_channel + BYTE_MASK + 1, 
               t_delta = t_stamp + DELTA_MASK + 1
            } LogToken_t;

extern struct LogChannel logChannels[];
extern uint8_t logTest;

#endif

