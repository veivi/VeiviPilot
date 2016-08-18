#include "Log.h"

// Must match the order with logChannelId_t declaration!!

struct LogChannel logChannels[] = {
   { lc_alpha, "ALPH", -180, 180, true },
   { lc_dynpressure, "PRES", -100, 10000 },
   { lc_accx, "ACCX", -150, 150 },
   { lc_accy, "ACCY", -150, 150 },
   { lc_accz, "ACCZ", -150, 150 },
   { lc_roll, "ROLL", -180, 180 },
   { lc_rollrate, "RRTE", -360, 360 },
   { lc_pitch, "PTCH", -90, 90 },
   { lc_pitchrate, "PRTE", -360, 360 },
   { lc_heading, "HEAD", 0, 360},
   { lc_yawrate, "YRTE", -360, 360},
   { lc_ailestick, "ASTK", -1, 1 },
   { lc_elevstick, "ESTK", -1, 1 },
   { lc_thrstick, "THRO", 0, 1 },
   { lc_rudstick, "RSTK", -1, 1 },
   { lc_aileron, "AILE", -1, 1 },
   { lc_elevator, "ELEV", -1, 1 },
   { lc_elevator_ff, "ELFF", -1, 1 },
   { lc_rudder, "RUDR", -1, 1 },
   { lc_mode, "MODE", 0, 1 },
   { lc_target, "TARG", -180, 180 },
   { lc_target_pr, "TPRT", -180, 180 },
   { lc_trim, "TRIM", -180, 180 },
   { lc_gain, "GAIN", 0, 50},
   { lc_test, "TEST", 0, 255},
   { lc_alt, "ALTI", -10, 300 } };

