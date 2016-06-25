#ifndef COMMAND_H
#define COMMAND_H

#include <stdint.h>

#define MAX_NAME_LEN 24
#define MAX_VARS 10

typedef enum {
  c_invalid,
  c_5048b_ref,
  c_ezero,
  c_azero,
  c_rzero,
  c_adefl,
  c_edefl,
  c_clear,
  c_dump,
  c_max,
  c_zero,
  c_eneutral,
  c_aneutral,
  c_store,
  c_report,
  c_stop,
  c_cycle,
  c_read,
  c_write,
  c_start,
  c_params,
  c_reset,
  c_loop,
  c_stamp,
  c_model,
  c_alpha,
  c_flapneutral,
  c_flapstep,
  c_backup,
  c_echo,
  c_bdefl,
  c_bneutral,
  c_rdefl,
  c_rneutral,
  c_rpm,
  c_baud,
  c_dumpz,
  c_stabilizer_pid_zn,
  c_outer_p,
  c_rudder_pid_zn,
  c_rmix,
  c_rattle,
  c_inner_pid_zn,
  c_arm,
  c_disarm,
  c_test,
  c_talk,
  c_defaults,
  c_aservo,
  c_eservo,
  c_fservo,
  c_rservo,
  c_gservo,
  c_bservo,
  c_name,
  c_ff,
  c_pid,
  c_wl,
  c_ias,
  c_roll_k,
  c_rollrate,
  c_atrim,
  c_etrim,
  c_rtrim,
  c_servorate,
  c_calibrate
} token_t;

typedef enum
  { e_int8, e_uint16, e_angle90, e_angle360, e_float, e_string } varType_t;

struct Command {
  char name[MAX_NAME_LEN];
  token_t token;
  varType_t varType;
  void *var[MAX_VARS];
};

extern const struct Command commands[];

#endif
