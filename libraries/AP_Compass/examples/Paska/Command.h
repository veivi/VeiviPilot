#ifndef COMMAND_H
#define COMMAND_H

#include <stdint.h>

#define MAX_NAME_LEN 24
#define MAX_VARS 10

typedef enum {
  c_invalid,
  c_5048b_ref,
  
  c_ezero,
  c_eneutral,
  c_eservo,
  c_edefl,
  c_azero,
  c_aneutral,
  c_aservo,
  c_adefl,
  c_rzero,
  c_rneutral,
  c_rservo,
  c_rdefl,
  c_sneutral,
  c_sservo,
  c_sdefl,
  c_bneutral,
  c_bservo,
  c_bdefl,
  c_flapneutral,
  c_fservo,
  c_flapstep,
  c_gservo,
  c_weight,
  c_thrust,
  c_elevon,
  c_vtail,
  c_margin,
  c_slope,
  c_tservo,
  c_at_zn,
  c_offset,

  c_dump,
  c_max,
  c_virtual,
  c_trim,
  c_stall,
  c_peak,
  c_zl,
  c_col_ab,
  c_col_max,
  c_store,
  c_report,
  c_start,
  c_stop,
  c_log,
  c_params,
  c_reset,
  c_clear,
  c_init,
  c_gauge,
  c_stamp,
  c_model,
  c_alpha,
  c_backup,
  c_stabilizer_pid_zn,
  c_inner_pid_zn,
  c_outer_p,
  c_pusher_pid_zn,
  c_rmix,
  c_rattle,
  c_arm,
  c_disarm,
  c_test,
  c_talk,
  c_defaults,
  c_name,
  c_ff,
  c_roll_k,
  c_rollrate,
  c_atrim,
  c_etrim,
  c_rtrim,
  c_servorate,
  c_calibrate,
  c_takeoff,
  c_beep,
  c_ping,
  c_update,
  c_tab_col,
  c_tab_elev,
  c_curve,
  c_climb,
  c_gear,
  c_delete
} token_t;

typedef enum
  { e_int8, e_uint16, e_angle, e_angle90, e_percent, e_float, e_string } varType_t;

struct Command {
  char name[MAX_NAME_LEN];
  token_t token;
  varType_t varType;
  void *var[MAX_VARS];
};

extern const struct Command commands[];

#endif
