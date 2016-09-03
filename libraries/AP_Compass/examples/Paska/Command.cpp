#include <AP_Progmem/AP_Progmem.h>
#include "Command.h"
#include "NVState.h"

const struct Command commands[] PROGMEM = {
  { "name", c_name, e_string, &vpParam.name },
  { "max", c_max, e_angle, &vpParam.alphaMax },
  { "zl", c_zl, e_angle, &vpParam.alphaZeroLift },
  { "5048b_ref", c_5048b_ref, e_uint16, &vpParam.alphaRef },
  { "inner_pid_zn", c_inner_pid_zn,
    e_float, &vpParam.i_Ku_C, &vpParam.i_Tu },
  { "outer_p", c_outer_p, e_float, &vpParam.o_P },
  { "ff", c_ff, e_float, &vpParam.ff_A, &vpParam.ff_B },
  { "stabilizer_pid_zn", c_stabilizer_pid_zn,
    e_float, &vpParam.s_Ku_C, &vpParam.s_Tu },
  { "pusher_pid_zn", c_pusher_pid_zn,
    e_float, &vpParam.p_Ku_C, &vpParam.p_Tu },
  { "wl", c_wl, e_angle, &vpParam.wl_Limit },
  { "rmix", c_rmix, e_float, &vpParam.r_Mix },
  { "edefl", c_edefl, e_angle90, &vpParam.elevDefl },
  { "eneutral", c_eneutral, e_angle90, &vpParam.elevNeutral },
  { "takeoff", c_takeoff, e_percent, &vpParam.takeoffTrim },
  { "eservo", c_eservo, e_int8, &vpParam.servoElev },
  { "adefl", c_adefl, e_angle90, &vpParam.aileDefl },
  { "aneutral", c_aneutral, e_angle90, &vpParam.aileNeutral },
  { "aservo", c_aservo, e_int8, &vpParam.servoAile },
  { "rdefl", c_rdefl, e_angle90, &vpParam.rudderDefl },
  { "rneutral", c_rneutral, e_angle90, &vpParam.rudderNeutral },
  { "rservo", c_rservo, e_int8, &vpParam.servoRudder },
  { "fstep", c_flapstep, e_angle90, &vpParam.flapStep },
  { "fneutral", c_flapneutral,
    e_angle90, &vpParam.flapNeutral, &vpParam.flap2Neutral },
  { "fservo", c_fservo,
    e_int8, &vpParam.servoFlap, &vpParam.servoFlap2 },
  { "bdefl", c_bdefl, e_angle90, &vpParam.brakeDefl },
  { "bneutral", c_bneutral, e_angle90, &vpParam.brakeNeutral },
  { "bservo", c_bservo, e_int8, &vpParam.servoBrake },
  { "gservo", c_gservo, e_int8, &vpParam.servoGear },  
  { "ias", c_ias, e_float, &vpParam.iasMin },
  { "roll_k", c_roll_k, e_float, &vpParam.roll_C },
  { "servorate", c_servorate, e_float, &vpParam.servoRate },
  { "update", c_update },
  { "beep", c_beep },
  { "ping", c_ping },
  { "model", c_model },
  { "alpha", c_alpha },
  { "dumpz", c_dump },
  { "clear", c_clear },
  { "store", c_store },
  { "report", c_report },
  { "stop", c_stop },
  { "log", c_log },
  { "start", c_start },
  { "params", c_params },
  { "reset", c_reset },
  { "loop", c_gauge },
  { "gauge", c_gauge },
  { "stamp", c_stamp },
  { "backup", c_backup },
  { "arm", c_arm },
  { "disarm", c_disarm },
  { "test", c_test },
  { "talk", c_talk },
  { "rattle", c_rattle },
  { "defaults", c_defaults },
  { "atrim", c_atrim },
  { "etrim", c_etrim },
  { "rtrim", c_rtrim },
  { "rollrate", c_rollrate },
  { "pitchrate", c_pitchrate },
  { "calibrate", c_calibrate },  
  { "", c_invalid },  
};

