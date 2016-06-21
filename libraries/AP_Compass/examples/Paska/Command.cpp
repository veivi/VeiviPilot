#include <AP_Progmem/AP_Progmem.h>
#include "Command.h"
#include "NVState.h"

const struct Command commands[] PROGMEM = {
  { "name", c_name, e_string, &paramRecord.name },
  { "max", c_max, e_angle360, &paramRecord.alphaMax },
  { "5048b_ref", c_5048b_ref, e_uint16, &paramRecord.alphaRef },
  { "inner_pid_zn", c_inner_pid_zn,
    e_float, &paramRecord.i_Ku_C, &paramRecord.i_Tu },
  { "outer_p", c_outer_p, e_float, &paramRecord.o_P_C },
  { "ff", c_ff, e_float, &paramRecord.ff_A, &paramRecord.ff_B },
  { "stabilizer_pid_zn", c_stabilizer_pid_zn,
    e_float, &paramRecord.s_Ku_C, &paramRecord.s_Tu },
  { "rudder_pid_zn", c_rudder_pid_zn,
    e_float, &paramRecord.r_Ku, &paramRecord.r_Tu },
  { "wl", c_wl, e_float, &paramRecord.wl_Limit },
  { "rmix", c_rmix, e_float, &paramRecord.r_Mix },
  { "edefl", c_edefl, e_angle90, &paramRecord.elevDefl },
  { "eneutral", c_eneutral, e_angle90, &paramRecord.elevNeutral },
  { "eservo", c_eservo, e_int8, &paramRecord.servoElev },
  { "adefl", c_adefl, e_angle90, &paramRecord.aileDefl },
  { "aneutral", c_aneutral, e_angle90, &paramRecord.aileNeutral },
  { "aservo", c_aservo, e_int8, &paramRecord.servoAile },
  { "rdefl", c_rdefl, e_angle90, &paramRecord.rudderDefl },
  { "rneutral", c_rneutral, e_angle90, &paramRecord.rudderNeutral },
  { "rservo", c_rservo, e_int8, &paramRecord.servoRudder },
  { "fstep", c_flapstep, e_angle90, &paramRecord.flapStep },
  { "fneutral", c_flapneutral,
    e_angle90, &paramRecord.flapNeutral, &paramRecord.flap2Neutral },
  { "fservo", c_fservo,
    e_int8, &paramRecord.servoFlap, &paramRecord.servoFlap2 },
  { "bdefl", c_bdefl, e_angle90, &paramRecord.brakeDefl },
  { "bneutral", c_bneutral, e_angle90, &paramRecord.brakeNeutral },
  { "bservo", c_bservo, e_int8, &paramRecord.servoBrake },
  { "gservo", c_gservo, e_int8, &paramRecord.servoGear },  
  { "pid", c_pid, e_int8, &paramRecord.c_PID },  
  { "ias", c_ias, e_float, &paramRecord.ias_Low, &paramRecord.ias_High },
  { "servorate", c_servorate, e_float, &paramRecord.servoRate },
  { "model", c_model },
  { "zero", c_zero },
  { "alpha", c_alpha },
  { "dump", c_dump },
  { "dumpz", c_dumpz },
  { "clear", c_clear },
  { "store", c_store },
  { "report", c_report },
  { "stop", c_stop },
  { "cycle", c_cycle },
  { "start", c_start },
  { "params", c_params },
  { "reset", c_reset },
  { "loop", c_loop },
  { "stamp", c_stamp },
  { "backup", c_backup },
  { "echo", c_echo },
  { "rpm", c_rpm },
  { "arm", c_arm },
  { "disarm", c_disarm },
  { "test", c_test },
  { "talk", c_talk },
  { "rattle", c_rattle },
  { "defaults", c_defaults },
  { "atrim", c_atrim },
  { "etrim", c_etrim },
  { "rtrim", c_rtrim },
  { "calibrate", c_calibrate },  
  { "", c_invalid },  
};

