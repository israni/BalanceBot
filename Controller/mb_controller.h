#ifndef MB_CONTROLLER_H
#define MB_CONTROLLER_H

#include "mb_pid.h"
#include "mb_structs.h"
#define CFG_PATH_POS "lqg_position.cfg"
#define CFG_PATH_SPD "lqg_speed.cfg"
#define CFG_PATH "pid.cfg"

float ticks_to_rps(int ticks);
float limit_error(float error, float limit);
int mb_initialize_controller();
int mb_load_controller_config();

int mb_initialize_controller_lqg();
int mb_load_controller_config_lqg();
int mb_controller_update(mb_state_t* mb_state);
int mb_destroy_controller();

PID_t * left_pid;
PID_t * right_pid;
PID_t * angle_pid;
PID_t * position_pid;

pid_parameters_t left_pid_params;
pid_parameters_t right_pid_params;
pid_parameters_t angle_pid_params;
pid_parameters_t position_pid_params;

LQG_t * left_lqg;
LQG_t * right_lqg;

#endif
