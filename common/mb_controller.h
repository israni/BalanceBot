#ifndef MB_CONTROLLER_H
#define MB_CONTROLLER_H

#include "mb_pid.h"
#include "mb_structs.h"
#define CFG_PATH "pid.cfg"

float ticks_to_rps(int ticks);
float limit_error(float error, float limit);
int mb_initialize_controller();
int mb_load_controller_config();
int mb_controller_update(mb_state_t* mb_state, mb_setpoints_t* mb_setpoints, mb_odometry_t* mb_odometry);
int mb_destroy_controller();

PID_t * left_pid;
PID_t * right_pid;
PID_t * angle_pid;
PID_t * velocity_pid;
PID_t * position_pid;
PID_t * turn_pid;

pid_parameters_t left_pid_params;
pid_parameters_t right_pid_params;
pid_parameters_t angle_pid_params;
pid_parameters_t velocity_pid_params;
pid_parameters_t position_pid_params;
pid_parameters_t turn_pid_params;

rc_ringbuf_t commandbuff_pid;

#endif

