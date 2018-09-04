#ifndef BB_H
#define BB_H

// usefulincludes is a collection of common system includes
#include <rc_usefulincludes.h> 
// main roboticscape API header
#include <roboticscape.h>
#include <stdlib.h>
#include <string.h>
#include "../common/mb_defs.h"
#include "../common/mb_structs.h"
#include "../common/mb_motors.h"
#include "../common/mb_pid.h"
#include "../common/mb_controller.h"
#include "../common/mb_odometry.h"

// global variables
rc_imu_data_t imu_data;
pthread_mutex_t state_mutex;
mb_state_t mb_state;
mb_setpoints_t mb_setpoints;
mb_odometry_t mb_odometry;

rc_ringbuf_t encoderbuff_l;
rc_ringbuf_t encoderbuff_r;

// functions
void balancebot_controller();

//threads
void* setpoint_control_loop(void* ptr);
void* printf_loop(void* ptr);

#endif