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
#include <lcm/lcm.h>
#include "../lcmtypes/balancebot_msg_t.h"
#include "../lcmtypes/balancebot_gate_t.h"
#include "../lcmtypes/pose_xyt_t.h"

// global variables
rc_imu_data_t imu_data;
pthread_mutex_t state_mutex;
mb_state_t mb_state;
mb_setpoints_t mb_setpoints;
mb_odometry_t mb_odometry;

rc_ringbuf_t encoderbuff_l;
rc_ringbuf_t encoderbuff_r;
rc_ringbuf_t encoderbuff_l1;
rc_ringbuf_t encoderbuff_r1;
rc_ringbuf_t commandbuff;

// functions
void balancebot_controller();

//threads
void* setpoint_control_loop(void* ptr);
void* printf_loop(void* ptr);

// variable declarations
lcm_t * lcm;
balancebot_msg_t bb_msg;
pthread_mutex_t msg_mutex;

// forward declaration of functions
void* lcm_subscribe_loop(void* ptr);
void* printf_loop(void* ptr);
void beaglebone_message_handler(const lcm_recv_buf_t* rbuf,
                                const char* channel,
                                const balancebot_msg_t* msg,
                                void* userdata);

int saveGoals();


int generatePoints();
void update_obstacles();
float potential(float position_robot_guess_x, float position_robot_guess_y,float position_goal_x, float position_goal_y);
float distance(float x1,float y1,float x2,float y2);
void getDirection(float position_goal_x, float position_goal_y);


#endif