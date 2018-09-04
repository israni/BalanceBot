/*******************************************************************************
* mb_pid.c
*
* TODO: implement these functions to build a generic PID controller
*       with a derivative term low pass filter
*
*******************************************************************************/

#include "mb_pid.h"

#define MAX_OUTPUT 1.0
#define MIN_OUTPUT -1.0
#define ITERM_MIN -1.0
#define ITERM_MAX 1.0

PID_t * PID_Init(float Kp, float Ki, float Kd, float dFilterHz, float updateHz) {
  // zero initial values
  PID_t *pid =  malloc(sizeof(PID_t));
  return pid;
}

float PID_Compute(PID_t* pid, float error) {    
  return 0;
}

void PID_SetTunings(PID_t* pid, float Kp, float Ki, float Kd) {
  //scale gains by update rate in seconds for proper units
}

void PID_SetOutputLimits(PID_t* pid, float min, float max){
}

void PID_SetIntegralLimits(PID_t* pid, float min, float max){
}

void PID_ResetIntegrator(PID_t* pid){
}

void PID_SetDerivativeFilter(PID_t* pid, float dFilterHz){
}

void PID_SetUpdateRate(PID_t* pid, float updateHz){
}