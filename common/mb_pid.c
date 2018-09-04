/*******************************************************************************
* mb_pid->c
*
* TODO: implement these functions to build a generic PID controller
*       with a derivative term low pass filter
*
*******************************************************************************/

#include "mb_pid.h"
#include "mb_defs.h"

#define MAX_OUTPUT 1.0
#define MIN_OUTPUT -1.0
#define ITERM_MIN -1.0
#define ITERM_MAX 1.0

PID_t * PID_Init(float Kp, float Ki, float Kd, float dFilterHz, float updateHz) {
  // zero initial values
  PID_t* pid =  malloc(sizeof(PID_t));
  
  pid->kp = Kp;
  pid->ki = Ki;
  pid->kd = Kd;
  pid->dFilterHz = dFilterHz;
  pid->updateHz = updateHz;
  pid->pidInput = 0;
  pid->pidOutput = 0;
  pid->pTerm = 0;
  pid->iTerm = 0;
  pid->dTerm = 0;
  pid->prevInput = 0;
  pid->iTermMin = -1000*Kp; //change
  pid->iTermMax = Kp*1000; //change
  pid->outputMin = -1.0;//-10*Kp; //change
  pid->outputMax = 1.0;//10*Kp; //change
  pid->dFilter = rc_empty_filter();
  rc_butterworth_lowpass(&(pid->dFilter), 3, DT, dFilterHz*6.28); //low pass filter for DTerm
  pid->errorbuff = rc_empty_ringbuf();
  rc_alloc_ringbuf(&(pid->errorbuff), 3);
  rc_reset_ringbuf(&(pid->errorbuff));

  return pid;
}

float PID_Compute(PID_t* pid, float error, float threshold, int resetI) {

  float filtered_error = rc_march_filter(&(pid->dFilter), error);  //filter d term
  rc_insert_new_ringbuf_value(&(pid->errorbuff), filtered_error);
  float last_error = rc_get_ringbuf_value(&(pid->errorbuff), 1);

  //float error = rc_march_filter(&(pid->filter), error1);
  //printf("%f, %f\n",error);

  //set p term
	pid->pTerm = error * pid->kp;
  //printf("pTerm: %f\n",pid->pTerm);

  if(fabs(error) > threshold || resetI){
    pid->iTerm = 0;
  }

  //set i term
	if(pid->iTerm < pid->iTermMax && pid->iTerm > pid->iTermMin)
	{
		pid->iTerm = pid->iTerm + (error * pid->ki);
	}
  //printf("iTerm: %f, error: %f\n",pid->iTerm,error);

  //set d term
  pid->dTerm = (filtered_error - last_error) * pid->kd / DT;
  
  //printf("last error: %f\n",rc_get_ringbuf_value(&(pid->errorbuff), 1));

  pid->pidOutput = pid->pTerm + pid->iTerm + pid->dTerm;
	
  if(pid->pidOutput > pid->outputMax){
    pid->pidOutput = pid->outputMax;
  }
  if(pid->pidOutput < pid->outputMin){
    pid->pidOutput = pid->outputMin;
  }

  //pid->pidOutput = 0.5;
  //printf("%f\n",pid->pidOutput);

  return 0;
}

void PID_SetTunings(PID_t* pid, float Kp, float Ki, float Kd) {
  //scale gains by update rate in seconds for proper units
	pid->kp = Kp;
  	pid->ki = Ki;
  	pid->kd = Kd;
}

void PID_SetOutputLimits(PID_t* pid, float min, float max){
	pid->outputMin = min;
  	pid->outputMax = max;
}

void PID_SetIntegralLimits(PID_t* pid, float min, float max){
	pid->iTermMin = min;
  pid->iTermMax = max;
}

void PID_ResetIntegrator(PID_t* pid){
	pid->iTerm = 0;
}

void PID_SetDerivativeFilter(PID_t* pid, float filterHz){
	rc_butterworth_lowpass(&(pid->dFilter), 3, DT, filterHz*6.28); //low pass filter for DTerm
}

void PID_SetUpdateRate(PID_t* pid, float updateHz){
	pid->updateHz = updateHz;
}