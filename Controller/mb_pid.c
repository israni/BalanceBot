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
  pid->iTermMin = -Kp*1000; //change
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
  pid->dTerm = (error - last_error) * pid->kd / DT;
  
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

LQG_t * LQG_Init() {
  // initialize LQG controller for speed or position
  // positionSpeedFlag = 0   position controller
  // positionSpeedFlag = 1   speed controller
  LQG_t *lqg =  malloc(sizeof(LQG_t));
  
  lqg->A = rc_empty_matrix();
  lqg->B = rc_empty_matrix();
  lqg->C = rc_empty_vector();
  lqg->x_current = rc_empty_vector();
  lqg->x_previous = rc_empty_vector();
  lqg->temp1 = rc_empty_vector();
  lqg->temp2 = rc_empty_vector();
  lqg->inputVector = rc_empty_vector();
  lqg->medianBuffer = rc_empty_vector();
  
  rc_matrix_zeros( &(lqg->A), 6, 6);
  rc_matrix_zeros( &(lqg->B), 6, 3);
  rc_vector_zeros( &lqg->C, 6);
  rc_vector_zeros( &lqg->x_current, 6);
  rc_vector_zeros( &lqg->x_previous, 6);
  rc_vector_zeros( &lqg->temp1, 6);
  rc_vector_zeros( &lqg->temp2, 6);
  rc_vector_zeros( &lqg->inputVector, 3);
  rc_vector_zeros( &lqg->medianBuffer, 3);
  lqg->filteredSpeed = 0;
  
  return lqg;
}

float LQG_Compute(LQG_t* lqg, float command, float pitchMeasurement, int speedTickMeasurement) {    

  
  lqg->medianBuffer.d[2]=lqg->medianBuffer.d[1];
  lqg->medianBuffer.d[1]=lqg->medianBuffer.d[0];
  lqg->medianBuffer.d[0]=(float)speedTickMeasurement;
  
  float tempMax = -1000000.0;
  float tempMin = 1000000.0;
  int medianMaxIndex = 0;
  int medianMinIndex = 0;
  int ii;
  for (ii=0;ii<3;ii++) {
    if ( lqg->medianBuffer.d[ii] > tempMax) {
      tempMax = lqg->medianBuffer.d[ii];
      medianMaxIndex = ii; //find maximum index
    }
    if ( lqg->medianBuffer.d[ii] < tempMin) {
      tempMin = lqg->medianBuffer.d[ii];
      medianMinIndex = ii; //find minimum index
    }
  }
  for (ii=0;ii<3;ii++) {
    if (ii != medianMaxIndex && ii != medianMinIndex) {
      lqg->filteredSpeed = lqg->medianBuffer.d[ii]*0.02566660665; //pick median and convert from ticks/0.01s to m/s of robot
    }
  }
  
  //speed to sensor = 20.4*48/100/2/pi/0.04
  //sensor to speed = 100*2*pi*0.04/20.4/48
  lqg->inputVector.d[0]=command; //position or velocity command, set positionSpeedFlag appropriately
  lqg->inputVector.d[1]=pitchMeasurement; //pitch angle measurement in rads
  lqg->inputVector.d[2]=lqg->filteredSpeed; //filtered speed of wheel center in m/s
  
  rc_matrix_times_col_vec(lqg->A, lqg->x_previous, &lqg->temp1);  // A*x(k)
  rc_matrix_times_col_vec(lqg->B, lqg->inputVector, &lqg->temp2); // B*w(k)
  rc_vector_sum(lqg->temp1, lqg->temp2, &lqg->x_current);      // x(k+1) = A*x(k)+B*w(k)
  float torque = rc_vector_dot_product(lqg->C, lqg->x_current);  // u = C*x(k+1)
  float PWM = 4.869*torque; //torque converted to PWM
  
  if (fabs(PWM)>1.0) {
    PWM = (PWM>0)-(PWM<0);
    lqg->A.d[5][2] = 0.0;
    lqg->A.d[5][5] = 0.0;
    lqg->B.d[5][0] = 0.0;
  }
  else {
    lqg->A.d[5][2] = -1.0;
    lqg->A.d[5][5] = 1.0;
    lqg->B.d[5][0] = 1.0;
  }
  
  lqg->x_previous = lqg->x_current;

  return PWM;
}