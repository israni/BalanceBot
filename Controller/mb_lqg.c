/*******************************************************************************
* mb_lqg.c
*
*
*******************************************************************************/

#include "mb_lqg.h"
//#include "../../libraries/rc_usefulincludes.h"
//#include "../../libraries/roboticscape.h"

#define MAX_OUTPUT 1.0
#define MIN_OUTPUT -1.0
#define ITERM_MIN -1.0
#define ITERM_MAX 1.0

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
	rc_vector_sum(lqg->temp1, lqg->temp2, &lqg->x_current);		   // x(k+1) = A*x(k)+B*w(k)
	float torque = rc_vector_dot_product(lqg->C, lqg->x_current);  // u = C*x(k+1)
	float PWM = 4.869*torque; //torque converted to PWM
	
	if (fabs(PWM)>1.0) {
		PWM = (PWM>0)-(PWM<0);
	}
	
	lqg->x_previous = lqg->x_current;

  return PWM;
}









