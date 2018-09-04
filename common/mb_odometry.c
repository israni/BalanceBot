/*******************************************************************************
* mb_odometry.c
*
* TODO: Implement these functions to add odometry and dead rekoning 
*
*******************************************************************************/

#include "../balancebot/balancebot.h"

#define DTHETA_THRESH 0.00001
#define BASE_WIDTH 0.223
#define FILTER_THRESHOLD 20

void mb_initialize_odometry(mb_odometry_t* mb_odometry, float x, float y, float theta){
	mb_odometry->x = 0;
	mb_odometry->y = 0;
	mb_odometry->theta = 0;

	rc_alloc_ringbuf(&encoderbuff_lo, 2);
  	rc_reset_ringbuf(&encoderbuff_lo);
  	rc_alloc_ringbuf(&encoderbuff_ro, 2);
  	rc_reset_ringbuf(&encoderbuff_ro);
  	rc_alloc_ringbuf(&gyrobuff, 2);
  	rc_reset_ringbuf(&gyrobuff);
}

void mb_update_odometry(mb_odometry_t* mb_odometry, mb_state_t* mb_state){

	int useGyro = 0;

	int left_encoder = rc_get_encoder_pos(1)*ENC_1_POL;
	int right_encoder = rc_get_encoder_pos(2)*ENC_2_POL;
	rc_insert_new_ringbuf_value(&gyrobuff, mb_state->theta);
	float gyro_delta_theta = rc_get_ringbuf_value(&gyrobuff, 0) - rc_get_ringbuf_value(&gyrobuff, 1);
	
	if(abs(left_encoder - rc_get_ringbuf_value(&encoderbuff_lo, 0))<FILTER_THRESHOLD || fabs(left_encoder) < 30){
		rc_insert_new_ringbuf_value(&encoderbuff_lo, left_encoder);
	}
	if (abs(right_encoder - rc_get_ringbuf_value(&encoderbuff_ro, 0))<FILTER_THRESHOLD || fabs(right_encoder) < 30){
		rc_insert_new_ringbuf_value(&encoderbuff_ro, right_encoder);
	}
	left_encoder = rc_get_ringbuf_value(&encoderbuff_lo, 0);
	right_encoder = rc_get_ringbuf_value(&encoderbuff_ro, 0);

	float delta_d = (float)(left_encoder + right_encoder)/2.0;
	delta_d = ((delta_d/ENCODER_RES)/GEAR_RATIO)*3.14159*WHEEL_DIAMETER;  //convert to meters

	float delta_theta;

	if(fabs(delta_d - gyro_delta_theta) > DTHETA_THRESH){
		useGyro = 1;
	}
	else{
		useGyro = 1; //always use gyro
	}

	if(useGyro){
		delta_theta = gyro_delta_theta;
	}
	else {
		delta_theta = (float)(right_encoder - left_encoder);
		delta_theta = (((delta_theta/ENCODER_RES)/GEAR_RATIO)*3.14159*WHEEL_DIAMETER)/BASE_WIDTH;
	}

	mb_odometry->theta = mb_odometry->theta + delta_theta;
	mb_odometry->theta = mb_clamp_radians(mb_odometry->theta);
	mb_odometry->x = mb_odometry->x + delta_d*cos(mb_odometry->theta);
	mb_odometry->y = mb_odometry->y + delta_d*sin(mb_odometry->theta);
}


float mb_clamp_radians(float angle){
	if(angle > PI){
		angle = angle - 2*PI; 
	} else if(angle < -PI){
		angle = angle + 2*PI;
	}
    return angle;
}