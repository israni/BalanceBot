#include "mb_controller.h"
#include "mb_defs.h"

/*******************************************************************************
* int mb_initialize()
*
* this initializes all the LQG controllers from the configuration file
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/
//#define SAMPLE_RATE_HZ 100

float ticks_to_rps(int ticks){
    return (((ticks)/ENCODER_RES)*SAMPLE_RATE_HZ)/GEAR_RATIO;
}

float rps_to_mps(float rps){
    return rps*3.14159*WHEEL_DIAMETER;
}

float ticks_to_meters(int ticks){
    return ((ticks/ENCODER_RES)/GEAR_RATIO)*3.14159*WHEEL_DIAMETER;
}

float limit_error(float error, float limit){
    //limit error to a reasonable range
    if(fabs(error) > limit) {
        error = ((error > 0) - (error < 0)) * limit;
    }
    return error;
}

int mb_initialize_controller(){

    mb_load_controller_config();
    
    left_pid = PID_Init(
        left_pid_params.kp,
        left_pid_params.ki,
        left_pid_params.kd,
        left_pid_params.dFilterHz,
        SAMPLE_RATE_HZ
        //150.0
        );

    right_pid = PID_Init(
        right_pid_params.kp,
        right_pid_params.ki,
        right_pid_params.kd,
        right_pid_params.dFilterHz,
        SAMPLE_RATE_HZ
        //250.0
        );

    angle_pid = PID_Init(
        angle_pid_params.kp,
        angle_pid_params.ki,
        angle_pid_params.kd,
        angle_pid_params.dFilterHz,
        SAMPLE_RATE_HZ
        //40.0
        );

    position_pid = PID_Init(
        position_pid_params.kp,
        position_pid_params.ki,
        position_pid_params.kd,
        position_pid_params.dFilterHz,
        SAMPLE_RATE_HZ
        );
   
    PID_SetOutputLimits(left_pid,-1.0, 1.0);
    PID_SetOutputLimits(right_pid,-1.0, 1.0);
    PID_SetOutputLimits(angle_pid,-10.0,10.0); //no meaningful limits
    PID_SetOutputLimits(position_pid,-0.174,0.174);

    return 0;
}

/*******************************************************************************
* int mb_load_controller_config()
*
* this provides a basic configuration load routine
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/


int mb_load_controller_config(){
    FILE* file = fopen(CFG_PATH, "r");
    if (file == NULL){
        printf("Error opening pid.cfg\n");
    }

    fscanf(file, "%f %f %f %f", 
        &left_pid_params.kp,
        &left_pid_params.ki,
        &left_pid_params.kd,
        &left_pid_params.dFilterHz
        );

    fscanf(file, "%f %f %f %f",
        &right_pid_params.kp,
        &right_pid_params.ki,
        &right_pid_params.kd,
        &right_pid_params.dFilterHz
        );

    fscanf(file, "%f %f %f %f",
        &angle_pid_params.kp,
        &angle_pid_params.ki,
        &angle_pid_params.kd,
        &angle_pid_params.dFilterHz
        );

    fscanf(file, "%f %f %f %f",
        &position_pid_params.kp,
        &position_pid_params.ki,
        &position_pid_params.kd,
        &position_pid_params.dFilterHz
        );

    fclose(file);
    return 0;
}

/*******************************************************************************
* int mb_controller_update()
* 
* TODO: Write your cascaded PID controller here
* take inputs from the global mb_state
* write outputs to the global mb_state
*
* return 0 on success
*
*******************************************************************************/

int mb_controller_update(mb_state_t* mb_state){

	mb_state->left_cmd = LQG_Compute(left_lqg, mb_state->target, mb_state->alpha, -mb_state->left_encoder);
	mb_state->right_cmd = LQG_Compute(right_lqg, mb_state->target, mb_state->alpha, -mb_state->right_encoder);

    /*float error_position = mb_state->target_position - ticks_to_meters(mb_state->left_encoder_total+mb_state->right_encoder_total)/2.0;
    error_position = limit_error(error_position, 0.7536);
    float error_threshold_position = 0.7536;
    printf("%f\n", error_position);
    PID_Compute(position_pid, error_position, error_threshold_position, 0);
    mb_state->target_angle = position_pid->pidOutput;
    //printf("%f\n", mb_state->target_angle);
    
    //float a = mb_state->alpha;
    //float target_offset = 0;//((a > 0) - (a < 0)) * 3.1415926; //this is a sign function using properties of c
    float error_angle = mb_state->target_angle - mb_state->alpha;
    //printf("%f\n", error_angle);

    float error_threshold_angle = 0.087*2;
    int resetI;
    if(fabs(error_angle) > error_threshold_angle){
        resetI = 1;
    } else { resetI = 0;}
    PID_Compute(angle_pid, error_angle, error_threshold_angle, resetI);

    mb_state->left_command_speed = angle_pid->pidOutput;
    mb_state->right_command_speed = angle_pid->pidOutput;

    //printf("left_command_speed: %f, %f\n",mb_state->left_command_speed,(((mb_state->left_encoder)/ENCODER_RES)*SAMPLE_RATE_HZ)/GEAR_RATIO);
    //printf("right_command_speed: %f, %f\n",mb_state->right_command_speed,(((mb_state->right_encoder)/ENCODER_RES)*SAMPLE_RATE_HZ)/GEAR_RATIO);
    

    float error_left = mb_state->left_command_speed - ticks_to_rps(mb_state->left_encoder);
    float error_right = mb_state->right_command_speed - ticks_to_rps(mb_state->right_encoder);

    //limit error to a reasonable range
    error_left = limit_error(error_left, 3.0);
    error_right = limit_error(error_right, 3.0);

    //printf("%f\n",error_right);
    float error_threshold_speed = 3.0;
    PID_Compute(left_pid, error_left, error_threshold_speed, resetI);
    PID_Compute(right_pid, error_right, error_threshold_speed, resetI);

    mb_state->left_cmd = left_pid->pidOutput;
    mb_state->right_cmd = right_pid->pidOutput;

    //printf("duty cycle: %f\n", mb_state->left_cmd);
    */
    return 0;
}


/*******************************************************************************
* int mb_destroy_controller()
* 
* TODO: Free all resources associated with your controller
*
* return 0 on success
*
*******************************************************************************/

int mb_destroy_controller(){
	/*rc_free_matrix(lqg->A);
	rc_free_matrix(lqg->B);
	rc_free_vector(lqg->C);
	rc_free_vector(lqg->x_current);
	rc_free_vector(lqg->x_previous);
	rc_free_vector(lqg->temp1);
	rc_free_vector(lqg->temp1);
	rc_free_vector(lqg->inputVector);
	rc_free_vector(lqg->medianBuffer);*/
	//free(filteredSpeed);
	//free(positionSpeedFlag);
    /*free(left_pid);
    free(right_pid);
    free(angle_pid);
    free(position_pid);*/
    return 0;
}

int mb_initialize_controller_lqg(){

	left_lqg = LQG_Init();
	right_lqg = LQG_Init();
    mb_load_controller_config_lqg(); //to configure position or velocity control set left_lqg->positionSpeedFlag = 0 or 1, respectively before calling this function. 
	//right_lqg->positionSpeedFlag does NOTHING
    rc_print_matrix(left_lqg->A);
    rc_print_matrix(left_lqg->B);
    rc_print_vector(left_lqg->C);
    rc_print_matrix(right_lqg->A);
    rc_print_matrix(right_lqg->B);
    rc_print_vector(right_lqg->C);

	
    return 0;
}

/*******************************************************************************
* int mb_load_controller_config()
*
* this provides a basic configuration load routine
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/


int mb_load_controller_config_lqg(){
	FILE* file;
	FILE* file1;
	FILE* file2;
	file1 = fopen(CFG_PATH_POS, "r");
	file2 = fopen(CFG_PATH_SPD, "r");

	if (left_lqg->positionSpeedFlag) {
		file = file2;
	}
	else {
		file = file1;
	}
	
	
    if (file == NULL){
        printf("Error opening file\n");
    }
	//foo->bar is equivalent to (*foo).bar, i.e. it gets the member called bar from the struct that foo points to.
	//not sure if I need ampersands below
	//read A
	int ii;
	for (ii=0;ii<6;ii++) {
    fscanf(file, "%f %f %f %f %f %f", 
        &left_lqg-> A.d[ii][0],
        &left_lqg->A.d[ii][1],
        &left_lqg->A.d[ii][2],
        &left_lqg->A.d[ii][3],
		&left_lqg->A.d[ii][4],
		&left_lqg->A.d[ii][5]
        );
	}
	
	//read B
	for (ii=0;ii<6;ii++) {
    fscanf(file, "%f %f %f", 
        &left_lqg->B.d[ii][0],
        &left_lqg->B.d[ii][1],
        &left_lqg->B.d[ii][2]
        );
	}
	
	//read C
	fscanf(file, "%f %f %f %f %f %f", 
        &left_lqg->C.d[0],
        &left_lqg->C.d[1],
        &left_lqg->C.d[2],
		&left_lqg->C.d[3],
        &left_lqg->C.d[4],
        &left_lqg->C.d[5]
        );

	//copy matrices (A,B,C) from left_lqg to right_lqg
	//foo->bar is equivalent to (*foo).bar, i.e. it gets the member called bar from the struct that foo points to.
	rc_duplicate_matrix(left_lqg-> A, &(right_lqg-> A));
	rc_duplicate_matrix(left_lqg->B, &(right_lqg-> B));
	right_lqg->C.d[0] = left_lqg->C.d[0];
	right_lqg->C.d[1] = left_lqg->C.d[1];
	right_lqg->C.d[2] = left_lqg->C.d[2];
	right_lqg->C.d[3] = left_lqg->C.d[3];
	right_lqg->C.d[4] = left_lqg->C.d[4];
	right_lqg->C.d[5] = left_lqg->C.d[5];
    fclose(file);
    return 0;
}