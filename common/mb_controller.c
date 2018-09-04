#include "mb_controller.h"
#include "mb_defs.h"

/*******************************************************************************
* int mb_initialize()
*
* this initializes all the PID controllers from the configuration file
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/

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
        );

    right_pid = PID_Init(
        right_pid_params.kp,
        right_pid_params.ki,
        right_pid_params.kd,
        right_pid_params.dFilterHz,
        SAMPLE_RATE_HZ
        );

    angle_pid = PID_Init(
        angle_pid_params.kp,
        angle_pid_params.ki,
        angle_pid_params.kd,
        angle_pid_params.dFilterHz,
        SAMPLE_RATE_HZ
        );

    velocity_pid = PID_Init(
        velocity_pid_params.kp,
        velocity_pid_params.ki,
        velocity_pid_params.kd,
        velocity_pid_params.dFilterHz,
        SAMPLE_RATE_HZ
        );

    position_pid = PID_Init(
        position_pid_params.kp,
        position_pid_params.ki,
        position_pid_params.kd,
        position_pid_params.dFilterHz,
        SAMPLE_RATE_HZ
        );

    turn_pid = PID_Init(
        turn_pid_params.kp,
        turn_pid_params.ki,
        turn_pid_params.kd,
        turn_pid_params.dFilterHz,
        SAMPLE_RATE_HZ
        );
   
    PID_SetOutputLimits(left_pid,-1.0, 1.0);
    PID_SetOutputLimits(right_pid,-1.0, 1.0);
    PID_SetOutputLimits(angle_pid,-10.0,10.0); //no meaningful limits
    PID_SetOutputLimits(velocity_pid,-0.35,0.35);
    PID_SetOutputLimits(position_pid,-1.4,1.4);
    PID_SetOutputLimits(turn_pid,-2.0,2.0);

    rc_alloc_ringbuf(&commandbuff_pid, 2);
    rc_reset_ringbuf(&commandbuff_pid);

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
        &velocity_pid_params.kp,
        &velocity_pid_params.ki,
        &velocity_pid_params.kd,
        &velocity_pid_params.dFilterHz
        );

    fscanf(file, "%f %f %f %f",
        &position_pid_params.kp,
        &position_pid_params.ki,
        &position_pid_params.kd,
        &position_pid_params.dFilterHz
        );

    fscanf(file, "%f %f %f %f",
        &turn_pid_params.kp,
        &turn_pid_params.ki,
        &turn_pid_params.kd,
        &turn_pid_params.dFilterHz
        );

    fclose(file);
    return 0;
}

float mb_clamp_angle(float angle){
    if(angle > PI){
        angle = angle - 2*PI; 
    } else if(angle < -PI){
        angle = angle + 2*PI;
    }
    return angle;
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

int mb_controller_update(mb_state_t* mb_state, mb_setpoints_t* mb_setpoints, mb_odometry_t* mb_odometry){
    PID_SetOutputLimits(position_pid,-mb_state->max_speed,mb_state->max_speed);
    if(!mb_setpoints->manual_ctl){
        int sign;
        if(fabs(mb_state->los_angle - mb_odometry->theta)>1.57 && !mb_setpoints->square_btn && !mb_state->mazeflag){
            sign = -1;
        } else {
            sign = 1;
        }
        float error_position = sign*mb_state->range;
        PID_Compute(position_pid, error_position, 1000.0 ,0);
        mb_state->target_velocity = position_pid->pidOutput;

        float a = 0.01/(mb_state->delay+0.01);
        float clamp_target_velocity = mb_state->target_velocity;
        if(clamp_target_velocity > mb_state->max_speed){
            clamp_target_velocity = mb_state->max_speed;
        } else if(clamp_target_velocity < -mb_state->max_speed){
            clamp_target_velocity = -mb_state->max_speed;
        }
        float filtered_command = (1-a)*rc_get_ringbuf_value(&commandbuff_pid, 0) + a*clamp_target_velocity;
        rc_insert_new_ringbuf_value(&commandbuff_pid, filtered_command);
        mb_state->target_velocity = filtered_command;

        float error_threshold_theta = 10.0;
        float error_theta = mb_clamp_angle(mb_state->target_theta - mb_odometry->theta);
        PID_Compute(turn_pid, error_theta, error_threshold_theta, 0);
        mb_state->turn_offset = turn_pid->pidOutput;
    }
    
    float velocity = rps_to_mps(ticks_to_rps(mb_state->left_encoder+mb_state->right_encoder)/2.0);
    float error_velocity = mb_state->target_velocity - velocity;
    
    PID_Compute(velocity_pid, error_velocity, 5.0, 0);

    mb_state->target_angle = -velocity_pid->pidOutput;

    float error_angle = mb_state->target_angle - mb_state->alpha;

    float error_threshold_angle = 0.087*8;
    int resetI;
    if(fabs(error_angle) > error_threshold_angle){
        resetI = 1;
    } else { resetI = 0;}
    PID_Compute(angle_pid, error_angle, error_threshold_angle, resetI);

    mb_state->left_command_speed = angle_pid->pidOutput;
    mb_state->right_command_speed = angle_pid->pidOutput;

    float error_left = mb_state->left_command_speed-mb_state->turn_offset - ticks_to_rps(mb_state->left_encoder);
    float error_right = mb_state->right_command_speed+mb_state->turn_offset - ticks_to_rps(mb_state->right_encoder);

    //limit error to a reasonable range
    error_left = limit_error(error_left, 3.0);
    error_right = limit_error(error_right, 3.0);

    float error_threshold_speed = 3.0;
    PID_Compute(left_pid, error_left, error_threshold_speed, resetI);
    PID_Compute(right_pid, error_right, error_threshold_speed, resetI);

    mb_state->left_cmd = left_pid->pidOutput;
    mb_state->right_cmd = right_pid->pidOutput;
    
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
    free(left_pid);
    free(right_pid);
    free(angle_pid);
    free(position_pid);
    return 0;
}