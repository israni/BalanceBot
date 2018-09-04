/*******************************************************************************
* mb_motors.c
*
* Control up to 2 DC motordrivers
*
*******************************************************************************/
#include "mb_motors.h"

// global initialized flag
int mb_motors_initialized = 0;

/*******************************************************************************
* int mb_initialize_motors()
* 
* set up gpio assignments, pwm channels, and make sure motors are left off.
* GPIO exporting must be done previously with simple_init_gpio()
* initialized motors should start disabled
*******************************************************************************/
int mb_initialize_motors(){

    //#ifdef DEBUG
    printf("Initializing: PWM\n");
    //#endif

    if(rc_pwm_init(1,DEFAULT_PWM_FREQ)){
        printf("ERROR: failed to initialize hrpwm1\n");
        return -1;
    }

    if(rc_pwm_init(2,DEFAULT_PWM_FREQ)){
        printf("ERROR: failed to initialize hrpwm2\n");
        return -1;
    }


    mb_motors_initialized = 1;
    //#ifdef DEBUG
    printf("motors initialized...\n");
    //#endif
    mb_disable_motors();
    return 0;
}


/*******************************************************************************
* mb_enable_motors()
* 
* turns on the standby pin to enable the h-bridge ICs
* returns 0 on success, -1 on failure
*******************************************************************************/
int mb_enable_motors(){
    if(mb_motors_initialized==0){
        printf("ERROR: trying to enable motors before they have been initialized\n");
        return -1;
    }
    //_set_motor_free_spin_all();
    rc_gpio_set_value_mmap(MOT_EN , 0);
    return 0;
}

/*******************************************************************************
* int mb_disable_motors()
* 
* disables PWM output signals and
* turns off the enable pin to disable the h-bridge ICs
* returns 0 on success
*******************************************************************************/
int mb_disable_motors(){
    if(mb_motors_initialized==0){
        printf("ERROR: trying to disable motors before they have been initialized\n");
        return -1;
    }
    rc_gpio_set_value_mmap((MOT_EN) , 1);
    //rc_gpio_set_value_mmap((MDIR1)  , 1);
    //rc_gpio_set_value_mmap((MDIR2)  , 1);
    return 0;
}

/*******************************************************************************
* int mb_set_motor(int motor, float duty)
* 
* set a motor direction and power
* motor is from 1 to 2, duty is from -1.0 to +1.0
* returns 0 on success
*******************************************************************************/
int mb_set_motor(int motor, float duty){
    if(mb_motors_initialized==0){
        printf("ERROR: trying to rc_set_motor_all before they have been initialized\n");
        return -1;
    }
    int direction = 1;

    //rc_pwm_init(1,DEFAULT_PWM_FREQ);
    //rc_pwm_init(2,DEFAULT_PWM_FREQ);

    //printf("set motors\n");

    if(duty < 0)
    {
        duty = fabs(duty);
        direction = -1;
    }


    if(motor == 1)
    {
        //printf("motor1 duty: %f\n",duty);
        direction = direction * MOT_1_POL;
        if(direction == -1){
            direction = 0;
        }
      rc_gpio_set_value_mmap(MDIR1, direction);
      rc_pwm_set_duty_mmap(1, 'A', duty);
    }
    if(motor == 2)
    {
        //printf("motor2 duty %f\n",duty);
        direction = direction * MOT_2_POL;
        if(direction == -1){
            direction = 0;
        }
      rc_gpio_set_value_mmap(MDIR2, direction);
      rc_pwm_set_duty_mmap(1, 'B', duty);
    }
    
    return 0;
}

/*******************************************************************************
* int mb_set_motor_all(float duty)
* 
* applies the same duty cycle argument to both motors
*******************************************************************************/
int mb_set_motor_all(float duty){
    if(mb_motors_initialized==0){
        printf("ERROR: trying to rc_set_motor_all before they have been initialized\n");
        return -1;
    }
    mb_set_motor(LEFT_MOTOR, duty);
    mb_set_motor(RIGHT_MOTOR, duty);
    return 0;
}
