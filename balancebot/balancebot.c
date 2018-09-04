/*******************************************************************************
* balancebot.c
*
* Main template code for the balanceBot
* 
*******************************************************************************/
#include "balancebot.h"

#define MEDIAN_WINDOW 3
#define END 999
#define     LCM_HZ              1000
#define     OPTITRACK_CHANNEL   "OPTITRACK_CHANNEL"
#define FILTER_THRESHOLD 20
#define NUMGATES 4

/*******************************************************************************
* int main() 
*
*******************************************************************************/
int main(){
	// always initialize cape library first
	if(rc_initialize()){
		fprintf(stderr,"ERROR: failed to initialize rc_initialize(), are you root?\n");
		return -1;
	}

	//set cpu freq to max performance
	rc_set_cpu_freq(FREQ_1000MHZ);

	// start printf_thread if running from a terminal
	// if it was started as a background process then don't bother
	if(isatty(fileno(stdout))){
		printf("starting print thread... \n");
		pthread_t  printf_thread;
		pthread_create(&printf_thread, NULL, printf_loop, (void*) NULL);
	}

	// start control thread
	printf("starting setpoint thread... \n");
	pthread_t  setpoint_control_thread;
	pthread_create(&setpoint_control_thread, NULL, setpoint_control_loop, (void*) NULL);

	//COPIED FROM TEST_LCM
	pose_xyt_t init_pose = {.utime=0, .x=0, .y=0, .theta=0}; 
    bb_msg.utime = init_pose.utime;
    bb_msg.pose = init_pose;
    bb_msg.gates = (balancebot_gate_t*)malloc(sizeof(balancebot_gate_t)*4);

    printf("Creating LCM backend.\n");
    // create mutex to ensure data security
    pthread_mutex_init(&msg_mutex, NULL);

	// create lcm object, subscriber thread
    lcm = lcm_create(NULL);
    pthread_t lcm_subscribe_thread;
    pthread_create(&lcm_subscribe_thread, NULL, lcm_subscribe_loop,
                    (void*) NULL);
    //END COPIED FROM LCM

	// set up IMU configuration
	printf("initializing imu... \n");
	rc_imu_config_t imu_config = rc_default_imu_config();
	imu_config.dmp_sample_rate = SAMPLE_RATE_HZ;
	imu_config.orientation = ORIENTATION_Z_UP;

	if(rc_initialize_imu_dmp(&imu_data, imu_config)){
		fprintf(stderr,"ERROR: can't talk to IMU! NOT Exiting.\n");
		//return -1;
	}

	//rc_nanosleep(5E9); // wait for imu to stabilize

	//initialize state mutex
    pthread_mutex_init(&state_mutex, NULL);

	//attach controller function to IMU interrupt
	printf("initializing controller...\n");
	mb_initialize_controller();

	printf("initializing motors...\n");
	mb_initialize_motors();

	printf("resetting encoders...\n");
	rc_set_encoder_pos(1, 0);
	rc_set_encoder_pos(2, 0);

	printf("initializing odometry...\n");
	mb_initialize_odometry(&mb_odometry, 0.0,0.0,0.0);

	saveGoals();
	mb_odometry.dtagf=0;
	mb_odometry.num_gates_crossed=0;

	printf("enabling motors...\n");
	mb_enable_motors();

	rc_alloc_ringbuf(&encoderbuff_l1, 2);
  	rc_reset_ringbuf(&encoderbuff_l1);
	rc_alloc_ringbuf(&encoderbuff_r1, 2);
  	rc_reset_ringbuf(&encoderbuff_r1);
	rc_alloc_ringbuf(&encoderbuff_l, MEDIAN_WINDOW);
  	rc_reset_ringbuf(&encoderbuff_l);
	rc_alloc_ringbuf(&encoderbuff_r, MEDIAN_WINDOW);
  	rc_reset_ringbuf(&encoderbuff_r);
  	rc_alloc_ringbuf(&commandbuff, 2);
  	rc_reset_ringbuf(&commandbuff);

	printf("attaching imu interupt...\n");
	rc_set_imu_interrupt_func(&balancebot_controller);

	printf("we are running!!!...\n");
	// done initializing so set state to RUNNING
	rc_set_state(RUNNING); 

	// Keep looping until state changes to EXITING
	while(rc_get_state()!=EXITING){

		// all the balancing is handled in the imu interupt function
		// other functions are handled in other threads
		// there is no need to do anything here but sleep

		// always sleep at some point
		usleep(100000);
	}
	
	// exit cleanly
	mb_disable_motors();
	rc_cleanup();
	 
	return 0;
}

int turn(float t){
	float stop_threshold = 0.01;
	mb_state.range = sqrt(pow(mb_state.target_x-mb_odometry.x,2)+pow(mb_state.target_y-mb_odometry.y,2));
	mb_state.target_theta = t;
	if(fabs(mb_state.target_theta - mb_odometry.theta) < stop_threshold){
		return 1;
	} else {
		return 0;
	}
}

int drive(){
	float lambda = 1.85;
	float range_saturation = 0.01;
	float stop_threshold = 0.05;
	float velocity = (mb_state.left_encoder+mb_state.right_encoder)/2.0;
	velocity = ((velocity/ENCODER_RES)/GEAR_RATIO)*3.14159*WHEEL_DIAMETER;
	mb_state.los_angle = atan2((mb_state.target_y-mb_odometry.y),(mb_state.target_x-mb_odometry.x));
	mb_state.range = sqrt(pow(mb_state.target_x-mb_odometry.x,2)+pow(mb_state.target_y-mb_odometry.y,2));

	if(mb_state.range > range_saturation){
		mb_state.target_theta = mb_state.target_theta + lambda*(velocity*sin(mb_state.los_angle-mb_odometry.theta)/mb_state.range);
	} else {
		mb_state.target_theta = mb_state.target_theta + lambda*(velocity*sin(mb_state.los_angle-mb_odometry.theta)/range_saturation);
	}

	if(mb_state.range < stop_threshold){
		return 1;
	} else {
		return 0;
	}
}

int drive_straight(float x, float y){
	mb_state.target_x = x;
	mb_state.target_y = y;
	if(mb_state.turnflag==0){
		mb_state.turnflag = turn(atan2((mb_state.target_y-mb_odometry.y),(mb_state.target_x-mb_odometry.x)));
	}
	else if(mb_state.turnflag==1){
		if(drive()){
			mb_state.turnflag = 2; //keep driving while flag is 1
		}
		else{
			mb_state.turnflag = 1;
		}
	}
	if(mb_state.turnflag==2){
		return 1; //routine is finished
	}
	else{
		return 0; //routine is not finished
	}
}

int square(){
	mb_state.max_speed = 0.5;
	mb_state.delay = 0.00;
	PID_SetTunings(position_pid,1.4,0,0);
	PID_SetTunings(velocity_pid,0.17,0.001,0.5); 

	mb_odometry.optix = bb_msg.pose.x; 
    mb_odometry.optiy = -bb_msg.pose.y;
    if(fabs(mb_odometry.theta) > 3.14159/2.0){
    	if(bb_msg.pose.theta > 0){
    		mb_odometry.optitheta = 3.14159 - bb_msg.pose.theta;
    	}
    	else{
    		mb_odometry.optitheta = -3.14159 - bb_msg.pose.theta;
    	}
    }
    else{
    	mb_odometry.optitheta = bb_msg.pose.theta;
    }
    printf("%f,%f,%f,%f,%f,%f\n",mb_odometry.x,mb_odometry.y,mb_odometry.theta,mb_odometry.optix,mb_odometry.optiy,mb_odometry.optitheta);

	if(mb_state.square_step==-1){
		mb_odometry.x = 0;
		mb_odometry.y = 0;
		mb_state.square_step = 0;
	}
	else if(mb_state.square_step==0){
		if(drive_straight(1.0, 0.0)){
			mb_state.turnflag = 0;
			mb_state.square_step = 1;
		} else {
			mb_state.square_step = 0;
		}
	} else if(mb_state.square_step==1){
		if(drive_straight(1.0, 1.0)){
			mb_state.turnflag = 0;
			mb_state.square_step = 2;
		} else {
			mb_state.square_step = 1;
		}
	} else if(mb_state.square_step==2){
		if(drive_straight(0.0, 1.0)){
			mb_state.turnflag = 0;
			mb_state.square_step = 3;
		} else {
			mb_state.square_step = 2;
		}
	} else if(mb_state.square_step==3){
		if(drive_straight(0.0, 0.0)){
			mb_state.turnflag = 0;
			mb_state.square_step = 0;
		} else {
			mb_state.square_step = 3;
		}
	} else if(mb_state.square_step==4){
		if(drive_straight(0.5, 0.0)){
			mb_state.turnflag = 0;
			mb_state.square_step = 0;
		} else {
			mb_state.square_step = 4;
		}
	} 
	if(mb_state.square_step==5){
		mb_state.range = 0;
		return 1;
	}
	else {
		return 0;
	}
}

int task4(){
	mb_state.max_speed = 0.7;
	mb_state.delay = 0.10;
	PID_SetTunings(position_pid,1.4,0,0);
	PID_SetTunings(velocity_pid,0.17,0.001,0.5); 
	float d = 0.5;
	float s = 0.1;
	mb_state.mazeflag = 1;
	if(mb_state.race_step==-1){
		mb_odometry.x = 0;
		mb_odometry.y = 0;
		mb_state.race_step = 0;
	}
	else if(mb_state.race_step==0){
		mb_state.target_x = 0.3+d;
		mb_state.target_y = 0.9;
		if(drive()){
			mb_state.race_step=1;
		}
	}
	else if(mb_state.race_step==1){
		mb_state.target_x = 0.0;
		mb_state.target_y = 0.9;
		if(drive()){
			mb_state.race_step=2;
		}
	}
	else if(mb_state.race_step==2){
		mb_state.target_x = -0.6+d;
		mb_state.target_y = 0.3;
		if(drive()){
			mb_state.race_step=3;
		}
	}
	else if(mb_state.race_step==3){
		mb_state.target_x = -0.9;
		mb_state.target_y = 0.3;
		if(drive()){
			mb_state.race_step=4;
		}
	}
	else if(mb_state.race_step==4){
		mb_state.target_x = -0.6+d;
		mb_state.target_y = 0.3;
		if(drive()){
			mb_state.race_step=5;
		}
	}
	else if(mb_state.race_step==5){
		mb_state.target_x = -0.6+s;
		mb_state.target_y = -0.9;
		if(drive()){
			mb_state.race_step=6;
		}
	}
	else if(mb_state.race_step==6){
		mb_state.target_x = -0.6-3*s;
		mb_state.target_y = -0.9;
		if(drive()){
			mb_state.race_step=7;
		}
	}
	else if(mb_state.race_step==7){
		mb_state.mazeflag = 0;
		mb_state.target_x = 0.6+d;
		mb_state.target_y = -0.6+2*s;
		if(drive()){
			mb_state.race_step=8;
		}
	}
	if(mb_state.race_step==8){
		mb_state.range = 0;
		return 1;
	}
	else {
		return 0;
	}
}

int dragrace(){
	PID_SetTunings(position_pid,1.0,0,0); 
	PID_SetTunings(velocity_pid,0.13,0.002,0.0); 
	mb_state.max_speed = 1.48; //1.5 should work
	mb_state.delay = 1.3; //1.27 works for one
	mb_state.target_x = 10.0;
	mb_state.target_y = 0;
	if(mb_state.dragrace_step==-1){
		mb_odometry.x = 0;
		mb_odometry.y = 0;
		mb_state.dragrace_step = 0;
	}
	else if(mb_state.dragrace_step==0){
		if(drive()){
			mb_state.dragrace_step = 1;
		}
		else{
			mb_state.dragrace_step = 0;
		}
	}
	if(mb_state.dragrace_step==0){
		return 0;
	}
	else{
		mb_state.target_x = 11.0;
		mb_state.target_y = 0;
		mb_state.target_velocity=0.0;
		mb_state.range=0.0;
		PID_SetTunings(position_pid,1.4,0,0);
		return 1;
	}
}

int task5(){
	PID_SetTunings(position_pid,2.0,0,0);
	PID_SetTunings(velocity_pid,0.17,0.001,0.5); 
	mb_state.max_speed = 0.5;
	mb_state.delay = 0.30;

	mb_odometry.optix = bb_msg.pose.x; 
    mb_odometry.optiy = bb_msg.pose.y;
    if(fabs(mb_odometry.theta) > 3.14159/2.0){
    	if(bb_msg.pose.theta > 0){
    		mb_odometry.optitheta = 3.14159 - bb_msg.pose.theta;
    	}
    	else{
    		mb_odometry.optitheta = -3.14159 - bb_msg.pose.theta;
    	}
    }
    else{
    	mb_odometry.optitheta = bb_msg.pose.theta;
    }

    mb_odometry.x = mb_odometry.optix;
    mb_odometry.y = -mb_odometry.optiy;

    generatePoints();
    update_obstacles();

    if(mb_odometry.dtagf==0){
		getDirection(mb_odometry.entry_x[mb_odometry.num_gates_crossed],mb_odometry.entry_y[mb_odometry.num_gates_crossed]);
		drive();
		//can change mb_odometry.x to mb_odometry.optix
		if(fabs(mb_state.target_x - mb_odometry.entry_x[mb_odometry.num_gates_crossed]) < 0.05 &&
			fabs(mb_state.target_y - mb_odometry.entry_y[mb_odometry.num_gates_crossed]) < 0.05){
			mb_odometry.dtagf = 1;
			mb_odometry.num_gates_crossed++;
		}
	}
	else
	{
		getDirection(mb_odometry.exit_x[mb_odometry.num_gates_crossed-1],mb_odometry.exit_y[mb_odometry.num_gates_crossed-1]);
		drive();
		if(fabs(mb_state.target_x - mb_odometry.exit_x[mb_odometry.num_gates_crossed-1]) < 0.05 &&
			fabs(mb_state.target_y - mb_odometry.exit_y[mb_odometry.num_gates_crossed-1]) < 0.05){
			
			if(mb_odometry.num_gates_crossed==4){
				mb_odometry.dtagf = 2;
			}
			else{
				mb_odometry.dtagf = 0;
			}
		}
	}
	if(mb_odometry.dtagf == 2){
		mb_state.range = 0;
		return 1;
	}
	else{
		return 0;
	}
}

int saveGoals(){
	int i;
	for(i=0;i<NUMGATES;i++){
		mb_odometry.leftpost_x[i] = bb_msg.gates[i+0].left_post[0];
        mb_odometry.leftpost_y[i] = -bb_msg.gates[i+0].left_post[1];
        mb_odometry.rightpost_x[i] = bb_msg.gates[i+0].right_post[0];
        mb_odometry.rightpost_y[i] = -bb_msg.gates[i+0].right_post[1];
	}
	return 1;
}

/*******************************************************************************
* void balancebot_controller()
*
* discrete-time balance controller operated off IMU interrupt
* Called at SAMPLE_RATE_HZ
*
* TODO: You must implement this function to keep the balancebot balanced
* 
*
*******************************************************************************/
void balancebot_controller(){

	//lock state mutex
	pthread_mutex_lock(&state_mutex);
	// Read IMU
	mb_state.alpha = imu_data.dmp_TaitBryan[TB_PITCH_X];
	mb_state.dtheta = mb_state.theta - imu_data.dmp_TaitBryan[TB_YAW_Z];
	mb_state.theta = imu_data.dmp_TaitBryan[TB_YAW_Z];

	int left_encoder = rc_get_encoder_pos(1);
	int right_encoder = rc_get_encoder_pos(2);
	
	if(abs(left_encoder - rc_get_ringbuf_value(&encoderbuff_l1, 0))<FILTER_THRESHOLD || abs(left_encoder) < 30){
		rc_insert_new_ringbuf_value(&encoderbuff_l1, left_encoder);
	}
	if (abs(right_encoder - rc_get_ringbuf_value(&encoderbuff_r1, 0))<FILTER_THRESHOLD || abs(right_encoder) < 30){
		rc_insert_new_ringbuf_value(&encoderbuff_r1, right_encoder);
	}
	int filtered_speed_l = rc_get_ringbuf_value(&encoderbuff_l1, 0);
	int filtered_speed_r = rc_get_ringbuf_value(&encoderbuff_r1, 0);

	mb_state.left_encoder = ENC_1_POL * filtered_speed_l;
    mb_state.right_encoder = ENC_2_POL * filtered_speed_r;
    mb_state.left_encoder_total += ENC_1_POL * filtered_speed_l;
    mb_state.right_encoder_total += ENC_2_POL * filtered_speed_r;

    // Update odometry 
    mb_update_odometry(&mb_odometry, &mb_state);

    // Calculate controller outputs
    mb_controller_update(&mb_state, &mb_setpoints, &mb_odometry);

    //unlock state mutex
    pthread_mutex_unlock(&state_mutex);

    // reset encoders to 0
    rc_set_encoder_pos(1, 0);
    rc_set_encoder_pos(2, 0);
    
    if(!mb_setpoints.manual_ctl){
    	
    	if(mb_setpoints.square_btn){
    		square();
    	}
    	else if(mb_setpoints.dragrace_btn){
    		dragrace();
    		mb_state.mazeflag = 0;
    	}
    	else{
    		task4();
    	}
    	mb_set_motor(RIGHT_MOTOR, mb_state.right_cmd);
   		mb_set_motor(LEFT_MOTOR, mb_state.left_cmd);
   	}

    if(mb_setpoints.manual_ctl){
    	mb_state.max_speed = 0.6;
		mb_state.delay = 0.2;
    	float a = 0.01/(0.2+0.01);
    	float filtered_command = (1-a)*rc_get_ringbuf_value(&commandbuff, 0) + a*rc_get_dsm_ch_normalized(3)*1.45;
    	rc_insert_new_ringbuf_value(&commandbuff, filtered_command);
    	mb_state.target_velocity = filtered_command;
    	mb_state.turn_offset = rc_get_dsm_ch_normalized(4)*2;
    	mb_set_motor(RIGHT_MOTOR, mb_state.right_cmd);
   		mb_set_motor(LEFT_MOTOR, mb_state.left_cmd);
   	}

}


/*******************************************************************************
*  setpoint_control_loop()
*
*  sets current setpoints based on dsm radio data, odometry, and Optitrak
*
*
*******************************************************************************/
void* setpoint_control_loop(void* ptr){

	// start dsm listener for radio control
	rc_initialize_dsm();

	while(1){
		if (rc_is_new_dsm_data()) {

			//mb_state.target_angle = rc_get_dsm_ch_normalized(1)/3.0;
	 		
			// TODO: Handle the DSM data from the Spektrum radio reciever
			// You may also implement switching between manual and autonomous mode
			// using channel 5 of the DSM data.

		//mb_setpoints.fwd_velocity = FWD_VEL_SENSITIVITY * rc_get_dsm_ch_normalized(3);
		//mb_setpoints.turn_velocity = TURN_VEL_SENSITIVITY * rc_get_dsm_ch_normalized(4);
		
		if(rc_get_dsm_ch_normalized(5) > 0.0){
			mb_setpoints.manual_ctl = 1;
			mb_setpoints.square_btn = 0;
			mb_setpoints.dragrace_btn = 0;
			mb_state.square_step = -1;
			mb_state.dragrace_step = -1;
			mb_state.race_step = -1;
		}
		else{
			mb_setpoints.manual_ctl = 0;
		}

		if(rc_get_dsm_ch_normalized(6) < -0.8){
			mb_setpoints.square_btn = 1;
		}
		else if(rc_get_dsm_ch_normalized(6) > 0.8){
			mb_setpoints.dragrace_btn = 1;
		}
		else{
			mb_setpoints.square_btn = 0;
			mb_setpoints.dragrace_btn = 0;
			mb_state.square_step = -1;
			mb_state.dragrace_step = -1;
		}

	 	}
	 	usleep(1000000 / RC_CTL_HZ);
	}
}




/*******************************************************************************
* printf_loop() 
*
* prints diagnostics to console
* this only gets started if executing from terminal
*
* TODO: Add other data to help you tune/debug your code
*******************************************************************************/
// define loop for printing information
void* printf_loop(void* ptr){

    //printf("\nPose (X,Y,Z) - # Gates - Next Gate (L-(X,Y), R-(X,Y)) : \n");
    while(rc_get_state()!=EXITING){
        /*printf("\r");
        printf("(%4.2f, %4.2f, %4.2f) - %2d - ((%4.2f, %4.2f), (%4.2f, %4.2f))",
                    bb_msg.pose.x, 
                    bb_msg.pose.y, 
                    bb_msg.pose.theta,
                    bb_msg.num_gates,
                    bb_msg.gates[1].left_post[0],
                    bb_msg.gates[1].left_post[1],
                    bb_msg.gates[1].right_post[0],
                    bb_msg.gates[1].right_post[1]);
        fflush(stdout);*/
        usleep(1000000 / PRINTF_HZ);
    }
    return NULL;
}

// define lcm subscription thread
void *lcm_subscribe_loop(void *data){
    // pass in lcm object instance, channel from which to read from
    // function to call when data receiver over the channel,
    // and the lcm instance again?
    balancebot_msg_t_subscribe(lcm,
                               OPTITRACK_CHANNEL,
                               beaglebone_message_handler,
                               lcm);

    while(1){
        // define a timeout (for erroring out) and the delay time
        lcm_handle_timeout(lcm, 1);
        usleep(1000000 / LCM_HZ);
    }
    lcm_destroy(lcm);
    return 0;
}
// function called by lcm subscription loop on channel OPTITRACK_CHANNEL
void beaglebone_message_handler(const lcm_recv_buf_t* rbuf,
                                const char* channel,
                                const balancebot_msg_t* msg,
                                void* userdata){

    // lock the mutex
    pthread_mutex_lock(&msg_mutex);
    // write data from message to local variable of same type

    // NOTE: updated to use message specific function msg_t_copy(msg)
    // Bugs found with individually copying data
    bb_msg = *(balancebot_msg_t_copy(msg));

    pthread_mutex_unlock(&msg_mutex);
}

//This sets for each goal:
//mb_odometry.obstacle_xy
//mb_odometry.obstacle_radius
//mb_odometry.entry_xy
//mb_odometry.exit_xy
int generatePoints(){
    int i;
    float x1;
    float y1;
    float x2;
    float y2;
    float goal_offset;
    float gate_distance;
    float D;

    for(i=0;i<NUMGATES;i++){
        x1 = mb_odometry.leftpost_x[i];
        y1 = mb_odometry.leftpost_y[i];
        x2 = mb_odometry.rightpost_x[i];
        y2 = mb_odometry.rightpost_y[i];

        mb_odometry.obstacle_x[i] = (x1+x2)/2.0;
        mb_odometry.obstacle_y[i] = (y1+y2)/2.0;
        gate_distance = sqrt(pow(x2-x1,2)+pow(y2-y1,2));
        mb_odometry.obstacle_radius[i] = (gate_distance/2.0)+0.15;

        D = mb_odometry.obstacle_radius[i];
        goal_offset = D + 0.05;
        mb_odometry.entry_x[i] = mb_odometry.obstacle_x[i]-(goal_offset/gate_distance)*(y1-y2);
        mb_odometry.entry_y[i] = mb_odometry.obstacle_y[i]-(goal_offset/gate_distance)*(x2-x1);
        mb_odometry.exit_x[i] = mb_odometry.obstacle_x[i]+(goal_offset/gate_distance)*(y1-y2);
        mb_odometry.exit_y[i] = mb_odometry.obstacle_y[i]+(goal_offset/gate_distance)*(x2-x1);
    }
    return 1;
}

//This turns a large gate obstacle into two smaller post obstacles once a gate is opened
void update_obstacles(){

    int i;
    float post_radius = 0.12;
    switch(mb_odometry.num_gates_crossed){
        case 0:

            for (i=0;i<NUMGATES*2;i++){
                if(i<4)
                    {
                    mb_odometry.q_obs_x[i] = mb_odometry.obstacle_x[i];
                    mb_odometry.q_obs_y[i] = mb_odometry.obstacle_y[i];
                    mb_odometry.q_obs_r[i] = mb_odometry.obstacle_radius[i];
                }
                else{
                    mb_odometry.q_obs_x[i]= 100000;
                    mb_odometry.q_obs_y[i]= 100000;
                    mb_odometry.q_obs_r[i] = 100000;
                }
            }

            break;

        case 1:
            
            for (i=0;i<NUMGATES*2;i++){
                
                if(i<2)
                {
                    mb_odometry.q_obs_x[i]=mb_odometry.leftpost_x[i];
                    mb_odometry.q_obs_x[i+1]=mb_odometry.rightpost_x[i];
                    mb_odometry.q_obs_y[i]=mb_odometry.leftpost_y[i];
                    mb_odometry.q_obs_y[i+1]=mb_odometry.rightpost_y[i];
                    mb_odometry.q_obs_r[i]= post_radius;
                    mb_odometry.q_obs_r[i+1]= post_radius;
                    i++;
                }
                
                else if (i>=2 && i<5)
                    {
                    mb_odometry.q_obs_x[i] = mb_odometry.obstacle_x[i-1];
                    mb_odometry.q_obs_y[i] = mb_odometry.obstacle_y[i-1];
                    mb_odometry.q_obs_r[i] = mb_odometry.obstacle_radius[i-1];
                }

                else{
                    mb_odometry.q_obs_x[i]= 100000;
                    mb_odometry.q_obs_y[i]= 100000;
                    mb_odometry.q_obs_r[i] = 100000;
                }
            }

            break;

        case 2:
            
            for (i=0;i<NUMGATES*2;i++){
                
                if (i<4)
                {
                    mb_odometry.q_obs_x[i]=mb_odometry.leftpost_x[i];
                    mb_odometry.q_obs_x[i+1]=mb_odometry.rightpost_x[i];
                    mb_odometry.q_obs_x[i+2]=mb_odometry.leftpost_x[i+1];
                    mb_odometry.q_obs_x[i+3]=mb_odometry.rightpost_x[i+1];

                    mb_odometry.q_obs_y[i]=mb_odometry.leftpost_y[i];
                    mb_odometry.q_obs_y[i+1]=mb_odometry.rightpost_y[i];
                    mb_odometry.q_obs_y[i+2]=mb_odometry.leftpost_y[i+1];
                    mb_odometry.q_obs_y[i+3]=mb_odometry.rightpost_y[i+1];

                    mb_odometry.q_obs_r[i]= post_radius;
                    mb_odometry.q_obs_r[i+1]= post_radius;
                    mb_odometry.q_obs_r[i+2]= post_radius;
                    mb_odometry.q_obs_r[i+3]= post_radius;
                    i=i+3;
                }
                
                else if (i>=4 && i<6)
                    {
                    mb_odometry.q_obs_x[i] = mb_odometry.obstacle_x[i-2];
                    mb_odometry.q_obs_y[i] = mb_odometry.obstacle_y[i-2];
                    mb_odometry.q_obs_r[i] = mb_odometry.obstacle_radius[i-2];
                }

                else{
                    mb_odometry.q_obs_x[i]= 100000;
                    mb_odometry.q_obs_y[i]= 100000;
                    mb_odometry.q_obs_r[i] = 100000;
                }
            }

            break;

        case 3:
            for (i=0;i<NUMGATES*2;i++){
                
                if (i<6)
                {
                    mb_odometry.q_obs_x[i]=mb_odometry.leftpost_x[i];
                    mb_odometry.q_obs_x[i+1]=mb_odometry.rightpost_x[i];
                    mb_odometry.q_obs_x[i+2]=mb_odometry.leftpost_x[i+1];
                    mb_odometry.q_obs_x[i+3]=mb_odometry.rightpost_x[i+1];
                    mb_odometry.q_obs_x[i+4]=mb_odometry.leftpost_x[i+2];
                    mb_odometry.q_obs_x[i+5]=mb_odometry.rightpost_x[i+2];

                    mb_odometry.q_obs_y[i]=mb_odometry.leftpost_y[i];
                    mb_odometry.q_obs_y[i+1]=mb_odometry.rightpost_y[i];
                    mb_odometry.q_obs_y[i+2]=mb_odometry.leftpost_y[i+1];
                    mb_odometry.q_obs_y[i+3]=mb_odometry.rightpost_y[i+1];
                    mb_odometry.q_obs_y[i+4]=mb_odometry.leftpost_y[i+2];
                    mb_odometry.q_obs_y[i+5]=mb_odometry.rightpost_y[i+2];

                    mb_odometry.q_obs_r[i]= post_radius;
                    mb_odometry.q_obs_r[i+1]= post_radius;
                    mb_odometry.q_obs_r[i+2]= post_radius;
                    mb_odometry.q_obs_r[i+3]= post_radius;
                    mb_odometry.q_obs_r[i+4]= post_radius;
                    mb_odometry.q_obs_r[i+5]= post_radius;
                    i=i+5;
                }
                
                else if (i>=6 && i<7)
                    {
                    mb_odometry.q_obs_x[i] = mb_odometry.obstacle_x[i-3];
                    mb_odometry.q_obs_y[i] = mb_odometry.obstacle_y[i-3];
                    mb_odometry.q_obs_r[i] = mb_odometry.obstacle_radius[i-3];
                }

                else{
                    mb_odometry.q_obs_x[i]= 100000;
                    mb_odometry.q_obs_y[i]= 100000;
                    mb_odometry.q_obs_r[i] = 100000;
                }
            }

            break;

        case 4:
        	i=0;
                mb_odometry.q_obs_x[i]=mb_odometry.leftpost_x[i];
                    mb_odometry.q_obs_x[i+1]=mb_odometry.rightpost_x[i];
                    mb_odometry.q_obs_x[i+2]=mb_odometry.leftpost_x[i+1];
                    mb_odometry.q_obs_x[i+3]=mb_odometry.rightpost_x[i+1];
                    mb_odometry.q_obs_x[i+4]=mb_odometry.leftpost_x[i+2];
                    mb_odometry.q_obs_x[i+5]=mb_odometry.rightpost_x[i+2];
                    mb_odometry.q_obs_x[i+6]=mb_odometry.leftpost_x[i+3];
                    mb_odometry.q_obs_x[i+7]=mb_odometry.rightpost_x[i+3];

                    mb_odometry.q_obs_y[i]=mb_odometry.leftpost_y[i];
                    mb_odometry.q_obs_y[i+1]=mb_odometry.rightpost_y[i];
                    mb_odometry.q_obs_y[i+2]=mb_odometry.leftpost_y[i+1];
                    mb_odometry.q_obs_y[i+3]=mb_odometry.rightpost_y[i+1];
                    mb_odometry.q_obs_y[i+4]=mb_odometry.leftpost_y[i+2];
                    mb_odometry.q_obs_y[i+5]=mb_odometry.rightpost_y[i+2];
                    mb_odometry.q_obs_y[i+6]=mb_odometry.leftpost_y[i+3];
                    mb_odometry.q_obs_y[i+7]=mb_odometry.rightpost_y[i+3];

                    mb_odometry.q_obs_r[i]= post_radius;
                    mb_odometry.q_obs_r[i+1]= post_radius;
                    mb_odometry.q_obs_r[i+2]= post_radius;
                    mb_odometry.q_obs_r[i+3]= post_radius;
                    mb_odometry.q_obs_r[i+4]= post_radius;
                    mb_odometry.q_obs_r[i+5]= post_radius;
                    mb_odometry.q_obs_r[i+6]= post_radius;
                    mb_odometry.q_obs_r[i+7]= post_radius;

                break;

        default:
        	printf("this shouldn't happen");          
    }
}

//Calculates potential at a point
float potential(float position_robot_guess_x, float position_robot_guess_y, float position_goal_x, float position_goal_y)
{
    float lam = 1.0;
    float k = 12.0;
    // Defining all of these.
	//beta_i, grad_beta_i
    float beta [NUMGATES*2];
    
	// Define World
    // r_0 is radius of the world
    float r_0 = 6*12*2.54/100.0;

    // origin is basically centre of world
    float origin_x = 0;
    float origin_y = 0;

	// position_goal is basically q_goal
    float d_guess = distance(position_goal_x, position_goal_y, position_robot_guess_x, position_robot_guess_y);

    // We find beta_i for all i from 1 to N and multiply them all to get beta_q
    float beta_0_q = -pow((distance(position_robot_guess_x, position_robot_guess_y, origin_x, origin_y)),2) + pow((r_0),2);
    int N = NUMGATES*2; //= no. of obstacles

    float beta_q = 1.0;

    int i;
    for(i=0;i<N;i++)
    {
        if(mb_odometry.q_obs_x[i]>1000){
            break;
        }
    	beta[i] = pow((distance(position_robot_guess_x,position_robot_guess_y,mb_odometry.q_obs_x[i],mb_odometry.q_obs_y[i])),2)-pow((mb_odometry.q_obs_r[i]),2);
    	beta_q = beta_q*beta[i];  
    }

    beta_q = beta_q * beta_0_q;
    float potential_value = pow(d_guess,2)/pow((lam*beta_q + pow(d_guess,2*k)),(1.0/k));
    return potential_value;
}

//Determines which direction has the lowest gradient from the robot
void getDirection(float position_goal_x, float position_goal_y){
    float d = 0.10;
    float N = 64.0;
    float x = mb_odometry.x;
    float y = mb_odometry.y;
    int i;
    float theta_guess;
    float x_guess;
    float y_guess;
    float index_min;
    float min_value = 999999999999999.9;
    float potential_val;
    for(i=0;i<N;i++){
        theta_guess = i*(2*3.14159/N);
        x_guess = x + d*cos(theta_guess);
        y_guess = y + d*sin(theta_guess);
        potential_val = potential(x_guess,y_guess,position_goal_x,position_goal_y);
        if(potential_val < min_value){
            min_value = potential_val;
            index_min = i;
        }
    }
    mb_state.target_x = x + d*cos(index_min*(2*3.14159/N));
    mb_state.target_y = y + d*sin(index_min*(2*3.14159/N));
}

float distance(float x1,float y1,float x2,float y2)
{
    float d = sqrt( pow(x1 - x2,2) + pow(y1 - y2,2) );
    return d;
}