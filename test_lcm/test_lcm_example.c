/*******************************************************************************
*
*   test_lcm_example.c
*   Publishes IMU data to TODO what channel what we publishing
*
*   tsnowak@umich.edu
*   ***************************************************************************/

#include <lcm/lcm.h>
#include "../lcmtypes/balancebot_msg_t.h"
#include "../lcmtypes/balancebot_gate_t.h"
#include "../lcmtypes/pose_xyt_t.h"
#include "../balancebot/balancebot.h"

#define     LCM_HZ              1000
#define     OPTITRACK_CHANNEL   "OPTITRACK_CHANNEL"

// variable declarations
lcm_t * lcm = NULL;
balancebot_msg_t bb_msg;
pthread_mutex_t msg_mutex;

// forward declaration of functions
void* lcm_subscribe_loop(void* ptr);
void* printf_loop(void* ptr);
void beaglebone_message_handler(const lcm_recv_buf_t* rbuf,
                                const char* channel,
                                const balancebot_msg_t* msg,
                                void* userdata);


int main(){                                                                    

    // always initialize cape library first                                    
    if(rc_initialize()){                                                       
        fprintf(stderr,"ERROR: failed to initialize rc_initialize(), \
                    are you root?\n");
        return -1;                                                             
    }                                                                          
                                                                               
    //set cpu freq to max speed                                          
    rc_set_cpu_freq(FREQ_1000MHZ);                                             

    pose_xyt_t init_pose = {.utime=0, .x=0, .y=0, .theta=0}; 
    bb_msg.utime = init_pose.utime;
    bb_msg.pose = init_pose;
    bb_msg.num_gates = 2;
    bb_msg.gates = (balancebot_gate_t*)malloc(sizeof(balancebot_gate_t)*4);

    printf("Creating LCM backend.\n");
    // create mutex to ensure data security
    pthread_mutex_init(&msg_mutex, NULL);

    // create lcm object, subscriber thread
    lcm = lcm_create(NULL);
    pthread_t lcm_subscribe_thread;
    pthread_create(&lcm_subscribe_thread, NULL, lcm_subscribe_loop,
                    (void*) NULL);

    printf("Setting state to RUNNING.\n");
    // done initializing so set state to RUNNING                               
    rc_set_state(RUNNING);                                                     
                                                                               
    // create print thread
    if (isatty(fileno(stdout))){
        printf("Starting Print Thread\n");
        pthread_t printf_thread;
        pthread_create(&printf_thread, NULL, printf_loop, (void*) NULL);
    }                                                                           
                                                                               
    // Keep looping until state changes to EXITING                             
    while(rc_get_state()!=EXITING){                                            

        if(rc_get_state()==PAUSED){                                       
            // do other things                                                 
        }                                                                      
        // sleep                                                               
        usleep(100000);                                                        
    }
    rc_cleanup();                                                              
                                                                               
    return 0;                                                                  
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


// define loop for printing information
void* printf_loop(void* ptr){

    printf("\nPose (X,Y,Z) - # Gates - Next Gate (L-(X,Y), R-(X,Y)) : \n");
    while(rc_get_state()!=EXITING){
        printf("\r");
        printf("(%4.2f, %4.2f, %4.2f) - %2d - ((%4.2f, %4.2f), (%4.2f, %4.2f))",
                    bb_msg.pose.x, 
                    bb_msg.pose.y, 
                    bb_msg.pose.theta,
                    bb_msg.num_gates,
                    bb_msg.gates[1].left_post[0],
                    bb_msg.gates[1].left_post[1],
                    bb_msg.gates[1].right_post[0],
                    bb_msg.gates[1].right_post[1]);
        fflush(stdout);
        usleep(1000000 / PRINTF_HZ);
    }
    return NULL;
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
