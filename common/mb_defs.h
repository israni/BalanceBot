/*******************************************************************************
* mb_defs.h
*
*   defines for your bot
*   You will need to fill this in based on the data sheets, schematics, etc. 
*      and your specific configuration...
* 
*******************************************************************************/
#ifndef MB_DEFS_H
#define MB_DEFS_H

#define DEFAULT_PWM_FREQ        25000 // period of motor drive pwm
#define LEFT_MOTOR              1     // id of left motor DONE
#define RIGHT_MOTOR             2     // id of right motor DONE
#define MDIR1                   60    // gpio1.28  P9.12 DONE
#define MDIR2                   48    // gpio1.16  P9.15 DONE
#define MOT_EN                  20    // gpio0.20  P9.41 DONE
#define MOT_1_POL               1.0    // polarity of left motor DONE
#define MOT_2_POL               -1.0    // polarity of right motor DONE
#define ENC_1_POL               1.0    // polarity of left encoder DONE
#define ENC_2_POL               -1.0    // polarity of right encoder DONE
#define GEAR_RATIO              20.4    // gear ratio of motor DONE
#define ENCODER_RES             48.0   // encoder counts per motor shaft revolution DONE
#define WHEEL_DIAMETER          0.0817 // diameter of wheel in meters DONE
#define WHEEL_BASE              0.01  // wheel separation distance in meters
#define FWD_VEL_SENSITIVITY     0.0   // sensitivity of RC control for moving
#define TURN_VEL_SENSITIVITY    0.0   // sensitivity of RC control for turning
#define SAMPLE_RATE_HZ          100.0   // main filter and control loop speed
#define DT                      1.0/SAMPLE_RATE_HZ  // 1/sample_rate
#define PRINTF_HZ               10.0    // rate of print loop
#define RC_CTL_HZ               100    // rate of RC data update
#define TOP_SPEED				0.25

#endif
