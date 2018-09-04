#ifndef MB_STRUCTS_H
#define MB_STRUCTS_H
#define NUMGATES 4

typedef struct mb_state mb_state_t;
struct mb_state{
    // raw sensor inputs
    float   alpha;             // body angle (rad)
    float   theta;             // heading  (rad)
    float   dtheta;             // heading  (rad)
    int     left_encoder;      // left encoder counts since last reading
    int     right_encoder;     // right encoder counts since last reading
    int     left_encoder_total;
    int     right_encoder_total;

    //outputs
    float   left_cmd;  //left wheel command [-1..1]
    float   right_cmd; //right wheel command [-1..1]

    //TODO: Add more variables to this state as needed
    float left_command_speed;  //target speed in rotations per second for the wheel
    float right_command_speed;

    float target_angle;
    float target_theta;

    float target_velocity;
    float target_position;
    float target_x;
    float target_y;

    float range; //R
    float los_angle; //Beta

    float turn_offset;

    //these are here because I'm lazy.
    int square_step;
    int dragrace_step;
    int race_step;
    int test_step;
    int turnflag;
    int mazeflag;

    float max_speed;
    float delay;
};

typedef struct mb_setpoints mb_setpoints_t;
struct mb_setpoints{

    float fwd_velocity; // fwd velocity in m/s
    float turn_velocity; // turn velocity in rad/s
    int manual_ctl;
    int square_btn;
    int dragrace_btn;
    float x;
    float y;
};

typedef struct mb_odometry mb_odometry_t;
struct mb_odometry{

    float x;        //x position from initialization in m
    float y;        //y position from initialization in m
    float theta;    //orientation from initialization in rad
    float optix;
    float optiy;
    float optitheta;

    float waypoint_x;
    float waypoint_y;

    float leftpost_x[NUMGATES];
    float rightpost_x[NUMGATES];
    float leftpost_y[NUMGATES];
    float rightpost_y[NUMGATES];
    int gateopen[NUMGATES];
    int potentialFieldCalculated;

    float obstacle_x[NUMGATES];
    float obstacle_y[NUMGATES];
    float obstacle_radius[NUMGATES];
    float entry_x[NUMGATES];
    float exit_x[NUMGATES];
    float entry_y[NUMGATES];
    float exit_y[NUMGATES];
    int num_gates_crossed;

    float q_obs_x[NUMGATES*2];
    float q_obs_y[NUMGATES*2];
    float q_obs_r[NUMGATES*2];

    int dtagf;
};

#endif