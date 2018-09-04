// To calculate the following-
//d, grad_d, beta, grad_beta
// USe these along with lambda (lam) and kappa (k) to calculate grad_psi

// Defining all of these.
//beta_i, grad_beta_i
    float beta [bb_msg.num_gates*2];
    
// Define World
    // r_0 is radius of the world
    float r_0;

    // origin is basically centre of world
    float origin_x;
    float origin_y;

// Define obstacles
    //Positions of obstacles
    float q_obs_x [bb_msg.num_gates*2];
    float q_obs_y [bb_msg.num_gates*2];

    // radius of the obstacles in the world
    float r [bb_msg.num_gates*2];

// position_goal is basically q_goal
    float position_goal_x;
    float position_goal_y;

// position_robot is basically q
    float position_robot_x;
    float position_robot_y;



position-robots_guess_x
position-robots_guess_y

d_guess =  distance(position_goal_x, position_goal_y, position_robot_x, position_robot_y);

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

    for(i=1;i<=bb_msg.num_gates;i++){
        x1 = mb_odometry.leftpost_x[i];
        y1 = mb_odometry.leftpost_y[i];
        x2 = mb_odometry.rightpost_x[i];
        y2 = mb_odometry.rightpost_y[i];

        mb_odometry.obstacle_x[i] = (x1+x2)/2;
        mb_odometry.obstacle_y[i] = (y1+y2)/2;
        gate_distance = sqrt(pow(x2-x1,2)+pow(y2-y1,2));
        mb_odometry.obstacle_radius = (gate_distance/2.0)+0.1;

        gate_distance = sqrt(pow(y1-y2,2)+pow(x2-x1,2));
        D = mb_odometry.obstacle_radius;
        mb_odometry.entry_x[i] = mb_odometry.obstacle_x[i]+(goal_offset/gate_distance)*(y1-y2);
        mb_odometry.entry_y[i] = mb_odometry.obstacle_y[i]+(goal_offset/gate_distance)*(x2-x1);
        mb_odometry.exit_x[i] = mb_odometry.obstacle_x[i]-(goal_offset/gate_distance)*(y1-y2);
        mb_odometry.exit_y[i] = mb_odometry.obstacle_y[i]-(goal_offset/gate_distance)*(x2-x1);
    }
    return 1;
}

void update_obstacles(){
// Define obstacles
    //Positions of obstacles
    float q_obs_x [bb_msg.num_gates*2];
    float q_obs_y [bb_msg.num_gates*2];

    // radius of the obstacles in the world
    float r [bb_msg.num_gates*2];

    switch(num_gates_crossed){
        case 0:
            int i;
            for (i=0;i<NUMGATES*2;i++){
                if i<4
                    {
                    q_obs_x[i] = mb_odometry.obstacle_x[i];
                    q_obs_y[i] = mb_odometry.obstacle_y[i];
                    r[i] = mb_odometry.obstacle_radius[i];
                }
                else{
                    q_obs_x[i]= 100000;
                    q_obs_y[i]= 100000;
                    r[i] = 100000;
                }
            }

            break;

        case 1:
            
            for (i=0;i<NUMGATES*2;i++){
                
                if i<2
                {
                    q_obs_x[i]=leftpost_x[i];
                    q_obs_x[i+1]=leftpost_x[i+1];
                    q_obs_y[i]=leftpost_y[i];
                    q_obs_y[i+1]=leftpost_y[i+1];
                    r[i]= 0.05; // Define it somewhere else OWen!
                    i++;
                }
                
                if (i>=2 && i<5)
                    {
                    q_obs_x[i] = mb_odometry.obstacle_x[i];
                    q_obs_y[i] = mb_odometry.obstacle_y[i];
                    r[i] = mb_odometry.obstacle_radius[i];
                }

                else{
                    q_obs_x[i]= 100000;
                    q_obs_y[i]= 100000;
                    r[i] = 100000;
                }
            }

            break;

        case 2:
            
            for (i=0;i<NUMGATES*2;i++){
                
                if i<4
                {
                    q_obs_x[i]=leftpost_x[i];
                    q_obs_x[i+1]=leftpost_x[i+1];
                    q_obs_x[i+2]=leftpost_x[i+2];
                    q_obs_x[i+3]=leftpost_x[i+3];

                    q_obs_y[i]=leftpost_y[i];
                    q_obs_y[i+1]=leftpost_y[i+1];
                    q_obs_y[i+2]=leftpost_y[i+2];
                    q_obs_y[i+3]=leftpost_y[i+3];

                    r[i]= 0.05; // Define it somewhere else OWen!
                    i=i+3;
                }
                
                if (i>=4 && i<6)
                    {
                    q_obs_x[i] = mb_odometry.obstacle_x[i];
                    q_obs_y[i] = mb_odometry.obstacle_y[i];
                    r[i] = mb_odometry.obstacle_radius[i];
                }

                else{
                    q_obs_x[i]= 100000;
                    q_obs_y[i]= 100000;
                    r[i] = 100000;
                }
            }

            break;

        case 3:
            for (i=0;i<NUMGATES*2;i++){
                
                if i<6
                {
                    q_obs_x[i]=leftpost_x[i];
                    q_obs_x[i+1]=leftpost_x[i+1];
                    q_obs_x[i+2]=leftpost_x[i+2];
                    q_obs_x[i+3]=leftpost_x[i+3];
                    q_obs_x[i+4]=leftpost_x[i+4];
                    q_obs_x[i+5]=leftpost_x[i+5];

                    q_obs_y[i]=leftpost_y[i];
                    q_obs_y[i+1]=leftpost_y[i+1];
                    q_obs_y[i+2]=leftpost_y[i+2];
                    q_obs_y[i+3]=leftpost_y[i+3];
                    q_obs_y[i+4]=leftpost_y[i+4];
                    q_obs_y[i+5]=leftpost_y[i+5];

                    r[i]= 0.05; // Define it somewhere else OWen!
                    i=i+5;
                }
                
                if (i>=6 && i<7)
                    {
                    q_obs_x[i] = mb_odometry.obstacle_x[i];
                    q_obs_y[i] = mb_odometry.obstacle_y[i];
                    r[i] = mb_odometry.obstacle_radius[i];
                }

                else{
                    q_obs_x[i]= 100000;
                    q_obs_y[i]= 100000;
                    r[i] = 100000;
                }
            }

            break;

        case 4:
                for (i=0;i<NUMGATES*2;i++){
                
                    q_obs_x[i]=leftpost_x[i];
                    q_obs_y[i]=leftpost_y[i];
                    
                    r[i]= 0.05; // Define it somewhere else OWen!
                    i=i+5;
                }
                break;
                
    }
    mb_odometry.q_obs_x = q_obs_x;
    mb_odometry.q_obs_y = q_obs_y;
    mb_odometry.q_obs_r = r;
}


float potential(r_0,r,q_obs_x,q_obs_y,position_goal_x, position_goal_y, position_robot_guess_x, position_robot_guess_y,float lam,k)
{
    // Defining all of these.
//beta_i, grad_beta_i
    float beta [bb_msg.num_gates*2];
    
// Define World
    // r_0 is radius of the world
    float r_0 = 6*12*2.54/100.0;

    // origin is basically centre of world
    float origin_x = 0;
    float origin_y = 0;

// Define obstacles
    //Positions of obstacles
    float q_obs_x = mb_odometry.obstacle_x;
    float q_obs_y = mb_odometry.obstacle_y;

    // radius of the obstacles in the world
    float r [bb_msg.num_gates*2];

// position_goal is basically q_goal
    float position_goal_x;
    float position_goal_y;

// position_robot is basically q
    float position_robot_x;
    float position_robot_y;

    // d= d(q,q_goal)
    d_guess = distance(position_goal_x, position_goal_y, position_robot_guess_x, position_robot_guess_y);

    // We find beta_i for all i from 1 to N and multiply them all to get beta_q

    beta_0_q = -pow((distance(position_robot_x, position_robot_y, origin_x, origin_y)),2) + pow((r_0),2);
    N = bb_msg.num_gates*2; //= no. of obstacles

    float beta_q = 1.0;

    int i;
    for(i=0;i<N;i++)
    {
    beta[i] = pow((distance(position_robot_x,position_robot_y,q_obs_x[i],q_obs_y[i])),2)-pow((r[i]),2);
    beta_q = beta_q*beta[i];  
    }

    beta_q = beta_q * beta_0_q;

    potential = pow(d,2)/pow((lam*beta_q + pow(d,2*k)),(1/k));
    return potential;
}

float distance(x1,y1,x2,y2)
{
    d = sqrt( pow(x1 - x2,2) + pow(y1 - y2,2) );
    return d;
}

