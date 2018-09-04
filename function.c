//Structs to go in mb_odometry:
float obstacle_xy[NUMGATES][2];
float obstacle_radius[NUMGATES];
float entry_xy[NUMGATES][2];
float exit_xy[NUMGATES][2];

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