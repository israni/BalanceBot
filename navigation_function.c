// To calculate the following-
//d, grad_d, beta, grad_beta
// USe these along with lambda (lam) and kappa (k) to calculate grad_psi

// Defining all of these.
//beta_i, grad_beta_i
    float beta [no. of gates *2];
    float grad_beta_x [no. of gates *2];
    float grad_beta_y [no. of gates *2];

// Define World
    // r_0 is radius of the world
    float r_0;

    // origin is basically centre of world
    float origin_x;
    float origin_y;

// Define obstacles
    //Positions of obstacles
    float q_obs_x [no. of gates*2];
    float q_obs_y [no. of gates*2];

    // radius of the obstacles in the world
    float r [no. of gates *2];

// position_goal is basically q_goal
    float position_goal_x;
    float position_goal_y;

// position_robot is basically q
    float position_robot_x;
    float position_robot_y;


// d= d(q,q_goal)
d = distance(position_goal_x, position_goal_y, position_robot_x, position_robot_y);

// We find beta_i for all i from 1 to N and multiply them all to get beta_q

beta_0_q = -(distance(position_robot_x, position_robot_y, origin_x, origin_y))^2 + (r_0)^2;
N = no. of gates*2; //= no. of obstacles

float beta_q = 1.0;

for i= 1:N
{
  beta(i) = (distance(position_robot_x,position_robot_y,q_obs_x(i),q_obs_y(i)))^2-(r(i))^2;
  beta_q = beta_q*beta(i);
  grad_beta_x(i) = position_robot_x - q_obs_x(i);
  grad_beta_y(i) = position_robot_y - q_obs_y(i);
}

grad_beta_x_total = -2*(position_robot_x - origin_x);
grad_beta_y_total = -2*(position_robot_y - origin_y);

for j= 1:N
{
grad_beta_x_total = grad_beta_x_total + grad_beta_x(j)*beta_q/beta(j);
grad_beta_y_total = grad_beta_y_total + grad_beta_y(j)*beta_q/beta(j);
}


grad_psi_at_q_x = (2*d*grad_d_x*((lam*beta_q + d^2*k)^(2/k)) - (1/k)*(lam*beta_q*position_robot_x + d^(2*k))^((1/k)-1)*(lam*grad_beta_x_total + 2*k*(d)^(2*k-1)*grad_d_x))/((lam*beta_q + d^(2*k))^(2/k));
grad_psi_at_q_y = (2*d*grad_d_y*((lam*beta_q + d^2*k)^(2/k)) - (1/k)*(lam*beta_q*position_robot_y + d^(2*k))^((1/k)-1)*(lam*grad_beta_y_total + 2*k*(d)^(2*k-1)*grad_d_y))/((lam*beta_q + d^(2*k))^(2/k));


function distance(x1,y1,x2,y2)
{
d = sqrt((x1 - x2)^2 + (y1 - y2)^2);
}
