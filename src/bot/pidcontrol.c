#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <math.h>
#include <lcm/lcm.h>

#include "vx/vx.h"
#include "vx/vxo_drawables.h"
#include "vx/vx_remote_display_source.h"

#include "common/getopt.h"
#include "common/timestamp.h"

#include "math/matd.h"
#include "math/math_util.h"
#include "math/gsl_util_vector.h"
#include "math/gsl_util_matrix.h"
#include "math/gsl_util_eigen.h"
#include "math/gsl_util_blas.h"
#include "math/homogenous.h"

#include "imagesource/image_util.h"
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"

#include "lcmtypes/maebot_diff_drive_t.h"
#include "lcmtypes/maebot_laser_t.h"
#include "lcmtypes/maebot_leds_t.h"
#include "lcmtypes/maebot_sensor_data_t.h"
#include "lcmtypes/maebot_motor_feedback_t.h"
#include "lcmtypes/pose_xyt_t.h"
#include "lcmtypes/rplidar_laser_t.h"

#include "xyt.h"
#include "proj568.h"

void pidcontrol(state_t * state){
    double Ts=0.05;

	double kp = 0.015;
	double kd = 0*0.005/Ts;
	double ki = 0*Ts*.01;


	double v = 0.0025;
	double L = 0.08;
	double R = 0.015;

	double goaldist;
	
	double theta_d;
	
	static unsigned char subgoalReach = 1;
	static double goalx = 0;
	static double goaly = 0;

	static double theta_error_old = 0;
	static double theta_error_int = 0;


	if(subgoalReach){
	  goalx = state->gPath.nextWaypoint[0];
	  goaly = state->gPath.nextWaypoint[1];

	  theta_error_old = 0;
	  theta_error_int = 0;


	  //usleep (500000);

	  //goalx = state->goal[0];
	  //goaly = state->goal[1];
	}

	printf("sub goal x = %f, sub goal y = %f\n",goalx,goaly);
	printf(" current pose = %f %f %f\n",state->scan_pose->xyt[0],state->scan_pose->xyt[1],state->scan_pose->xyt[2]);
	double subgoaldist = sqrt( pow((goalx - state->scan_pose->xyt[0]),2) + pow((goaly - state->scan_pose->xyt[1]),2) );


	if(fabs(subgoaldist) < 0.2){
	  subgoalReach = 1;
	  
	}
	else{
	  subgoalReach = 0;

	}

	
	theta_d = atan2(goaly - state->scan_pose->xyt[1], goalx - state->scan_pose->xyt[0]);

	double theta_error = theta_d - state->scan_pose->xyt[2];


	//if(theta_error > 60*M_PI/180)
	//theta_error = 60*M_PI/180;
	//else if(theta_error < -60*M_PI/180)
	//theta_error = -60*M_PI/180;

	printf("pose theta = %f, theta error = %f\n",state->scan_pose->xyt[2],theta_error);

	while (theta_error < -M_PI){
		theta_error += 2*M_PI;
	}
	while (theta_error > M_PI){
		theta_error -= 2*M_PI;
	}

	if (abs(theta_error) > (M_PI/2)) {
		v = 0.002;
	}
	
	theta_error_int += theta_error;
	double theta_error_der = theta_error - theta_error_old;
	double w = kp*theta_error + kd*theta_error_der + ki*theta_error_int;

	double v_r = (2*v+w*L)/(2*R);
	double v_l = (2*v-w*L)/(2*R);

	if(v_r > 0.5)
	  v_r = 0.5;

	if(v_r < -0.5)
	  v_r = -0.5;

	if(v_l > 0.5)
	  v_l = 0.5;


	if(v_l < -0.5)
	  v_l = -0.5;

	goaldist = sqrt(pow((state->goal[0] - state->scan_pose->xyt[0]),2)+ pow((state->goal[1] - state->scan_pose->xyt[1]),2));
	//printf("%f\n",goaldist);
	if (goaldist < 0.25){
		v_r = 0;
		v_l = 0;
	}

	//v_r = 0;
	//v_l = 0;

	printf("vr = %f\n",v_r);
	printf("vl = %f\n",v_l);

	state->cmd.motor_right_speed = v_r;
	state->cmd.motor_left_speed  = v_l;

    theta_error_old = theta_error;
}
