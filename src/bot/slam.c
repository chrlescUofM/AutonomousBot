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

#include "lcmtypes/maebot_diff_drive_t.h"
#include "lcmtypes/maebot_laser_t.h"
#include "lcmtypes/maebot_leds_t.h"
#include "lcmtypes/maebot_sensor_data_t.h"
#include "lcmtypes/maebot_motor_feedback_t.h"
#include "lcmtypes/pose_xyt_t.h"
#include "lcmtypes/rplidar_laser_t.h"

#include "xyt.h"
#include "proj568.h"

#define POCC 0.98
#define PFREE 0.2
#define OG_ALPHA 0.025
#define OG_BETA 2

#define MAP_RESET 50

extern zarray_t *trajectory;


double inv_sensor_model(state_t *state,double X,double Y,rplidar_laser_t range_data){

    double min_angle  = 500;
    double cell_range = 0; 
    double cell_angle = 0;
    double diff_angle = 0;     
    int K;
    
    cell_range = sqrt( pow((state->scan_pose->xyt[0] - X),2) +  pow((state->scan_pose->xyt[1] - Y),2)  );
    cell_angle = atan2(Y-state->scan_pose->xyt[1],X-state->scan_pose->xyt[0] ) - state->scan_pose->xyt[2];

    if(cell_angle < 0)
    	cell_angle = 2*3.142 + cell_angle;


     // Virtual perceptual range
    if(cell_range > 10){
	return 0;
    }
    else{
	for(int i=0;i<range_data.nranges;i++){

	    int traj_size = zarray_size(trajectory);
	    int id        = traj_size - 1;
	    
	    trajectory_t pose1;
	    trajectory_t pose2;

	    double xyt[3] = {0,0,0};
	    
	    
	    if( (range_data.times[i] < state->scan_pose->utime) && (traj_size>0) ){
		
		zarray_get(trajectory,id,&pose1);
		
		while(range_data.times[i] < pose1.utime){
		    
		    id = id - 1;
		    zarray_get(trajectory,id,&pose1);
		}
		
		zarray_get(trajectory,id+1,&pose2);
		
		
		int64_t time_diff = pose2.utime - pose1.utime;
		
		xyt[0] = (pose2.xyt[0] - pose1.xyt[0])/(time_diff)*(range_data.times[i]-pose1.utime) + pose1.xyt[0];
		xyt[1] = (pose2.xyt[1] - pose1.xyt[1])/(time_diff)*(range_data.times[i]-pose1.utime) + pose1.xyt[1];
		xyt[2] = (pose2.xyt[2] - pose1.xyt[2])/(time_diff)*(range_data.times[i]-pose1.utime) + pose1.xyt[2];
		
	    }
	    else{
		memcpy(xyt,state->scan_pose->xyt,3*sizeof(double));
	    }
	    
	    
	    cell_range = sqrt( pow((xyt[0] - X),2) +  pow((xyt[1] - Y),2)  );
	    cell_angle = atan2(Y-xyt[1],X-xyt[0] ) - xyt[2];
	    
	    if(cell_angle < 0)
		cell_angle = 2*3.142 + cell_angle;
	    
	    
	    
	    diff_angle = fabs(cell_angle - (2*3.142-range_data.thetas[i])); 	  
	    
	    if(diff_angle <= min_angle){
		min_angle = diff_angle;
		K = i;
	    }	
	}
	
	
	
	if(fabs(cell_angle - (2*3.142-range_data.thetas[K]))*180/3.142 > OG_BETA)
	    return 0;
	
	
	if( (cell_range - range_data.ranges[K]) < -OG_ALPHA)
	    return log(PFREE/(1-PFREE));
	
	if( fabs(cell_range - range_data.ranges[K]) < OG_ALPHA)
	    return log(POCC/(1-POCC));
	else
	    return 0;
	
    }

}

void update_mapB(state_t *state,rplidar_laser_t range_data){

   
    double cell_range = 0; 
    double cell_angle = 0;
    int *laseri    = (int*) calloc(range_data.nranges,sizeof(int));
    int *laserj    = (int*) calloc(range_data.nranges,sizeof(int));
    int *robint_i  = (int*) calloc(range_data.nranges,sizeof(int));
    int *robint_j  = (int*) calloc(range_data.nranges,sizeof(int));
    double xyt[3] = {0,0,0};


    static unsigned char map_reset = 0;
    static int counter = 0;

    counter++;

    if(counter>MAP_RESET){
      map_reset = 1;
      counter   = 0;
    }

    //printf("MAP reset counter = %d\n",counter);

    for(int i=0;i<range_data.nranges;i++){
	
	int traj_size = zarray_size(trajectory);
	int id        = traj_size - 1;
	
	trajectory_t pose1;
	trajectory_t pose2;
	
	
	if( (range_data.times[i] < state->scan_pose->utime) && (traj_size>0) ){
	    
	    zarray_get(trajectory,id,&pose1);
	    
	    while(range_data.times[i] < pose1.utime){
		
		id = id - 1;
		zarray_get(trajectory,id,&pose1);
	    }
	    
	    zarray_get(trajectory,id+1,&pose2);
	    
	    
	    int64_t time_diff = pose2.utime - pose1.utime;
	    
	    xyt[0] = (pose2.xyt[0] - pose1.xyt[0])/(time_diff)*(range_data.times[i]-pose1.utime) + pose1.xyt[0];
	    xyt[1] = (pose2.xyt[1] - pose1.xyt[1])/(time_diff)*(range_data.times[i]-pose1.utime) + pose1.xyt[1];
	    xyt[2] = (pose2.xyt[2] - pose1.xyt[2])/(time_diff)*(range_data.times[i]-pose1.utime) + pose1.xyt[2];
	    
	}
	else{
	    memcpy(xyt,state->scan_pose->xyt,3*sizeof(double));
	}
	
	
	cell_range = range_data.ranges[i];
	cell_angle = range_data.thetas[i];
	
	if(fabs(cell_range) > 100)
	  cell_range = 0;

	double cellx = cell_range*cos(-cell_angle);
	double celly = cell_range*sin(-cell_angle);

        double mapwx = cos(xyt[2])*cellx - sin(xyt[2])*celly + xyt[0];
	double mapwy = sin(xyt[2])*cellx + cos(xyt[2])*celly + xyt[1];

	int mapi,mapj;
	world2map(state,mapwx,mapwy,&mapi,&mapj);
	laseri[i] = mapi;
	laserj[i] = mapj;

	if(cell_range == 0){
	  laseri[i] = -1;
	  laserj[i] = -1;
	}
	
	//printf("laser range = %f, angle = %f\n",cell_range,cell_angle);
	//printf("laseri,laserj = %d %d\n",laseri[i],laserj[i]);

	int robi,robj;
	world2map(state,xyt[0],xyt[1],&robi,&robj);
	robint_i[i] = robi;
	robint_j[i] = robj;

    }
    
    
    int rayx[500],rayy[500];
    
    
    for(int i=0;i<range_data.nranges;i++){
    
	int mapij[2];
	int robij[2];
	int mapi = laseri[i];
	int mapj = laserj[i];
	int robi = robint_i[i];
	int robj = robint_j[i];
	int ray_n;

	mapij[0] = mapi;
	mapij[1] = mapj;
	robij[0] = robi;
	robij[1] = robj;
	
	if(mapi == -1 || mapj == -1){
	  //printf("invalid index - skipping beam\n");
	  continue;
	}

	//printf("map i j = %d %d\n",mapi,mapj);

	
	ray_n = bresenham(robij,mapij,rayy,rayx);
    

	if(ray_n >= 400){
	  printf("ray casting array size out of memory for rbij = %d %d, mapij = %d %d\n",robi,robj,mapi,mapj);
	    continue;
	}
    
	double logodds    = 0;
	
	// Robots location is a free cell
	logodds = log(PFREE/(1-PFREE));
	state->ogmap[robj * state->map_size_w + robi].logodds += logodds;
	
	logodds = state->ogmap[robj * state->map_size_w + robi].logodds;
	state->ogmap[robj * state->map_size_w + robi].prob    = 1 - 1/(1+exp(logodds));
    

	// Laser end point is an occupied cell
	logodds = log(POCC/(1-POCC));
	state->ogmap[mapj * state->map_size_w + mapi].logodds += logodds;
	logodds = state->ogmap[mapj * state->map_size_w + mapi].logodds;
	state->ogmap[mapj * state->map_size_w + mapi].prob    = 1 - 1/(1+exp(logodds));

    
	// Ray casted cells are all free
	for(int j=1;j<ray_n;j++){
	
	    unsigned char overlap = 0;
	
	    for(int k=0;k<range_data.nranges;k++){
		int map_ind  = laserj[k] * state->map_size_w + laseri[k];
		int free_ind = rayy[j] * state->map_size_w + rayx[j];
		if ( map_ind == free_ind){
		    overlap = 1;
		    break;
		}
		else if ( !map_reset ){
		  if(state->ogmap[free_ind].prob > POCC){
		    overlap = 1;
		    break;
		  }
		}
	    }
	
	    if(overlap){
		continue;
	    }
	
	    logodds = log(PFREE/(1-PFREE));
	
	    state->ogmap[rayy[j] * state->map_size_w + rayx[j]].logodds += logodds;
	    
	    logodds = state->ogmap[robj * state->map_size_w + robi].logodds;
	    state->ogmap[rayy[j] * state->map_size_w + rayx[j]].prob    = 1 - 1/(1+exp(logodds));
	
	}
	


	
    }


    free(laseri);
    free(laserj);
    free(robint_i);
    free(robint_j);
    map_reset = 0;
    
}




void update_map(state_t* state,rplidar_laser_t range_data){

   int map_size = state->map_size_h*state->map_size_w;
   
   double pose[3];

   map_t* ogmap = state->ogmap;

   memcpy(pose,state->scan_pose->xyt,sizeof(double)*3);
	
   for(int i=0;i<map_size;i++){	
      	ogmap[i].logodds += inv_sensor_model(state,ogmap[i].X,ogmap[i].Y,range_data);
        ogmap[i].prob     = 1 - 1/(1+exp(ogmap[i].logodds));
   }         
}

void Init_map(state_t *state){
    
    double dx       = state->cell_width;
    int map_sizeh   = state->map_size_h;
    int map_sizew   = state->map_size_w;

    double map_startx = state->map_startx;
    double map_starty = state->map_starty;    
    
    

    for(int i=0;i<map_sizeh;i++){
	for(int j=0;j<map_sizew;j++){

	    state->ogmap[i*map_sizew + j].Y       = map_starty + i*dx;
            state->ogmap[i*map_sizew + j].X       = map_startx + j*dx;
	    
	    state->ogmap[i*map_sizew + j].prob    = 0.5;
	    
	    state->ogmap[i*map_sizew + j].logodds = 0;
	}		
	
    }
}

void world2map(state_t* state,double wx,double wy,int *mx,int *my){
 
    double pointx  = round((double)(wx - state->map_startx)/state->cell_width);
    double pointy  = round((double)(wy - state->map_starty)/state->cell_width);
    
    *mx = pointx;
    *my = pointy;

}


/**
 * Note: X is Y and Y is X
 */
int bresenham(int *start,int *end,int *X,int *Y){

  
    int x[2] = {start[0],end[0]};
    int y[2] = {start[1],end[1]};
    
    int steep;

    int ys = start[1];
    int ye = end[1];

    int xs = start[0];
    int xe = end[0];


    if( abs(ye - ys) > abs(xe - xs) )
	steep = 1;
    else
	steep = 0;

    if(steep){

	int temp[2];
	temp[0] = x[0];
	temp[1] = x[1];
	x[0]    = y[0];
	x[1]    = y[1];
	y[0]    = temp[0];
        y[1]    = temp[1];

    }

    if( x[0] > x[1] ){
	int temp;
	temp = x[0];
	x[0] = x[1];
	x[1] = temp;

	temp = y[0];
	y[0] = y[1];
	y[1] = temp;

    }

    int delx  = x[1] - x[0];
    int dely  = abs(y[1] - y[0]);
    int error = 0;
    int x_n   = x[0];
    int y_n   = y[0];
    int ystep;

    if(delx > 400)
	return 1000;


    if ( y[0] < y[1] )
	ystep = 1;
    else
	ystep = -1;

    for(int i=0;i<=delx;i++){
	if(steep){
	    X[i] = x_n;
	    Y[i] = y_n;
	}
	else{
	    X[i] = y_n;
	    Y[i] = x_n;
	}

	x_n = x_n + 1;
	error += dely;
	
	if( 2*error >= delx ){
	    y_n = y_n + ystep;
	    error = error - delx;
	}


    }

   

    return delx+1;
    
}
