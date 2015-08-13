#ifndef PROJ_H
#define PROJ_H

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
#include "lcmtypes/pose_xyt_t.h"
#include "lcmtypes/scanpose_xyt_t.h"
#include "lcmtypes/rplidar_laser_t.h"


#define BIG_MAP 50000

#define JOYSTICK_REVERSE_SPEED1 -0.25f
#define JOYSTICK_FORWARD_SPEED1  0.35f

#define JOYSTICK_REVERSE_SPEED2 -0.35f
#define JOYSTICK_FORWARD_SPEED2  0.45f

#define MAX_REVERSE_SPEED -0.35f
#define MAX_FORWARD_SPEED  0.35f

#define VXO_GRID_SIZE 0.25 // [m]

#define SLIDING_TIME_WINDOW 10000000 // 10 s

#define GOAL_RADIUS 0.25 // [m]

#define ELLIPSES 3
#define TRAJ_LEN 100
#define PLOT_ELLIPSE_DISTANCE 1

#define dmax(A,B) A < B ? B : A
#define dmin(A,B) A < B ? A : B

//map is a square grid of size n

#define CELL_WIDTH 0.05
#define MAP_HEIGHT 100
#define MAP_WIDTH 100

#define MAP_STARTX -2
#define MAP_STARTY -2

#include "priorityQueue.h"


typedef struct state state_t;

typedef struct map map_t;

typedef struct path path_t;

struct map{

    double X;
    double Y;
    double prob;
    double logodds;
};

struct path{
    int startLoc[2];
    int goalLoc[2];
    int gridWidth;
    int gridHeight;
    int km;
    int **cost;
    int **g;
    int **rhs;
    int **xy;
    double nextWaypoint[2];
    int pathLength;
    bool first;
    PriorityQueue pq;
};


struct state {
    bool running;
    getopt_t *gopt;
    char *url;
    image_source_t *isrc;
    int fidx;
    lcm_t *lcm;
  
    int yaw;
    unsigned char go_map;
    pose_xyt_t *pose;
    scanpose_xyt_t *scan_pose;
   
    pthread_t command_thread;
    maebot_diff_drive_t cmd;
    bool manual_control;
    bool pid_control;

    pthread_t render_thread;
    bool   have_goal;
    double goal[2];

    pthread_t slam_thread;

    vx_world_t *vw;
    vx_application_t app;
    vx_event_handler_t veh;
    zhash_t *layer_map; // <display, layer>

    map_t ogmap[BIG_MAP];    
    map_t ogmap_old[BIG_MAP];
    double cell_width;
    int map_size_n;
    int map_size_h;
    int map_size_w;
    double map_startx;
    double map_starty;
   
    
    float pathXY[BIG_MAP*2*3];
    int pathlen;

    pthread_mutex_t mutex;

    path_t gPath;
};


/* Function declaraions for EECS 568 project */

// Function to initialize maps
void Init_map(state_t *state);

// Scan matching function with closed form solution for minimization 
void ScanMatchingB(double *xyt_scan,double *x_jk,double* A,double* B,
		   int nrangesA,int nrangesB);

// Occupancy grid mapping implementation
void slam(state_t *state,rplidar_laser_t range_data);

// PID control
void pidcontrol(state_t *state);

// Function to update maps (calls occupancy grid mapping)
void update_map(state_t* state,rplidar_laser_t range_data);

// update map function using bresenham algorithm
void update_mapB(state_t* state,rplidar_laser_t range_data);

//Inverse range sensor model for mapping
double inv_sensor_model(state_t *state,double X,double Y,rplidar_laser_t range_data);

//Utility function

//Coordinate transformation
void world2map(state_t* state,double wx,double wy,int *mx,int *my);

//Function for ray tracing
int bresenham(int *start,int *end,int *X,int *Y);



#endif
