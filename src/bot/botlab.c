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
#include "lcmtypes/scanpose_xyt_t.h"
#include "lcmtypes/rplidar_laser_t.h"

#include "proj568.h"
#include "xyt.h"

#include "smooth.h"
#include "dStarLite.h"

#define SETTLE 1

zarray_t *trajectory;
zarray_t *ellipse;

double dist_traveled = 0;

unsigned char draw_ellipse = 0;
unsigned char draw_covariance = 0;

rplidar_laser_t range_data;

static void
display_finished (vx_application_t *app, vx_display_t *disp)
{
    state_t *state = app->impl;

    pthread_mutex_lock (&state->mutex);
    {
        vx_layer_t *layer = NULL;
        zhash_remove (state->layer_map, &disp, NULL, &layer);
        vx_layer_destroy (layer);
    }
    pthread_mutex_unlock (&state->mutex);
}

static void
display_started (vx_application_t *app, vx_display_t *disp)
{
    state_t *state = app->impl;

    vx_layer_t *layer = vx_layer_create (state->vw);
    vx_layer_set_display (layer, disp);
    vx_layer_add_event_handler (layer, &state->veh);

    vx_layer_camera_op (layer, OP_PROJ_PERSPECTIVE);
    float eye[3]    = {  0,  0,  5};
    float lookat[3] = {  0,  0,  0 };
    float up[3]     = {  0,  1,  0 };
    vx_layer_camera_lookat (layer, eye, lookat, up, 1);

    pthread_mutex_lock (&state->mutex);
    {
        zhash_put (state->layer_map, &disp, &layer, NULL, NULL);
    }
    pthread_mutex_unlock (&state->mutex);
}

static int
touch_event (vx_event_handler_t *vh, vx_layer_t *vl, vx_camera_pos_t *pos, vx_touch_event_t *mouse)
{
    return 0;
}

static int
mouse_event (vx_event_handler_t *vh, vx_layer_t *vl, vx_camera_pos_t *pos, vx_mouse_event_t *mouse)
{
    state_t *state = vh->impl;

    // Button state
    bool m1 = mouse->button_mask & VX_BUTTON1_MASK;
    bool ctrl = mouse->modifiers & VX_CTRL_MASK;
    
   
    pthread_mutex_lock (&state->mutex);
    {
        if (m1 && ctrl) {
            // Ray cast to find click point
            vx_ray3_t ray;
            vx_camera_pos_compute_ray (pos, mouse->x, mouse->y, &ray);

            double ground[3];
            vx_ray3_intersect_xy (&ray, 0, ground);

            printf ("Mouse clicked at coords: [%8.3f, %8.3f]  Ground clicked at coords: [%6.3f, %6.3f]\n",
                    mouse->x, mouse->y, ground[0], ground[1]);

            state->goal[0] = ground[0];
            state->goal[1] = ground[1];
            state->have_goal = true;
        }
    }
    pthread_mutex_unlock (&state->mutex);

    return 0;
}

static int
key_event (vx_event_handler_t *vh, vx_layer_t *vl, vx_key_event_t *key)
{
    state_t *state = vh->impl;

    static bool key_shift=0, key_up=0, key_down=0, key_left=0, key_right=0;


    if( key->key_code == VX_KEY_E && key->released ){
      state->pid_control = 1;
      printf("pid control on\n");
    } 
    if( key->key_code == VX_KEY_R && key->released ){
      state->pid_control = 0;
      state->cmd.motor_left_speed = state->cmd.motor_right_speed = 0.0;
      printf("pid control off\n");
    }


    switch (key->key_code) {
        case VX_KEY_SHIFT:
            key_shift = !key->released;
            break;
        case VX_KEY_UP:
            key_up = !key->released;
            break;
        case VX_KEY_DOWN:
            key_down = !key->released;
            break;
        case VX_KEY_LEFT:
            key_left = !key->released;
            break;
        case VX_KEY_RIGHT:
            key_right = !key->released;
            break;
        case VX_KEY_CTRL:
            state->manual_control = !key->released;
	    printf("motor command - MANUAL\n");
            if (key->released){
                state->cmd.motor_left_speed = state->cmd.motor_right_speed = 0.0;
		
	    }
            break;
        default:
            break;
    }

    if (state->manual_control) {
        pthread_mutex_lock (&state->mutex);
        {
            // default to zero
            state->cmd.motor_left_speed = state->cmd.motor_right_speed = 0.0;

            float fwd_speed = JOYSTICK_FORWARD_SPEED1;
            float rev_speed = JOYSTICK_REVERSE_SPEED1;
            if (key_shift) { // speed boost
                fwd_speed = JOYSTICK_FORWARD_SPEED2;
                rev_speed = JOYSTICK_REVERSE_SPEED2;
            }

	    

            if (key_up) { // forward
                state->cmd.motor_left_speed = fwd_speed;
                state->cmd.motor_right_speed = fwd_speed;
                if (key_left) {
                    state->cmd.motor_left_speed -= 0.1;
                    state->cmd.motor_right_speed += 0.1;
                }
                else if (key_right) {
                    state->cmd.motor_left_speed += 0.1;
                    state->cmd.motor_right_speed -= 0.1;
                }
            }
            else if (key_down) { // reverse
                state->cmd.motor_left_speed = rev_speed;
                state->cmd.motor_right_speed = rev_speed;
                if (key_left) {
                    state->cmd.motor_left_speed += 0.1;
                    state->cmd.motor_right_speed -= 0.1;
                }
                else if (key_right) {
                    state->cmd.motor_left_speed -= 0.1;
                    state->cmd.motor_right_speed += 0.1;
                }
            }
            else if (key_left) { // turn left
                state->cmd.motor_left_speed =  rev_speed;
                state->cmd.motor_right_speed = -rev_speed;
            }
            else if (key_right) { // turn right
                state->cmd.motor_left_speed = -rev_speed;
                state->cmd.motor_right_speed = rev_speed;
            }
        }
        pthread_mutex_unlock (&state->mutex);
    }

    return 0;
}

static void
destroy (vx_event_handler_t *vh)
{
    // do nothing, since this event handler is statically allocated.
}

static state_t *global_state;
static void handler (int signum)
{
    switch (signum) {
        case SIGINT:
        case SIGQUIT:
            global_state->running = 0;
            break;
        default:
            break;
    }
}


// This thread continuously publishes command messages to the maebot
static void *
command_thread (void *data)
{
    state_t *state = data;
    const uint32_t Hz = 20;
    const char *channel = getopt_get_string (state->gopt, "maebot-diff-drive-channel");

    while (state->running) {
        pthread_mutex_lock (&state->mutex);
        {
            if (state->pid_control) {
              
	      pidcontrol(state);

            }
            // Publish
            state->cmd.utime = utime_now ();
            maebot_diff_drive_t_publish (state->lcm, channel, &(state->cmd));
        }
        pthread_mutex_unlock (&state->mutex);

        usleep (1000000/Hz);
    }

    return NULL;
}

static
void Get_Trajectory(float *line, zarray_t * traj){

    int size_traj = zarray_size(traj);
    
    trajectory_t pose;
    
    if(size_traj>0){
	for(int i=0;i<size_traj;i++){

	    if(i>=TRAJ_LEN)
		break;
	    
	    zarray_get(traj,(size_traj - 1) - i,&pose);
	    
	    line[3*i + 0] = (float)(pose.xyt[0]);
	    line[3*i + 1] = (float)(pose.xyt[1]);
	    line[3*i + 2] = (float)0;
	    
	}

	for(int i=size_traj;i<TRAJ_LEN;i++){
	    line[3*i + 0] = (float)(pose.xyt[0]);
	    line[3*i + 1] = (float)(pose.xyt[1]);
	    line[3*i + 2] = 0.0;
	}

    }
    else{
	for(int i=0;i<TRAJ_LEN;i++){
	       
	    line[3*i + 0] = 0;
	    line[3*i + 1] = 0;
	    line[3*i + 2] = 0;
	    
	}

    }

}

//Compute distance
static double dist(double x1,double y1,double x2, double y2){
    
    double d;

    d = sqrt( pow(x1-x2,2) + pow(y1-y2,2) );

    return d;
    
}

// This thread continously renders updates from the robot
static void *
render_thread (void *data)
{
    state_t *state = data;

    // Grid
    {
        vx_buffer_t *vb = vx_world_get_buffer (state->vw, "grid");
        vx_buffer_set_draw_order (vb, 0);
        vx_buffer_add_back (vb,
                            vxo_chain (vxo_mat_scale (VXO_GRID_SIZE),
                                       vxo_grid ()));



        vx_buffer_swap (vb);
    }

    // Axes
    {
        vx_buffer_t *vb = vx_world_get_buffer (state->vw, "axes");
        vx_buffer_set_draw_order (vb, 0);
        vx_buffer_add_back (vb,
                            vxo_chain (vxo_mat_scale3 (0.10, 0.10, 0.0),
                                       vxo_mat_translate3 (0.0, 0.0, -0.005),
                                       vxo_axes_styled (vxo_mesh_style (vx_red),
                                                        vxo_mesh_style (vx_green),
                                                        vxo_mesh_style (vx_black))));
        vx_buffer_swap (vb);
    }


    const int fps = 30;

    while (state->running) {
        pthread_mutex_lock (&state->mutex);
        {
	    
	    if(range_data.nranges>0){
	      
	      if(state->go_map){
		
	      update_mapB(state,range_data);
	      
	      }
	      
	    }
	    pathPlan(state);
           
            // Robot
            {
                vx_buffer_t *vb = vx_world_get_buffer (state->vw, "robot");
                vx_buffer_set_draw_order (vb, 1);
                enum {ROBOT_TYPE_TRIANGLE, ROBOT_TYPE_DALEK};
                vx_object_t *robot = NULL;
		vx_object_t *robot_odo = NULL;
                switch (ROBOT_TYPE_DALEK) {
                    case ROBOT_TYPE_DALEK: {
                        float line[6] = {0.0, 0.0, 0.151, 0.104, 0.0, 0.151};
                        robot = vxo_chain (vxo_lines (vx_resc_copyf (line, 6),
                                                      2,
                                                      GL_LINES,
                                                      vxo_lines_style (vx_red, 3.0f)),
                                           vxo_mat_scale3 (0.104, 0.104, 0.151),
                                           vxo_mat_translate3 (0.0, 0.0, 0.5),
                                           vxo_cylinder (vxo_mesh_style (vx_blue)));


			// Ghost robot based on odometry
			float rb_color[4] = {1.0,0.0,0.0,0.5};

			robot_odo = vxo_chain (vxo_lines (vx_resc_copyf (line, 6),
                                                      2,
                                                      GL_LINES,
                                                      vxo_lines_style (vx_red, 3.0f)),
                                           vxo_mat_scale3 (0.104, 0.104, 0.151),
                                           vxo_mat_translate3 (0.0, 0.0, 0.5),
                                           vxo_cylinder (vxo_mesh_style (rb_color)));

                        break;
                    }
                    case ROBOT_TYPE_TRIANGLE:
                    default:
                        robot = vxo_chain (vxo_mat_scale (0.104),
                                           vxo_mat_scale3 (1, 0.5, 1),
                                           vxo_triangle (vxo_mesh_style (vx_blue)));


			

			
                        break;
                 }
                
                if (state->pose)
                    vx_buffer_add_back (vb,
                                        vxo_chain (vxo_mat_from_xyt (state->pose->xyt),
                                                   robot_odo));
                else
                    vx_buffer_add_back (vb, robot_odo);

                vx_buffer_swap (vb);
		

		vx_buffer_t *vb2 = vx_world_get_buffer (state->vw, "robot2");
		if (state->scan_pose)
                    vx_buffer_add_back (vb2,
                                        vxo_chain (vxo_mat_from_xyt (state->scan_pose->xyt),
                                                   robot));

		vx_buffer_swap (vb2);

            }


	    
	    // Render occupancy grid map
	    
            {

              vx_buffer_t *vb = vx_world_get_buffer(state->vw, "map");
 
              for(int i=0;i<MAP_HEIGHT*MAP_WIDTH;i++){


		float map_color[] = {1-state->ogmap[i].prob,1-state->ogmap[i].prob,1-state->ogmap[i].prob,1};

	        double map_posx    = state->ogmap[i].X;
	        double map_posy    = state->ogmap[i].Y;
	        

                vx_object_t *cell = vxo_chain(vxo_mat_translate3(map_posx,map_posy,0.0),
					      vxo_mat_scale(state->cell_width),
	 				      vxo_rect(vxo_mesh_style(map_color),
                                                       vxo_lines_style(map_color,2.0f))); 
		vx_buffer_add_back(vb,cell);
              }             
             
	      vx_buffer_swap (vb);

	    }

	    //Store current and previous pose
	    trajectory_t current_pose;
	    trajectory_t prev_pose;
	    memset(&current_pose,0,sizeof(trajectory_t));
	    memset(&prev_pose,0,sizeof(trajectory_t));

	    current_pose.utime = state->scan_pose->utime;
	    memcpy(current_pose.xyt,state->scan_pose->xyt,3*sizeof(double));	    
	    
	    int traj_size = zarray_size(trajectory);
	    
	    if(traj_size)
		zarray_get(trajectory,traj_size-1,&prev_pose);
	    
	    double *xyt_current = state->scan_pose->xyt;
	    double *xyt_prev    = prev_pose.xyt;
	    
	    if(  fabs( xyt_current[0]*1e6 - xyt_prev[0]*1e6 ) > 1 || fabs( xyt_current[1]*1e6 - xyt_prev[1]*1e6) > 1 ){
		
		dist_traveled += dist(xyt_current[0],xyt_current[1],
				      xyt_prev[0],xyt_prev[1]);
				
		zarray_add(trajectory,&current_pose);
			
	    }
   
	 

	    // Visualizing trajectory
	    vx_object_t *traj = NULL;
	    
	    float lines[3*TRAJ_LEN];
	    Get_Trajectory(lines,trajectory);
	    
	    
	    vx_buffer_t *vx_traj = vx_world_get_buffer (state->vw, "trajectory");
	    
	    traj = vxo_chain (vxo_lines (vx_resc_copyf (lines, 3*TRAJ_LEN),
					 TRAJ_LEN,
					 GL_LINES,
					 vxo_lines_style (vx_red, 3.0f)));
	    
	    
	    vx_buffer_add_back (vx_traj,traj);
	    
	    vx_buffer_swap(vx_traj);
	    

	    // Visualizing path
	    vx_object_t *path = NULL;
	    	 
	    vx_buffer_t *vx_path = vx_world_get_buffer (state->vw, "path");
	    
	    path = vxo_chain (vxo_lines (vx_resc_copyf (state->pathXY, 3*state->pathlen),
					 state->pathlen,
					 GL_LINES,
					 vxo_lines_style (vx_blue, 3.0f)));
	    
	    
	    vx_buffer_add_back (vx_path,path);
	    
	    vx_buffer_swap(vx_path);
	    

	    // Visualizing lidar points
	   
	    vx_buffer_t *vx_lidar_red   = vx_world_get_buffer (state->vw, "lidar_red");
	    vx_buffer_t *vx_lidar_green = vx_world_get_buffer (state->vw, "lidar_green");

	    for(int i=0;i<range_data.nranges;i++){
		
		double x_laser = range_data.ranges[i]*cos(-range_data.thetas[i]);
		double y_laser = range_data.ranges[i]*sin(-range_data.thetas[i]);
		
		double xyt[3];
	       
		float points[3]   = {x_laser,y_laser,0};
		int npoints       = 1;
		vx_resc_t *verts1  = vx_resc_copyf(points, npoints*3);
		vx_resc_t *verts2  = vx_resc_copyf(points, npoints*3);
		
		vx_object_t *cloud_red   = vxo_chain(vxo_points(verts1, npoints, vxo_points_style(vx_red, 2.0f)));
		vx_object_t *cloud_green = vxo_chain(vxo_points(verts2, npoints, vxo_points_style(vx_green, 2.0f)));

		
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
		
		
		vx_buffer_add_back(vx_lidar_green,vxo_chain(vxo_mat_from_xyt (xyt),cloud_green));
		vx_buffer_add_back(vx_lidar_red,vxo_chain(vxo_mat_from_xyt (state->pose->xyt),cloud_red));
						     
	
	       
	    }
	    
	    vx_buffer_swap(vx_lidar_green);
	    vx_buffer_swap(vx_lidar_red);
	    

	    // Render goal
	    if (state->have_goal) {
                float color[4] = {0.0, 1.0, 0.0, 0.5};
                vx_buffer_t *vb = vx_world_get_buffer (state->vw, "goal");
                vx_buffer_set_draw_order (vb, -1);
                vx_buffer_add_back (vb,
                                    vxo_chain (vxo_mat_translate3 (state->goal[0], state->goal[1],0.25),
                                               vxo_mat_scale (GOAL_RADIUS),
                                               vxo_circle (vxo_mesh_style (color))));
                vx_buffer_swap (vb);
            }
	}
        pthread_mutex_unlock (&state->mutex);
        usleep (1000000/fps);
    }

    return NULL;
}



// === LCM Handlers =================
static void
maebot_motor_feedback_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                               const maebot_motor_feedback_t *msg, void *user)
{
    state_t *state = user;

    pthread_mutex_lock (&state->mutex);
    {
        

    }
    pthread_mutex_unlock (&state->mutex);
}

static void
maebot_sensor_data_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                            const maebot_sensor_data_t *msg, void *user)
{
    state_t *state = user;
    
    static unsigned int counter = 0;
   
    pthread_mutex_lock (&state->mutex);
    {
      
      state->yaw = msg->gyro[2];


      if(fabs(state->yaw) > 1800){
	printf("stopping map updates\n");
	state->go_map = 0;
	counter = 1;
      }

      if(counter < SETTLE){
	      counter++;
      }
      else{
	state->go_map = 1;
      }

    }
    pthread_mutex_unlock (&state->mutex);
}

static void
pose_xyt_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                  const pose_xyt_t *msg, void *user)
{
    state_t *state = user;

    pthread_mutex_lock (&state->mutex);
    {
            
      memcpy(state->pose,msg,sizeof(pose_xyt_t));
    }
    pthread_mutex_unlock (&state->mutex);
}

static void
rplidar_laser_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                       const rplidar_laser_t *msg, void *user)
{
    state_t *state = user;
    
    pthread_mutex_lock (&state->mutex);
    {
       
      memcpy(&range_data,msg,sizeof(rplidar_laser_t));
    }
    pthread_mutex_unlock (&state->mutex);
}

static void
scanpose_xyt_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                       const scanpose_xyt_t *msg, void *user)
{
    state_t *state = user;
    
    pthread_mutex_lock (&state->mutex);
    {
      memcpy(state->scan_pose,msg,sizeof(scanpose_xyt_t));
    }
    pthread_mutex_unlock (&state->mutex);
}


state_t *
state_create (void)
{
    state_t *state = calloc (1, sizeof (*state));

    state->running = 1;
    state->gopt = getopt_create ();
    state->lcm = lcm_create (NULL);

    state->have_goal = true;
    state->pid_control = false;
    state->vw = vx_world_create ();
    state->app.display_finished = display_finished;
    state->app.display_started = display_started;
    state->app.impl = state;
    state->veh.dispatch_order = -10;
    state->veh.touch_event = touch_event;
    state->veh.mouse_event = mouse_event;
    state->veh.key_event = key_event;
    state->veh.destroy = destroy;
    state->veh.impl = state;
    state->layer_map = zhash_create (sizeof(vx_display_t*), sizeof(vx_layer_t*), zhash_ptr_hash, zhash_ptr_equals);

    state->pose      = (pose_xyt_t*)calloc(1,sizeof(pose_xyt_t));
    state->scan_pose = (scanpose_xyt_t*)calloc(1,sizeof(scanpose_xyt_t));

    
    state->map_size_h   = MAP_HEIGHT;
    state->map_size_w   = MAP_WIDTH;
    state->cell_width   = CELL_WIDTH;
    state->map_startx   = MAP_STARTX;	
    state->map_starty   = MAP_STARTY;
    state->go_map       = 1;
    
    memset(&state->gPath,0,sizeof(path_t));

    // note, pg_sd() family of functions will trigger their own callback of my_param_changed(),
    // hence using a recursive mutex avoids deadlocking when using pg_sd() within my_param_changed()
    pthread_mutexattr_t attr;
    pthread_mutexattr_init (&attr);
    pthread_mutexattr_settype (&attr, PTHREAD_MUTEX_RECURSIVE);
    pthread_mutex_init (&state->mutex, &attr);

    return state;
}


int
main (int argc, char *argv[])
{
    // so that redirected stdout won't be insanely buffered.
    setvbuf (stdout, (char *) NULL, _IONBF, 0);

    vx_global_init ();

    state_t *state = NULL;
    global_state = state = state_create ();

    state->goal[0] = 1.65;
    state->goal[1] = -0.1;

    Init_map(state);
    initialize(state);

    // Clean up on Ctrl+C
    signal (SIGINT, handler);
    signal (SIGQUIT, handler);

    getopt_add_bool (state->gopt, 'h', "help", 0, "Show this help");
    getopt_add_int (state->gopt, 'l', "limitKBs", "-1", "Remote display bandwith limit in KBs. < 0: unlimited.");
    getopt_add_int (state->gopt, 'p', "port", "15151", "Vx display port");
    getopt_add_string (state->gopt, '\0', "maebot-motor-feedback-channel", "MAEBOT_MOTOR_FEEDBACK", "LCM channel name");
    getopt_add_string (state->gopt, '\0', "maebot-sensor-data-channel", "MAEBOT_SENSOR_DATA", "LCM channel name");
    getopt_add_string (state->gopt, '\0', "maebot-diff-drive-channel", "MAEBOT_DIFF_DRIVE", "LCM channel name");
    getopt_add_string (state->gopt, '\0', "odometry-channel", "BOTLAB_ODOMETRY", "LCM channel name");
    getopt_add_string (state->gopt, '\0', "rplidar-laser-channel", "RPLIDAR_LASER", "LCM channel name");


    if (!getopt_parse (state->gopt, argc, argv, 0)) {
        getopt_do_usage (state->gopt);
        exit (EXIT_FAILURE);
    }
    else if (getopt_get_bool (state->gopt,"help")) {
        getopt_do_usage (state->gopt);
        exit (EXIT_SUCCESS);
    }

    // Set up Vx remote display
    vx_remote_display_source_attr_t remote_attr;
    vx_remote_display_source_attr_init (&remote_attr);
    remote_attr.max_bandwidth_KBs = getopt_get_int (state->gopt, "limitKBs");
    remote_attr.advertise_name = "Maebot App";
    remote_attr.connection_port = getopt_get_int (state->gopt, "port");
    vx_remote_display_source_t *remote = vx_remote_display_source_create_attr (&state->app, &remote_attr);

    trajectory = zarray_create(sizeof(trajectory_t));

    ellipse    = zarray_create(16*sizeof(double));

    // Video stuff?

    // LCM subscriptions
    maebot_motor_feedback_t_subscribe (state->lcm,
                                       getopt_get_string (state->gopt, "maebot-motor-feedback-channel"),
                                       maebot_motor_feedback_handler, state);
    maebot_sensor_data_t_subscribe (state->lcm,
                                    getopt_get_string (state->gopt, "maebot-sensor-data-channel"),
                                    maebot_sensor_data_handler, state);
    pose_xyt_t_subscribe (state->lcm,
                          getopt_get_string (state->gopt, "odometry-channel"),
                          pose_xyt_handler, state);
    rplidar_laser_t_subscribe (state->lcm,
                               getopt_get_string (state->gopt, "rplidar-laser-channel"),
                               rplidar_laser_handler, state);
    scanpose_xyt_t_subscribe (state->lcm,
                          "SCAN-REGISTRATION",
                          scanpose_xyt_handler, state);


    // Launch worker threads
    pthread_create (&state->command_thread, NULL, command_thread, state);
    pthread_create (&state->render_thread, NULL, render_thread, state);
   

    // Loop forever
    while (state->running)
        lcm_handle_timeout (state->lcm, 500);

    pthread_join (state->command_thread, NULL);
    pthread_join (state->render_thread, NULL);
    //pthread_join (state->slam_thread, NULL);

    printf ("waiting vx_remote_display_source_destroy...");
    vx_remote_display_source_destroy (remote);
    printf ("done\n");
}



