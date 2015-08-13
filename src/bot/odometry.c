#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

//#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include "common/getopt.h"
#include "math/gsl_util_matrix.h"
#include "math/gsl_util_blas.h"
#include "math/math_util.h"
#include "math/gsl_util_math.h"

#include <gsl/gsl_linalg.h>

#include "lcmtypes/maebot_motor_feedback_t.h"
#include "lcmtypes/maebot_sensor_data_t.h"
#include "lcmtypes/pose_xyt_t.h"

#include "xyt.h"

#define ALPHA_STRING          "1.0"  // longitudinal covariance scaling factor
#define BETA_STRING           "1.0"  // lateral side-slip covariance scaling factor
#define ALPHA                  0.1
#define BETA                   0.001
#define GYRO_RMS_STRING       "1.0"    // [deg/s]
#define TICKS_PER_METER       4456.0
#define METERS_PER_TICK       (1.0/TICKS_PER_METER)
#define BASELINE               0.08635 //meters
#define GYRO_COV               0.000003

typedef struct state state_t;
struct state {
    getopt_t *gopt;
  
    lcm_t *lcm;
    const char *odometry_channel;
    const char *feedback_channel;
    const char *sensor_channel;

    // odometry params
    double meters_per_tick; // conversion factor that translates encoder pulses into linear wheel displacement
    double alpha;
    double beta;
    double gyro_rms;

    bool use_gyro;
    int64_t dtheta_utime;
    double dtheta;
    double dtheta_sigma;

    double xyt[3]; // 3-dof pose
    double Sigma[3*3];

    double left_ticks;
    double right_ticks;

    int yaw_cnt;
    int64_t yaw_cal_array[100];
    int64_t gyro_bias;
    int64_t yaw_int_0;

    unsigned char yaw_calibrated;
    double yaw;
    double yaw_old;
    

    FILE *fp;
};



int64_t find_gyro_bias(state_t *state);

gsl_matrix* matrix_multiply(gsl_matrix* matrix1, gsl_matrix* matrix2){

  gsl_matrix *matrix3;
  int i, j, k;

  // checks to see if the dimensions of matrix 1 and 2 are compatible for matrix multiplication
  if (matrix1->size2 != matrix2->size1){
    printf("Cannot multiply due to incompatitble matrix dimensions.\n\n");
    matrix3 = NULL;
  }
  else {
    // the resulting matrix has the same number of rows as matrix 1 and the same 
    // number of columns as matrix 2
    matrix3 = gsl_matrix_alloc(matrix1->size1, matrix2->size2); 
    gsl_matrix_set_zero(matrix3);

    // apply the definition of matrix multiplication
    for (k=0; k<(matrix2->size2);k++){
      for (j=0; j<(matrix1->size1);j++){
	for (i=0; i<(matrix1->size2);i++){
	  gsl_matrix_set(matrix3,j,k, gsl_matrix_get(matrix3,j,k) + (gsl_matrix_get(matrix1,j,i)*gsl_matrix_get(matrix2,i,k)));
	  
	}
      }
    }
  }

  return matrix3;

}


void print_matrix(const gsl_matrix* matrix){
  int i,j;

  printf("Number of Rows in Matrix: %d\n",(int)matrix->size1);
  printf("Number of Columns in Matrix: %d\n", (int)matrix->size2);

  // iterates through the matrix to print out each value
  for (i=0;i<(matrix->size1);i++){
    for (j=0;j<(matrix->size2);j++){
      printf("%f\t", gsl_matrix_get(matrix,i,j));
    }
    printf("\n");
  }
  printf("\n");

}


void print_vector(const gsl_vector* vector){
  int i;

  printf("Number of Elements in Vector: %d\n",(int)vector->size);

  // iterates through the matrix to print out each value
  for (i=0;i<(vector->size);i++){
    if(i==2){
      printf("%f\t", 180/M_PI*gsl_vector_get(vector,i));
      printf("\n");
    } else{
      printf("%f\t", gsl_vector_get(vector,i));
      printf("\n");
    }
  }
  printf("\n");

}


void update_position(state_t *state, double left_ticks, double right_ticks){

    printf("updating position\n");

    double dl, dr, delta_x, delta_y, delta_theta;

    // IMPLEMENT ME
 
    gsl_matrix *J_plus = gsl_matrix_calloc(3,6);
    gsl_vector *old_pose = gsl_vector_calloc(3);
    gsl_vector *change_pose = gsl_vector_calloc(3);
    gsl_vector *new_pose = gsl_vector_calloc(3);
    gsl_matrix *old_sigma = gsl_matrix_calloc(3,3);
    gsl_matrix *change_sigma = gsl_matrix_calloc(3,3);
    gsl_matrix *new_sigma = gsl_matrix_calloc(3,3);

    memcpy(old_pose->data, state->xyt, sizeof state->xyt);
    memcpy (old_sigma->data, state->Sigma, sizeof state->Sigma); 


    static unsigned char first_feedback = 0;

    if(first_feedback == 0){
      state->left_ticks  = left_ticks;
      state->right_ticks = right_ticks;
      first_feedback = 1;
    }

   
    // calculate new expected value
    dl = METERS_PER_TICK * (left_ticks - state->left_ticks);
    dr = METERS_PER_TICK * (right_ticks - state->right_ticks);

    delta_x = (dl + dr)/2.0;
    delta_y = 0;
    delta_theta = gslu_math_mod2pi((dr - dl)/BASELINE);
   
    gsl_vector_set(old_pose,2,gslu_math_mod2pi(old_pose->data[2]));

    gsl_vector_set(change_pose,0,delta_x);
    gsl_vector_set(change_pose,1,delta_y); 
    gsl_vector_set(change_pose,2,delta_theta);

    //printf("delx,dely,delT = %f,%f,%f\n",delta_x,delta_y,delta_theta);

    xyt_head2tail_gsl(new_pose, J_plus, old_pose, change_pose);
    
    gsl_vector_set(new_pose,2,gslu_math_mod2pi(new_pose->data[2]));

    //printf("dl,dr,theta = %f,%f,%f\n",dl,dr,new_pose->data[2]);
   
    //printf("x,y,theta = %f,%f,%f\n",new_pose->data[0],new_pose->data[1],new_pose->data[2]);

    // calculate new covariance
    gsl_matrix_set(change_sigma, 0,0, (ALPHA/4.0)*(fabs(dl) + fabs(dr)));
    gsl_matrix_set(change_sigma, 0,2, (ALPHA/(2*BASELINE))*(fabs(dr) - fabs(dl)));
    gsl_matrix_set(change_sigma, 1,1, BETA * fabs(dl + dr));
    gsl_matrix_set(change_sigma, 2,0, (ALPHA/(2*BASELINE))*(fabs(dr) - fabs(dl)));
    gsl_matrix_set(change_sigma, 2,2, (ALPHA/(BASELINE*BASELINE))*(fabs(dl) + fabs(dr)));

    //gsl_matrix_set(change_sigma, 2,2, 0);

    //printf("sig theta = %f\n",(ALPHA/(BASELINE*BASELINE))*(fabs(dl) + fabs(dr)));

    gsl_matrix *J_plus1 = gsl_matrix_calloc(3,3);
    gsl_matrix *J_plus2 = gsl_matrix_calloc(3,3);
    gsl_vector *column = gsl_vector_calloc(3);
   
    // set up J_plus1 and Jplus2
    gsl_matrix_get_col(column, J_plus, 0); 
    gsl_matrix_set_col(J_plus1, 0 ,column);
    gsl_matrix_get_col(column, J_plus, 1); 
    gsl_matrix_set_col(J_plus1, 1 ,column);
    gsl_matrix_get_col(column, J_plus, 2); 
    gsl_matrix_set_col(J_plus1, 2 ,column);

    gsl_matrix_get_col(column, J_plus, 3); 
    gsl_matrix_set_col(J_plus2, 0 ,column);
    gsl_matrix_get_col(column, J_plus, 4); 
    gsl_matrix_set_col(J_plus2, 1 ,column);
    gsl_matrix_get_col(column, J_plus, 5); 
    gsl_matrix_set_col(J_plus2, 2 ,column);

    // get transposes
    gsl_matrix *J_plus1_T = gsl_matrix_calloc(3,3);
    gsl_matrix *J_plus2_T = gsl_matrix_calloc(3,3);

    gsl_matrix_transpose_memcpy(J_plus1_T, J_plus1);
    gsl_matrix_transpose_memcpy(J_plus2_T, J_plus2);

    // multiply to get the covariances
    
    gsl_matrix *temp1 = gsl_matrix_calloc(3,3);
    gsl_matrix *temp2 = gsl_matrix_calloc(3,3);
    gsl_matrix *sigma_temp = gsl_matrix_calloc(3,3);

    temp1 = matrix_multiply(J_plus1, old_sigma);
    new_sigma = matrix_multiply(temp1, J_plus1_T);
    
    gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,1.0, J_plus1, old_sigma,0.0, temp1);

    gsl_blas_dgemm (CblasNoTrans, CblasTrans,1.0, temp1, J_plus1,0.0, new_sigma);

    gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,1.0, J_plus2, change_sigma,0.0, temp2);

    gsl_blas_dgemm (CblasNoTrans, CblasTrans,1.0, temp2, J_plus2,0.0, sigma_temp);


    printf("J1EJ1:\n");
    print_matrix(new_sigma);
    
    //temp3 = matrix_multiply(J_plus2, change_sigma);
    //temp4 = matrix_multiply(temp3, J_plus2_T);
 
    printf("J2EJ2\n");
    print_matrix(sigma_temp);
    
    gsl_matrix_add(new_sigma, sigma_temp);

    // gsl_matrix_set(new_sigma,2,2,fabs(gslu_math_mod2pi(new_sigma->data[8])));

    state->left_ticks = left_ticks;
    state->right_ticks = right_ticks;
    


    // update state
    memcpy(state->xyt, new_pose->data, sizeof state->xyt);
    memcpy(state->Sigma, new_sigma->data, sizeof state->Sigma);


    // prints for testing

    //printf("Old Position:\n");
    //print_vector(old_pose);
    //printf("Change Position:\n");
    //print_vector(change_pose);
    
    
    printf("Jacobian Plus:\n");
    print_matrix(J_plus);

    printf("Old Sigma:\n");
    print_matrix(old_sigma);
    printf("Change Sigma:\n");
    print_matrix(change_sigma);

    printf("New Sigma:\n");
    print_matrix(new_sigma);

    printf("New Position:\n");
    print_vector(new_pose);
 
    gsl_matrix_free(J_plus);
    gsl_matrix_free(J_plus1);
    gsl_matrix_free(J_plus2);

    gsl_matrix_free(temp1);
    gsl_matrix_free(temp2);
    gsl_matrix_free(sigma_temp);

    gsl_matrix_free(old_sigma);
    gsl_matrix_free(change_sigma);
    gsl_matrix_free(new_sigma);

    gsl_vector_free(old_pose);
    gsl_vector_free(change_pose);
    gsl_vector_free(new_pose);
  

  
    return;

}

static void
motor_feedback_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                        const maebot_motor_feedback_t *msg, void *user)
{

    state_t *state = user;
    double dl, dr, delta_x, delta_y, delta_theta;

    // IMPLEMENT ME
    gsl_matrix *J_plus       = gsl_matrix_calloc(3,6);
    gsl_vector *old_pose     = gsl_vector_calloc(3);
    gsl_vector *change_pose  = gsl_vector_calloc(3);
    gsl_vector *new_pose     = gsl_vector_calloc(3);
    gsl_matrix *old_sigma    = gsl_matrix_calloc(3,3);
    gsl_matrix *change_sigma = gsl_matrix_calloc(3,3);
    gsl_matrix *new_sigma    = gsl_matrix_calloc(3,3);
  

    memcpy(old_pose->data, state->xyt, sizeof state->xyt);
    memcpy (old_sigma->data, state->Sigma, sizeof state->Sigma); 

    static unsigned char first_feedback = 0;

    if(first_feedback == 0){
      state->left_ticks  = msg->encoder_left_ticks;
      state->right_ticks = msg->encoder_right_ticks;
      first_feedback = 1;
    }
   
    // calculate new expected value
    dl = METERS_PER_TICK * (msg->encoder_left_ticks - state->left_ticks);
    dr = METERS_PER_TICK * (msg->encoder_right_ticks - state->right_ticks);

    delta_x = (dl + dr)/2.0;
    delta_y = 0;

    if(state->yaw_calibrated)
	delta_theta = (state->yaw - state->yaw_old)*M_PI/180;
    else
	delta_theta = gslu_math_mod2pi((dr - dl)/BASELINE);
   
    gsl_vector_set(old_pose,2,gslu_math_mod2pi(old_pose->data[2]));

    gsl_vector_set(change_pose,0,delta_x);
    gsl_vector_set(change_pose,1,delta_y); 
    gsl_vector_set(change_pose,2,delta_theta);

  

    xyt_head2tail_gsl(new_pose, J_plus, old_pose, change_pose);
    
    gsl_vector_set(new_pose,2,gslu_math_mod2pi(new_pose->data[2]));
   
    // calculate new covariance
    gsl_matrix_set(change_sigma, 0,0, (ALPHA/4.0)*(fabs(dl) + fabs(dr)));
   
    gsl_matrix_set(change_sigma, 1,1, BETA * fabs(dl + dr));
    
    if (state->yaw_calibrated){
      gsl_matrix_set(change_sigma, 2,2, GYRO_COV);     
    }
     else{
      gsl_matrix_set(change_sigma, 2,2, (ALPHA/(BASELINE*BASELINE))*(fabs(dl) + fabs(dr)));
      gsl_matrix_set(change_sigma, 0,2, (ALPHA/(2*BASELINE))*(fabs(dr) - fabs(dl)));
      gsl_matrix_set(change_sigma, 2,0, (ALPHA/(2*BASELINE))*(fabs(dr) - fabs(dl)));
    }
    gsl_matrix *J_plus1 = gsl_matrix_calloc(3,3);
    gsl_matrix *J_plus2 = gsl_matrix_calloc(3,3);
    gsl_vector *column = gsl_vector_calloc(3);
   
    // set up J_plus1 and Jplus2
    gsl_matrix_get_col(column, J_plus, 0); 
    gsl_matrix_set_col(J_plus1, 0 ,column);
    gsl_matrix_get_col(column, J_plus, 1); 
    gsl_matrix_set_col(J_plus1, 1 ,column);
    gsl_matrix_get_col(column, J_plus, 2); 
    gsl_matrix_set_col(J_plus1, 2 ,column);

    gsl_matrix_get_col(column, J_plus, 3); 
    gsl_matrix_set_col(J_plus2, 0 ,column);
    gsl_matrix_get_col(column, J_plus, 4); 
    gsl_matrix_set_col(J_plus2, 1 ,column);
    gsl_matrix_get_col(column, J_plus, 5); 
    gsl_matrix_set_col(J_plus2, 2 ,column);

  
    // multiply to get the covariances
    
    gsl_matrix *temp1 = gsl_matrix_calloc(3,3);
    gsl_matrix *temp2 = gsl_matrix_calloc(3,3);
    gsl_matrix *sigma_temp = gsl_matrix_calloc(3,3);
   
    gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,1.0, J_plus1, old_sigma,0.0, temp1);

    gsl_blas_dgemm (CblasNoTrans, CblasTrans,1.0, temp1, J_plus1,0.0, new_sigma);

    gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,1.0, J_plus2, change_sigma,0.0, temp2);

    gsl_blas_dgemm (CblasNoTrans, CblasTrans,1.0, temp2, J_plus2,0.0, sigma_temp);

    
    gsl_matrix_add(new_sigma, sigma_temp);

    // update state
    memcpy(state->xyt, new_pose->data, sizeof state->xyt);
    memcpy(state->Sigma, new_sigma->data, sizeof state->Sigma);

    
    // update state
    memcpy(state->xyt, new_pose->data, sizeof state->xyt);
    memcpy(state->Sigma, new_sigma->data, sizeof state->Sigma);

    // print_matrix(new_sigma);
    state->left_ticks = msg->encoder_left_ticks;
    state->right_ticks = msg->encoder_right_ticks;
    
    // publish
    pose_xyt_t odo = { .utime = msg->utime };
    memcpy (odo.xyt, state->xyt, sizeof state->xyt);
    memcpy (odo.Sigma, state->Sigma, sizeof state->Sigma);
    pose_xyt_t_publish (state->lcm, state->odometry_channel, &odo);

    gsl_matrix_free(J_plus);
    gsl_matrix_free(J_plus1);
    gsl_matrix_free(J_plus2);

    gsl_matrix_free(temp1);
    gsl_matrix_free(temp2);
    gsl_matrix_free(sigma_temp);

    gsl_matrix_free(old_sigma);
    gsl_matrix_free(change_sigma);
    gsl_matrix_free(new_sigma);

    gsl_vector_free(old_pose);
    gsl_vector_free(change_pose);
    gsl_vector_free(new_pose);
  
    

}


int64_t find_gyro_bias(state_t *state){

  // used linear least squares approximation to get slope

  gsl_matrix *A = gsl_matrix_calloc(100,2);
  gsl_matrix *AT = gsl_matrix_calloc(2,100);
  gsl_matrix *ATA;
  gsl_matrix *ATA_inv = gsl_matrix_calloc(2,2);
  gsl_matrix *ATA_inv_AT;
  gsl_matrix *x;
  gsl_matrix *b = gsl_matrix_calloc(100,1);
  gsl_permutation *perm = gsl_permutation_alloc(2);

  int i, s;
  for (i=0;i<100;i++){
     gsl_matrix_set(A,i,0,1);
     gsl_matrix_set(A,i,1,i);
     gsl_matrix_set(b,i,0,state->yaw_cal_array[i]);
  }
  print_matrix(A);
  gsl_matrix_transpose_memcpy(AT, A);
  ATA = matrix_multiply(AT,A);
  
  gsl_linalg_LU_decomp(ATA, perm, &s);
  print_matrix(ATA);
  gsl_linalg_LU_invert(ATA, perm, ATA_inv);
  ATA_inv_AT = matrix_multiply(ATA_inv, AT);
  print_matrix(ATA_inv_AT);
  x = matrix_multiply(ATA_inv_AT,b);
  
  print_matrix(x);
  print_matrix(b);

  return (int64_t)gsl_matrix_get(x,1,0);
}



static void
sensor_data_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                     const maebot_sensor_data_t *msg, void *user)
{
    state_t *state = user;
    int64_t yaw_int;

    if (!state->use_gyro)
        return;

   
    // fprintf(state->fp,"%llu,",msg->gyro_int[2]);
    // fflush(state->fp);
    // IMPLEMENT ME

    static unsigned char first_feedback = 1;
           
    if (state->yaw_calibrated == 0){
      state->yaw_cal_array[state->yaw_cnt] = msg->gyro_int[2];
 
      if (state->yaw_cnt == 99){
	state->gyro_bias = find_gyro_bias(state);
	state->yaw_calibrated = 1;
	//printf("Yaw Calibrated\nBias: %lu",state->gyro_bias);
      }

      
    } else {

	state->yaw_old = state->yaw;

        yaw_int = msg->gyro_int[2] - ((int64_t)state->yaw_cnt * state->gyro_bias);

	if(first_feedback){
	    state->yaw_int_0 = yaw_int;
	    first_feedback = 0;
	}

	state->yaw = (yaw_int-state->yaw_int_0)/1.0e6 * 1.0/131.0;

	//printf("yaw: %lf, yaw int: %lu, yaw int 0: %lu\n", state->yaw, yaw_int,state->yaw_int_0 ); 
	
	//fprintf(state->fp,"%lf,",state->yaw);

    }

    

    state->yaw_cnt = state->yaw_cnt + 1;
    
   
}



int
main (int argc, char *argv[])
{
    // so that redirected stdout won't be insanely buffered.
    setvbuf (stdout, (char *) NULL, _IONBF, 0);

    state_t *state = calloc (1, sizeof *state);

    state->meters_per_tick = METERS_PER_TICK; // IMPLEMENT ME
    state->alpha = ALPHA;
    state->beta = BETA;

    // file for gyro integration test
    //state->fp = fopen("gyro-yaw-integration.txt","w");
    state->fp = fopen("yaw.txt","w");
    // Tests for the position estimate 
    /*
    update_position(state,4456,4456);
    update_position(state,0,1208);
    update_position(state,4456,4456);
    
    while(1);
    */


    // Used for testing the gyro bias
    /*
    for(int i=0; i<100; i++){
      state->yaw_cal_array[i] = 15*i;
    }

    state->gyro_bias = find_gyro_bias(state); 
    printf("Gyro Bias: %llu\n", state->gyro_bias);
    */


    //printf("getting options\n");
    
    state->gopt = getopt_create ();
    getopt_add_bool   (state->gopt, 'h', "help", 0, "Show help");
    getopt_add_bool   (state->gopt, 'g', "use-gyro", 0, "Use gyro for heading instead of wheel encoders");
    getopt_add_string (state->gopt, '\0', "odometry-channel", "BOTLAB_ODOMETRY", "LCM channel name");
    getopt_add_string (state->gopt, '\0', "feedback-channel", "MAEBOT_MOTOR_FEEDBACK", "LCM channel name");
    getopt_add_string (state->gopt, '\0', "sensor-channel", "MAEBOT_SENSOR_DATA", "LCM channel name");
    getopt_add_double (state->gopt, '\0', "alpha", ALPHA_STRING, "Longitudinal covariance scaling factor");
    getopt_add_double (state->gopt, '\0', "beta", BETA_STRING, "Lateral side-slip covariance scaling factor");
    getopt_add_double (state->gopt, '\0', "gyro-rms", GYRO_RMS_STRING, "Gyro RMS deg/s");

    if (!getopt_parse (state->gopt, argc, argv, 1) || getopt_get_bool (state->gopt, "help")) {
        printf ("Usage: %s [--url=CAMERAURL] [other options]\n\n", argv[0]);
        getopt_do_usage (state->gopt);
        exit (EXIT_FAILURE);
    }

    state->use_gyro = getopt_get_bool (state->gopt, "use-gyro");
    state->odometry_channel = getopt_get_string (state->gopt, "odometry-channel");
    state->feedback_channel = getopt_get_string (state->gopt, "feedback-channel");
    state->sensor_channel = getopt_get_string (state->gopt, "sensor-channel");
    state->alpha = getopt_get_double (state->gopt, "alpha");
    state->beta = getopt_get_double (state->gopt, "beta");
    state->gyro_rms = getopt_get_double (state->gopt, "gyro-rms") * DTOR;
    state->yaw_calibrated = 0;
    state->yaw     = 0;
    state->yaw_old = 0;
    
    //printf("subscribing to channels\n");

    // initialize LCM
    state->lcm = lcm_create (NULL);
    maebot_motor_feedback_t_subscribe (state->lcm, state->feedback_channel,
                                       motor_feedback_handler, state);
    maebot_sensor_data_t_subscribe (state->lcm, state->sensor_channel,
                                    sensor_data_handler, state);

    printf ("ticks per meter: %f\n", 1.0/state->meters_per_tick);

    while (1){
        lcm_handle (state->lcm);
    }
    
    
   
	
}
