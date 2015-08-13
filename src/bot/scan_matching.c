#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_blas.h>

#include "lcmtypes/pose_xyt_t.h"
#include "lcmtypes/scanpose_xyt_t.h"
#include "lcmtypes/rplidar_laser_t.h"

#include "xyt.h"
#include "filter.h"

#define PI 3.142
#define TRAJ_SIZE 5
//#define DEBUG

typedef struct state state_t;

struct state {
       
    lcm_t *lcm;
    pose_xyt_t *pose;
    pose_xyt_t pose_trajectory[10];
    int traj_ind;

    scanpose_xyt_t *scan_pose;
    pthread_mutex_t mutex;
    rplidar_laser_t range_data;

    mvag_filter_t t_filt_mvag;
};


static double diffnorm(double *x1, double *x2,int n){

    double sum = 0;

    for(int i=0;i<n;i++){
		sum += (x1[i] - x2[i]) * (x1[i] - x2[i]) ;
    }

    return sqrt(sum);

}

static void
MatVecProd(double *rbt, double *vec1,double *vec2){

    gsl_matrix *T    =  gsl_matrix_calloc(3,3);
    gsl_vector *V    =  gsl_vector_calloc(3);
    gsl_vector *V2   =  gsl_vector_calloc(3);


    double Tij[9]    = {cos(rbt[2]),-sin(rbt[2]),rbt[0],
			sin(rbt[2]), cos(rbt[2]), rbt[1],
			0, 0, 1};		

   
			
    memcpy(T->data,Tij,sizeof(double)*9);
    memcpy(V->data,vec1,sizeof(double)*3);
			
    gsl_blas_dgemv(CblasNoTrans,1.0,T,V,0.0,V2);

    memcpy(vec2,V2->data,sizeof(double)*3);
    
    gsl_matrix_free(T);
    gsl_vector_free(V);
    gsl_vector_free(V2);
}

static
void ScanMatchingB(double *xyt_scan,double *x_jk,double* A,double* B,
		   int nrangesA,int nrangesB){
    

    double T0[3];
    double T1[3]  = {0.5,0.5,0.5};
    int mi[500]; 
    int wi[500];

    memcpy(T0,x_jk,sizeof(double)*3);

    double vec_b[3],vec_a[3],vec_ac[3];
    
    double dmax = 0.05;
    double m[3];

    int total_match = 0;

    while(diffnorm(T0,T1,3) > 1e-6){

	#ifdef DEBUG
	printf("diff error = %f, [%f %f %f]\n",diffnorm(T0,T1,3),T0[0],T0[1],T0[2]);
	#endif
	
	memset(mi,0,250*sizeof(int));
	memset(wi,0,250*sizeof(int));

	total_match = 0;

	for(int i=0;i<nrangesB;i++){

	    double x_b = B[i*2+0];
	    double y_b = B[i*2+1];

	    vec_b[0]  = x_b; vec_b[1]  = y_b; vec_b[2]  = 1;
	    
	    MatVecProd(T0,vec_b,vec_ac);

	    double min_val = 1e6;

	    for(int j=0;j<nrangesA;j++){

		double x_a = A[j*2 + 0];
		double y_a = A[j*2 + 1];

		vec_a[0]=x_a; vec_a[1] = y_a; vec_a[2] = 1;
		
		double dist = diffnorm(vec_ac,vec_a,3);
		
		if(dist < min_val){
		    min_val      = dist;
		    mi[i]        = j;
		   
		}	
	    }
	    
	    m[0] = A[ mi[i]*2 + 0 ];
	    m[1] = A[ mi[i]*2 + 1 ];
	    m[2] = 1;
	    
	    if(diffnorm(m,vec_ac,2) <= dmax)
		wi[i]  = 1;
	    else
		wi[i]  = 0;

	    total_match +=  wi[i];
	}
	
	memcpy(T1,T0,sizeof(double)*3);
	
	double Sxk  = 0,   Syk  = 0;
	double Sxk1 = 0,   Syk1 = 0;
	double Sxkxk1 =0, Sxkyk1 = 0;
	double Sykxk1 = 0, Sykyk1 = 0;
     
	for(int i=0;i<nrangesB;i++){

		
	    double x_k = A[mi[i]*2 + 0];
	    double y_k = A[mi[i]*2 + 1];;

	    double x_k1 = B[i*2+0];
	    double y_k1 = B[i*2+1];

	    Sxk  += wi[i]*x_k;
	    Syk  += wi[i]*y_k;

	    Sxk1 += wi[i]*x_k1;
	    Syk1 += wi[i]*y_k1;

	    Sxkxk1 += wi[i] * x_k*x_k1;
	    Sykyk1 += wi[i] * y_k*y_k1;

	    Sxkyk1 += wi[i] * x_k*y_k1;
	    Sykxk1 += wi[i] * y_k*x_k1;

	}
	
	double tan_deltNr = Sxk*Syk1 + total_match*Sykxk1 - total_match*Sxkyk1 - Sxk1*Syk;
	double tan_deltDr = total_match*Sxkxk1 + total_match*Sykyk1 - Sxk*Sxk1 - Syk*Syk1;

	//double tan_delt   = tan_deltNr/tan_deltDr;
	
	double del_t = atan2(tan_deltNr,tan_deltDr);
	double del_x = (Sxk - cos(del_t)*Sxk1 + sin(del_t)*Syk1)/total_match;
	double del_y = (Syk - sin(del_t)*Sxk1 - cos(del_t)*Syk1)/total_match;

	T0[0] = del_x;
	T0[1] = del_y;
	T0[2] = del_t;

    
    }
    
    memcpy(xyt_scan,T0,sizeof(double)*3);

   
}


static void
GetNearesetOdometryPose(double *X_ij,state_t *state){


    int64_t lidar_time = state->range_data.utime;

    int j = 4;
    int64_t pose_time = state->pose_trajectory[j].utime;

    while(pose_time > lidar_time && j >= 0 ){
	j = j-1;
	if(j>=0)
	    pose_time = state->pose_trajectory[j].utime;
    }

    if(j<4){
	int64_t dT  = state->pose_trajectory[j+1].utime - state->pose_trajectory[j].utime;
	double dx   = state->pose_trajectory[j+1].xyt[0] - state->pose_trajectory[j].xyt[0];
	double dy   = state->pose_trajectory[j+1].xyt[1] - state->pose_trajectory[j].xyt[1];
	double dt   = state->pose_trajectory[j+1].xyt[2] - state->pose_trajectory[j].xyt[2];

	int64_t T1   =  state->pose_trajectory[j].utime;
	double x1    =  state->pose_trajectory[j].xyt[0];
	double y1    =  state->pose_trajectory[j].xyt[1];
	double t1    =  state->pose_trajectory[j].xyt[2];
	    
	X_ij[0] = x1 + dx/dT*(lidar_time - T1);
	X_ij[1] = y1 + dy/dT*(lidar_time - T1);
	X_ij[2] = t1 + dt/dT*(lidar_time - T1);
	
    }
    else{
	X_ij[0] = state->pose_trajectory[j].xyt[0];
	X_ij[1] = state->pose_trajectory[j].xyt[1];
	X_ij[2] = state->pose_trajectory[j].xyt[2];
    }
    
}

static void
pose_xyt_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                  const pose_xyt_t *msg, void *user)
{
    state_t *state = user;

    memcpy(state->pose,msg,sizeof(pose_xyt_t));

    
    for(int i=0;i<4;i++){
	memcpy(&state->pose_trajectory[i],&state->pose_trajectory[i+1],sizeof(pose_xyt_t));
    }    
    memcpy(&state->pose_trajectory[4],state->pose,sizeof(pose_xyt_t));
    
}

static void
rplidar_laser_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                       const rplidar_laser_t *msg, void *user)
{

    state_t *state = user;

    static unsigned char first_scan = 1;
    static double dataA[500*2];
    static double dataB[500*2];
    static int counter = 0;
    static double X_ij[3];

    double xyt_scan[3] = {0,0,0};

    memcpy(&state->range_data,msg,sizeof(rplidar_laser_t));
    
    
    int rangesA,rangesB;
              
    rangesB = 290;
    rangesA = 290;


    for(int i=0;i<rangesB;i++){
	dataB[i*2+0] = state->range_data.ranges[i]*cos(-state->range_data.thetas[i]);
	dataB[i*2+1] = state->range_data.ranges[i]*sin(-state->range_data.thetas[i]);	    
    }

    if(first_scan){
	    
	first_scan = 0;	
	memcpy(dataA,dataB,sizeof(double)*500*2);
    }
    counter++;
      
                
    double X_ik[3];
    double X_jk[3];
    double J_tail[3*6];

    GetNearesetOdometryPose(X_ik,state);

    //X_ik[2] = mv_avg_filter(X_ik[2],&state->t_filt_mvag);
    
    //memcpy(X_ik,state->pose->xyt,sizeof(double)*3);
    xyt_tail2tail(X_jk,J_tail,X_ij,X_ik);
    memcpy(X_ij,X_ik,sizeof(double)*3);

    ScanMatchingB(xyt_scan,X_jk,dataA,dataB,rangesA,rangesB);
	       
    if( isnan(xyt_scan[0]) || isnan(xyt_scan[1]) || isnan(xyt_scan[2]) ){
	xyt_scan[0] = 0.0;
	xyt_scan[1] = 0.0;
	xyt_scan[2] = 0.0;
	printf("NAN detected in scan solution\n");
    } 


    if( fabs(X_jk[0]) < 1e-3 && fabs(X_jk[1]) < 1e-3){
	xyt_scan[0] = 0.0;
	xyt_scan[1] = 0.0;
	xyt_scan[2] = 0.0;
    }

    for(int i=0;i<rangesB;i++){
	dataA[i*2+0] = dataB[i*2+0];
	dataA[i*2+1] = dataB[i*2+1];	    
    }

    double *J_plus       = NULL;
    double Xscan_ij[3];
    double Xscan_ik[3];
	    
    memcpy(Xscan_ij,state->scan_pose->xyt,sizeof(double)*3);
    xyt_head2tail(Xscan_ik,J_plus, Xscan_ij,xyt_scan);
    memcpy(state->scan_pose->xyt,Xscan_ik,sizeof(double)*3);

    //Throw away scan matching angle and use gyro
    state->scan_pose->xyt[2] = X_ik[2];

    state->scan_pose->utime = state->range_data.utime;
    scanpose_xyt_t_publish (state->lcm,"SCAN-REGISTRATION", state->scan_pose);

    
}


int main(){

    setvbuf (stdout, (char *) NULL, _IONBF, 0);

    state_t *state         = calloc (1, sizeof *state);
    state->pose            = (pose_xyt_t*)calloc(1,sizeof(pose_xyt_t));
    state->scan_pose       = (scanpose_xyt_t*)calloc(1,sizeof(scanpose_xyt_t));
    

    memset(&state->t_filt_mvag,0,sizeof(mvag_filter_t));
    state->t_filt_mvag.window = 0;

    state->lcm = lcm_create (NULL);
        
    pose_xyt_t_subscribe (state->lcm,
                          "BOTLAB_ODOMETRY",
                          pose_xyt_handler, state);

    rplidar_laser_t_subscribe (state->lcm,
                               "RPLIDAR_LASER",
                               rplidar_laser_handler, state);

    while (1){
        lcm_handle (state->lcm);
    }
}




