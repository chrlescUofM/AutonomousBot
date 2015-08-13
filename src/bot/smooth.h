#ifndef _SMOOTH
#define _SMOOTH

#include "proj568.h"

#define TRAJECTORY_RESOLUTION 0.01f //Resolution in meters between trajectory points.

typedef struct {
	double c1[2];
	double c2[2];
	int l1;
	int l2;
} Control_t;

void computeControlPoints(double K[BIG_MAP/2][2], double p1[BIG_MAP/2][2], double p2[BIG_MAP/2][2], int n);

int computeTrajectory(double pathSeg[BIG_MAP/2][2], double xy[BIG_MAP/2][2],
					  double p1[BIG_MAP/2][2], double p2[BIG_MAP/2][2], int n);
void smoothPath(state_t *state, double pathxy[BIG_MAP/2][2], double smoothedPath[BIG_MAP/2][2]);

#endif