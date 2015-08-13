#include "smooth.h"
#include <stdio.h> //printf
#include <stdlib.h> //malloc, free.
#include <math.h> //sqrt


void computeControlPoints(double K[BIG_MAP/2][2], double p1[BIG_MAP/2][2], double p2[BIG_MAP/2][2], int n)
{
	//Rhs vector
	double a[n][2];
	double b[n][2];
	double c[n][2];
	double r[n][2];
	double m;

	//Compute for both x and y direction.
	for(int j = 0; j < 2; ++j) {

		//Left Most Segment
		a[0][j] = 0;
		b[0][j] = 2;
		c[0][j] = 1;
		r[0][j] = K[0][j] + 2.*K[1][j];

		//Internal Segments
		for(int i = 1; i < n-1; i++) {
			a[i][j] = 1;
			b[i][j] = 4;
			c[i][j] = 1;
			r[i][j] = 4.*K[i][j]+2.*K[i+1][j];
		}

		//Right segment
		a[n-1][j] = 2;
		b[n-1][j] = 7;
		c[n-1][j] = 0;
		r[n-1][j] = 8.*K[n-1][j]+K[n][j];

		//Solve Ax=b using the Thomas algorithm
		for(int i = 1; i < n; i++) {
			m = a[i][j]/b[i-1][j];
			b[i][j] = b[i][j] - m*c[i-1][j];
			r[i][j] = r[i][j] - m*r[i-1][j];
		}

		p1[n-1][j] = r[n-1][j]/b[n-1][j];
		for(int i = n-2; i >= 0; --i) {
			p1[i][j] = (r[i][j] - (c[i][j]*p1[i+1][j]))/b[i][j];
		}

		//We have p1, now compute p2
		for(int i = 0; i < n-1; i++) {
			p2[i][j] = 2.*K[i+1][j] - p1[i+1][j];
		}
		p2[n-1][j] = 0.5*(K[n][j]+p1[n-1][j]);
	}
}

int computeTrajectory(double pathSeg[BIG_MAP/2][2], double xy[BIG_MAP/2][2], double p1[BIG_MAP/2][2], double p2[BIG_MAP/2][2], int n)
{
	double B,diff1,diff2,distance,uj;
	double controlPoints[4][2];
	int numPoints;
	int totalNumPoints = 0, index;
	double factorials[4] = {1, 3, 3, 1};
	//Compute each piece of trajectory independently.
	for(int k = 0; k < n; ++k) {
		//Extract the control points for the kth path segment.
		controlPoints[0][0] = xy[k][0];
		controlPoints[0][1] = xy[k][1];
		controlPoints[1][0] = p1[k][0];
		controlPoints[1][1] = p1[k][1];
		controlPoints[2][0] = p2[k][0];
		controlPoints[2][1] = p2[k][1];
		controlPoints[3][0] = xy[k+1][0];
		controlPoints[3][1] = xy[k+1][1];
		//Compute distance between points being connected.
		diff1 = (xy[k][0]-xy[k+1][0]);
		diff2 = (xy[k][1]-xy[k+1][1]);
		distance = sqrt(diff1*diff1 + diff2*diff2);
		//Use the distance to determine how many ponts are needed.
		numPoints = distance/TRAJECTORY_RESOLUTION;
		//Loop through the points and calculate bezier curve.
		for(double j = 0; j < numPoints; ++j) {
			uj = j/numPoints;
			index = j + totalNumPoints;
			pathSeg[index][0] = 0;
			pathSeg[index][1] = 0;
			for(int i = 0; i <= 3; ++i) {
				B = factorials[i]*pow(uj,i)*pow((1.-uj),3-i);
				pathSeg[index][0] += B * controlPoints[i][0];
				pathSeg[index][1] += B * controlPoints[i][1];
			}
		//	printf("pathSeg[%d] = %f, %f\n",index,pathSeg[index][0],pathSeg[index][1]);

		}
		totalNumPoints += numPoints;
	}
	//printf("\n\n");
	return totalNumPoints;
}

void smoothPath(state_t *state, double pathxy[BIG_MAP/2][2], double smoothedPath[BIG_MAP/2][2])
{
	//Path smoothing steps.
	int numControlPointsGenerated = state->gPath.pathLength - 1;
	double p1[BIG_MAP/2][2];
	double p2[BIG_MAP/2][2];
	computeControlPoints(pathxy,p1,p2,numControlPointsGenerated);
	state->gPath.pathLength = computeTrajectory(smoothedPath,pathxy,p1,p2,numControlPointsGenerated);
}

Control_t findControlPoints(double s1[2], double s2[2], double s3[2])
{
	double dx1 = s1[0] - s2[0];
	double dy1 = s1[1] - s2[1];
	double dx2 = s2[0] - s3[0];
	double dy2 = s2[1] - s3[1];

	double l1 = sqrt(dx1*dx1 + dy1*dy1);
	double l2 = sqrt(dx2*dx2 + dy2*dy2);

	double m1[2], m2[2];
	m1[0] = (s1[0] + s2[0])/2.0;
	m1[1] = (s1[1] + s2[1])/2.0;
	m2[0] = (s2[0] + s3[0])/2.0;
	m2[1] = (s2[1] + s3[1])/2.0;

	double dxm = (m1[0] - m2[0]);
	double dym = (m1[1] - m2[1]);

	double k = l2 / (l1 + l2);
	double cm[2];
	cm[0] = m2[0] + dxm*k;
	cm[1] = m2[1] + dym*k;
	double tx = s2[0] - cm[0];
	double ty = s2[1] - cm[1];

	Control_t ret;
	ret.c1[0] = m1[0] + tx;
	ret.c1[1] = m1[1] + ty;
	ret.c2[0] = m2[0] + tx;
	ret.c2[1] = m2[1] + ty;
	ret.l1 = l1;
	ret.l2 = l2;

	return ret;
}
/*
void makeBezierQuad()
{
	Control_t S1 = findControlPoints(s1,s2,s3)
}
*/
