#ifndef _DSTAR_LITE
#define _DSTAR_LITE

#define MAX_COST 10000

#include "priorityQueue.h"
#include "proj568.h"

//Called from botLab once at the start, sets up some defaults.
void initialize(state_t *state);

//Overall path planning
void pathPlan(state_t *state);

//Subcomponents of path planning.
bool computeShortestPath(state_t *state);
void buildShortestPath(state_t *state, int path[BIG_MAP/2][2]);
void reducePath(state_t *state, int path[BIG_MAP/2][2], int shortPath[BIG_MAP/2][2]);
void convertGridToReal(state_t *state, int path[BIG_MAP/2][2], double pathxy[BIG_MAP/2][2]);
void createGraphicsPath(state_t *state, double smoothedPath[BIG_MAP/2][2]);

//Extra helpers.
double min(double lhs, double rhs);
void calcKey(state_t *state, int x, int y, double key[2]);
void updateVertex(state_t *state, int x, int y);
void successors(state_t *state, int locx, int locy);
int minSuccess(state_t *state, int locx, int locy);
void minSuccessInd(state_t *state);
void successorSelf(state_t *state, int locx, int locy, int gold, bool update);
void convertOccupancy(state_t *state);

#endif