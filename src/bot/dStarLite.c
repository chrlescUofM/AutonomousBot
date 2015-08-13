#include "dStarLite.h"
#include "utility.h"
#include "smooth.h"
#include <limits.h>
#include <stdio.h> //printf
#include <stdlib.h> //malloc, free.
#include <math.h> //sqrt


double sum = 0, average = 0, totalSum = 0, numIterations = 0;
//Main path planning function
void pathPlan(state_t *state)
{
	++numIterations;
	//Reset variables needed to calculate the path.
	int path[BIG_MAP/2][2];				//holds the grid based path.
	//int shortPath[BIG_MAP/2][2];		//holds the linear reduced path.
	state->gPath.pathLength = 0;		//Length of grid based path.
	double pathxy[BIG_MAP/2][2];		//holds the real-pos based path.
	//double smoothedPath[BIG_MAP/2][2];	//holds the real-pos, smoothed path.
	
	//Uncomment this to run A* version, completely re-planning each iteration.
	//initialize(state);
	
	//Need to store the previous starting point.
	double startLast[2];
	startLast[0] = state->gPath.startLoc[0];
	startLast[1] = state->gPath.startLoc[1];
	state->gPath.startLoc[1] = round((state->scan_pose->xyt[0] - state->map_startx)/CELL_WIDTH);
	state->gPath.startLoc[0] = round((state->scan_pose->xyt[1] - state->map_starty)/CELL_WIDTH);

	//Make a copy of the previous cost matrix to use to tell which edges to update.
	double oldCost[100][100];
	for(int i = 0; i < 100; ++i) {
		for(int j = 0; j < 100; ++j) {
			oldCost[i][j] = state->gPath.cost[i][j];
		}
	}
	//Convert the occupancy grid probabilities into integer cost values.
	convertOccupancy(state);

	//If this isn't the first time through, update edge waits as needed.
 	if(!state->gPath.first) {
		int x,y;
		double diff1 = state->gPath.startLoc[0] - startLast[0];
    	double diff2 = state->gPath.startLoc[1] - startLast[1];
    	double h = sqrt(diff1*diff1+diff2*diff2);
		state->gPath.km += h;
		for(int i = 0; i < state->gPath.gridHeight; ++i) {
			for(int j = 0; j < state->gPath.gridWidth; ++j) {
				//If the cost has changed we need to handle all cells to this spot.
				if(oldCost[i][j] != state->gPath.cost[i][j]) {
					updateVertex(state,i,j);
				}
			}		
		}
	}
	average = totalSum / (numIterations);
	//printf("Average: %f \t Total: %f\n",average,totalSum);
	//Create all necessary path components.
	if(!computeShortestPath(state)){
		return;
	}

	buildShortestPath(state, path);
	//reducePath(state,path,shortPath);
	convertGridToReal(state, path, pathxy);
	//smoothPath(state,pathxy,smoothedPath);
	createGraphicsPath(state, pathxy);

	//Set first to false after we finish once.
	state->gPath.first = false;

	//Set the parameter needed for PID
	int requiredWP = 0;

	if(state->gPath.pathLength > 5)
	  requiredWP = 4;
	else if(state->gPath.pathLength > 4)
	  requiredWP = 3;
	else if(state->gPath.pathLength > 3)
	  requiredWP = 2;
	else if(state->gPath.pathLength > 2)
	  requiredWP = 1;
	else 
	  requiredWP = 0;

	if(state->gPath.pathLength > 1) {
		state->gPath.nextWaypoint[0] = pathxy[requiredWP][0];
		state->gPath.nextWaypoint[1] = pathxy[requiredWP][1];
	} else {
		printf("No nextWaypoint exists\n");
	}
}

void createGraphicsPath(state_t *state, double smoothedPath[BIG_MAP/2][2])
{
	//Create the path needed for graphics.
	for(int i = 0; i < state->gPath.pathLength-1; ++i) {
		state->pathXY[i*6]   = smoothedPath[i][0];
		state->pathXY[i*6+1] = smoothedPath[i][1];
		state->pathXY[i*6+2] = 0;
		state->pathXY[i*6+3] = smoothedPath[i+1][0];
		state->pathXY[i*6+4] = smoothedPath[i+1][1];
		state->pathXY[i*6+5] = 0;
	}
	//Update path length for graphics.
	state->pathlen = (state->gPath.pathLength-1)*2;
}

void convertGridToReal(state_t *state, int path[BIG_MAP/2][2], double pathxy[BIG_MAP/2][2])
{
	//Convert from grid locations to actual positions.
	for(int i = 0; i < state->gPath.pathLength; ++i) {
		pathxy[i][0] = state->ogmap[path[i][0]*state->gPath.gridWidth + path[i][1]].X;
		pathxy[i][1] = state->ogmap[path[i][0]*state->gPath.gridWidth + path[i][1]].Y;
	}

}

void reducePath(state_t *state, int path[BIG_MAP/2][2], int shortPath[BIG_MAP/2][2])
{
	double diffx1,diffx2,diffy1,diffy2;
	double slope1,slope2;
	int newLength = 0;
	if(state->gPath.pathLength > 0) {
		shortPath[newLength][0] = path[0][0];
		shortPath[newLength++][1] = path[0][1];
	}
	//Remove colinear points, shortening path.
	for(int i = 1; i < state->gPath.pathLength-1 ; ++i) {
		diffx1 = path[i][0] - path[i-1][0];
		diffx2 = path[i+1][0] - path[i][0];
		diffy1 = path[i][1] - path[i-1][1];
		diffy2 = path[i+1][1] - path[i][1];

		slope1 = (diffx1 == 0) ? 0 : (diffy1/diffx1);
		slope2 = (diffx2 == 0) ? 0 : (diffy2/diffx2);

		//If the slope across a point is un-equal, then we still need the point.
		if(slope1 != slope2) {
			shortPath[newLength][0] = path[i][0];
			shortPath[newLength++][1] = path[i][1];
		}
	}
	if(state->gPath.pathLength > 1) {
		shortPath[newLength][0] = path[state->gPath.pathLength-1][0];
		shortPath[newLength++][1] = path[state->gPath.pathLength-1][1];
	}
	state->gPath.pathLength = newLength;
}

void buildShortestPath(state_t *state, int path[BIG_MAP/2][2])
{
	
	while(!(state->gPath.startLoc[0] == state->gPath.goalLoc[0] && state->gPath.startLoc[1] == state->gPath.goalLoc[1])) {
		minSuccessInd(state);
		if(state->gPath.startLoc[0] == path[state->gPath.pathLength-1][0] && state->gPath.startLoc[1] == path[state->gPath.pathLength-1][1]) {
			printf("Path building failed\n");
			exit(1);
		}
		//Add this position to the path
		path[state->gPath.pathLength][0] = state->gPath.startLoc[0];
		path[state->gPath.pathLength++][1] = state->gPath.startLoc[1];
	}

	//Reset the last starting position for use in the next iteration of D* lite
	state->gPath.startLoc[0] = path[0][0];
	state->gPath.startLoc[1] = path[0][1];
}

bool computeShortestPath(state_t *state)
{
	double compKey[2];
	int u[2];
	int gold;
	double kold[2];
	double knew[2];
	calcKey(state,state->gPath.startLoc[0],state->gPath.startLoc[1],compKey);
	Q* top = _priorityQueue_top(&state->gPath.pq);

	//Continue until a path is found
	while((state->gPath.pq.size > 0 && top->key[0] < compKey[0]) ||
			state->gPath.rhs[state->gPath.startLoc[0]][state->gPath.startLoc[1]] !=  state->gPath.g[state->gPath.startLoc[0]][state->gPath.startLoc[1]]) {
		//Increment the total number of nodes expanded thus far.
		totalSum++;
		//Grab the position of the top element.
		u[0] = top->loc[0];
		u[1] = top->loc[1];
		kold[0] = top->key[0];
		kold[1] = top->key[1];
		calcKey(state,u[0],u[1],knew);
		if(kold[0] < knew[0]) {
			top->key[0] = knew[0];
			top->key[1] = knew[1];
		} else if(state->gPath.g[u[0]][u[1]] > state->gPath.rhs[u[0]][u[1]]) {
			state->gPath.g[u[0]][u[1]] = state->gPath.rhs[u[0]][u[1]];
			_priorityQueue_removeLoc(&state->gPath.pq, top->loc[0],top->loc[1]);
			successors(state,u[0],u[1]);
		} else {
			gold = state->gPath.g[u[0]][u[1]];
			state->gPath.g[u[0]][u[1]] = MAX_COST;
			successorSelf(state,u[0],u[1],gold,false);
		}
		top = _priorityQueue_top(&state->gPath.pq);
	}
	//Check if a valid path was found at all.
	if(state->gPath.rhs[state->gPath.startLoc[0]][state->gPath.startLoc[1]] >= MAX_COST){
			printf("No valid path to target found\n");
			return false;
	}
	return true;
}

void initialize(state_t *state)
{
    state->gPath.startLoc[1] = round(-state->map_startx / CELL_WIDTH);
    state->gPath.startLoc[0] = round(-state->map_starty / CELL_WIDTH);

    state->gPath.goalLoc[1]  = round((state->goal[0] - state->map_startx) / CELL_WIDTH);
    state->gPath.goalLoc[0]  = round((state->goal[1] - state->map_starty) / CELL_WIDTH);
	state->gPath.gridWidth   = state->map_size_w;
	state->gPath.gridHeight  = state->map_size_h;
	state->gPath.first = true;
	//Initialize the additive key to 0.
    state->gPath.km = 0;
    //Construct the cost map, g, and rhs.
    state->gPath.cost = malloc(state->gPath.gridHeight * sizeof(int*));
    state->gPath.g = malloc(state->gPath.gridHeight * sizeof(int*));
    state->gPath.rhs = malloc(state->gPath.gridHeight * sizeof(int*));
    for(int i = 0; i < state->gPath.gridHeight; ++i) {
    	state->gPath.cost[i] = malloc(state->gPath.gridWidth * sizeof(int));
    	state->gPath.g[i] = malloc(state->gPath.gridWidth * sizeof(int));
    	state->gPath.rhs[i] = malloc(state->gPath.gridWidth * sizeof(int));
    	for(int j = 0; j < state->gPath.gridWidth; ++j) {
    		state->gPath.cost[i][j] = 2;
    		state->gPath.g[i][j] = MAX_COST;
    		state->gPath.rhs[i][j] = MAX_COST;
    	}
    }
    //Set rhs of goal position to 0.
    state->gPath.rhs[state->gPath.goalLoc[0]][state->gPath.goalLoc[1]] = 0;
    double diff1 = state->gPath.startLoc[0] - state->gPath.goalLoc[0];
    double diff2 = state->gPath.startLoc[1] - state->gPath.goalLoc[1];
    double h = sqrt(diff1*diff1+diff2*diff2);
    Q temp;
    temp.loc[0] = state->gPath.goalLoc[0];
    temp.loc[1] = state->gPath.goalLoc[1];
    temp.key[0] = h;
    temp.key[1] = 0;
    //Initialize the priorityQueue
    _priorityQueue_init(&state->gPath.pq);
    _priorityQueue_insert(&state->gPath.pq,temp);
}

void calcKey(state_t *state, int x, int y, double key[2])
{
	//Use as the heurisitic the distance
	int diff1 = x - state->gPath.startLoc[0];
	int diff2 = y - state->gPath.startLoc[1];
	double h = sqrt(diff1*diff1 + diff2*diff2);
	int g = state->gPath.g[x][y];
	int rhs = state->gPath.rhs[x][y];
	key[0] = min(g,rhs) + h + state->gPath.km;
	key[1] = min(g,rhs);
}

void updateVertex(state_t *state, int x, int y)
{
	//If it is believed to be a wall, don't update.
	if(state->gPath.cost[x][y] >= MAX_COST) {
		return;
	}
	//If it's not the goal location, update rhs value.
	if(!(x == state->gPath.goalLoc[0] && y == state->gPath.goalLoc[1])) {
		state->gPath.rhs[x][y] = minSuccess(state,x,y);
	}

	//Grab the element if the element is in the priorityQueue.
	Q *el = _priorityQueue_get(&state->gPath.pq,x,y);
	int g = state->gPath.g[x][y];
	int rhs = state->gPath.rhs[x][y];
	//If the element was in the priority queue, remove it.
	if(el != NULL) {
		_priorityQueue_removeLoc(&state->gPath.pq,x,y);
	}
	//Add the element back to the priority queue if it is inconsistent.
	if(g != rhs) {
		Q temp;
		temp.loc[0] = x;
		temp.loc[1] = y;
		calcKey(state,x,y,temp.key);
		_priorityQueue_insert(&state->gPath.pq,temp);
	}
}

void successors(state_t *state, int locx, int locy)
{
	int x,y;
	//Update all successor vertices.
	for(int i = -1; i <= 1; ++i) {
		for(int j = -1; j <= 1; ++j) {
			x = locx + i;
			y = locy + j;
			if(x >= 0 && x < state->gPath.gridHeight && y >= 0 && y < state->gPath.gridWidth &&
				!(x == locx && y == locy)) {
				//Update rhs
				state->gPath.rhs[x][y] = min(state->gPath.rhs[x][y],state->gPath.cost[locx][locy]+state->gPath.g[locx][locy]);
				updateVertex(state,x,y);
			}
		}
	}
}

int minSuccess(state_t *state, int locx, int locy)
{
	int x,y;
	int minCost = INT_MAX;
	int thisCost;
	for(int i = -1; i <= 1; ++i) {
		for(int j = -1; j <= 1; ++j) {
			x = locx + i;
			y = locy + j;
			if(	x >= 0 && x < state->gPath.gridHeight && y >= 0 && y < state->gPath.gridWidth &&
				!(x == locx && y == locy)) {
					thisCost = state->gPath.cost[x][y]+state->gPath.g[x][y];
					if(thisCost < minCost) {
						minCost = thisCost;
					}
			}
		}
	}
	return minCost;
}

void minSuccessInd(state_t *state)
{
	int x,y, locx, locy;
	locx = state->gPath.startLoc[0];
	locy = state->gPath.startLoc[1];
	double minCost = INT_MAX;
	double thisCost;
	for(int i = -1; i <= 1; ++i) {
		for(int j = -1; j <= 1; ++j) {
			x = locx + i;
			y = locy + j;
			if(	x >= 0 && x < state->gPath.gridHeight && y >= 0 && y < state->gPath.gridWidth &&
				!(x == locx && y == locy)) {
					thisCost = state->gPath.cost[x][y]+state->gPath.g[x][y];
					if(thisCost < minCost) {
						minCost = thisCost;
						state->gPath.startLoc[0] = x;
						state->gPath.startLoc[1] = y;
					}
			}
		}
	}
}

void successorSelf(state_t *state, int locx, int locy, int gold, bool update)
{
	//Update all successor vertices
	int x,y;
	for(int i = -1; i <= 1; ++i) {
		for(int j = -1; j <= 1; ++j) {
			x = locx + i;
			y = locy + j;
			if(x >= 0 && x < state->gPath.gridHeight && y >= 0 && y < state->gPath.gridWidth) {
				if(!update) {
					if(state->gPath.rhs[x][y] == state->gPath.cost[locx][locy]+gold) {
						if(!(x == state->gPath.goalLoc[0] && y == state->gPath.goalLoc[1])) {
							//Find the minimum of the successors
							state->gPath.rhs[x][y] = minSuccess(state,locx,locy);
						}
					}
				} else {
					//gold is actually cold in this case.
					if(state->gPath.rhs[x][y] == gold + state->gPath.g[locx][locy]) {
						if(!(x == state->gPath.goalLoc[0] && y == state->gPath.goalLoc[1])) {
							//Find the minimum of the successors
							state->gPath.rhs[x][y] = minSuccess(state,locx,locy);
						}
					}
				}
				updateVertex(state,x,y);
			}
		}
	}
}
void convertOccupancy(state_t *state) 
{
	double prob;
	int x,y;
	int dialateWidth = 2;
	double dialateStrength = 40;
	double dist;
	for(int i = 0; i < state->gPath.gridHeight; ++i) {
		for(int j = 0; j < state->gPath.gridWidth; ++j) {
			prob = state->ogmap[i*state->gPath.gridWidth+j].prob;
			if(prob <= 0.5) {
				//state->gPath.cost[i][j] = 2;
			} else if(prob > 0.8) {
				state->gPath.cost[i][j] = MAX_COST;
				for(int k = -dialateWidth; k <= dialateWidth; ++k) {
					for(int m = -dialateWidth; m <= dialateWidth; ++m) {
						x = k+i;
						y = m+j;
						if(x >= 0 && x < state->gPath.gridHeight && y >= 0 && y < state->gPath.gridWidth && !(k == 0 && m == 0)) {
							dist = (abs(k)+abs(m));
							state->gPath.cost[x][y] = max(min(dialateStrength/dist,MAX_COST),state->gPath.cost[x][y]);
						}
					}
				}
			} else {
				state->gPath.cost[i][j] = MAX_COST/2;
			}
		}
	}
}
