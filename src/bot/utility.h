#ifndef _UTILITY
#define _UTILITY

void printStep(state_t *state, int step)
{
	printf("STEP %d\n",step);
	printf("g ----------------------------------\n");
	for(int i = 0; i < state->gPath.gridHeight; ++i) {
		for(int j = 0; j < state->gPath.gridWidth; ++j) {
			printf("%5d\t",state->gPath.g[i][j]);
		}
		printf("\n");
	}
	printf("rhs -------------------------------\n");
	for(int i = 0; i < state->gPath.gridHeight; ++i) {
		for(int j = 0; j < state->gPath.gridWidth; ++j) {
			printf("%5d\t",state->gPath.rhs[i][j]);
		}
		printf("\n");
	}
	printf("cost ------------------------------\n");
	for(int i = 0; i < state->gPath.gridHeight; ++i) {
		for(int j = 0; j < state->gPath.gridWidth; ++j) {
			printf("%5d\t",state->gPath.cost[i][j]);
		}
		printf("\n");
	}
}

double min(double lhs, double rhs)
{
	if(lhs < rhs)
		return lhs;
	return rhs;
}

double max(double lhs, double rhs)
{
	if(lhs > rhs)
		return lhs;
	return rhs;
}

#endif