#include "priorityQueue.h"
#include <limits.h>
#include <stdlib.h>
#include <stdio.h>

//Increase the capacity if need be.
void _increase_capacity(PriorityQueue *self)
{
	//Make sure there's room to expand into.
	if((self->size) >= self->capacity) {
		self->capacity *= 2;
		self->data = realloc(self->data, sizeof(Q) * self->capacity);
		if(self->data == NULL) {
 			printf("priorityQueue::_increase_capacity memory not allocated.");
 			exit(0);
 		}
	}
}

void _priorityQueue_init(PriorityQueue *self)
{
	//Set the initial size and capacity.
	self->size = 0;
	self->capacity = PRIORITY_QUEUE_INITIAL_CAPACITY;

	//Allocate memory for vector->data
 	self->data = malloc(sizeof(Q) * self->capacity);
 	if(self->data == NULL) {
 		printf("priorityQueue::_priorityQueue_init memory not allocated.");
 		exit(0);
 	}
}
//Really bad implementation for now!!
Q* _priorityQueue_top(PriorityQueue *self)
{
	double minKey = INT_MAX;
	int index = -1;
	for(int i = 0; i < self->size; ++i) {
		//printf("key[%d] = %f,%f\n",i,self->data[i].key[0],self->data[i].key[1]);
		if(self->data[i].key[0] < minKey) {
			minKey = self->data[i].key[0];
			index = i;
		}
	}
	if(index != -1)
		return &(self->data[index]);
	return NULL;
}

Q* _priorityQueue_get(PriorityQueue *self, int x, int y)
{
	for(int i = 0; i < self->size; ++i) {
	//	printf("loc[%d] = %d, %d\n",i,self->data[i].loc[0],self->data[i].loc[1]);
		if(self->data[i].loc[0] == x && self->data[i].loc[1] == y) {
			return &(self->data[i]);
		}
	}
	return NULL;
}
//Really bad implementation for now!!
void _priorityQueue_remove(PriorityQueue *self, Q el)
{
	int index = -1;
	for(int i = 0; i < self->size; ++i) {
		if(self->data[i].loc[0] == el.loc[0] && self->data[i].loc[1] == el.loc[1]) {
			index = i;
		}
	}
	//If we found the element, lets pull it out and slide everything after it up one.
	if(index != -1) {
		for(int i = index; i < self->size-1; ++i) {
			self->data[i] = self->data[i+1];
		}
	}
}

void _priorityQueue_removeLoc(PriorityQueue *self, int x, int y) 
{
	int index = -1;
	for(int i = 0; i < self->size; ++i) {
		if(self->data[i].loc[0] == x && self->data[i].loc[1] == y) {
			index = i;
		}
	}
	//If we found the element, lets pull it out and slide everything after it up one.
	if(index != -1) {
		for(int i = index; i < self->size-1; ++i) {
			self->data[i] = self->data[i+1];
		}
	}
	//Subtract one from the size.
	self->size--;
}

void _priorityQueue_insert(PriorityQueue *self, Q el)
{
	/*printf("%d, %d \n",el.loc[0],el.loc[1]);
	//Check if it is already in the priority queue
	for(int i = 0; i < self->size; ++i){
		if(self->data[i].loc[0] == el.loc[0] && self->data[i].loc[1] == el.loc[1]) {
			return;
		}
	}*/
	//Increase the capacity if need be.
	_increase_capacity(self);
	//Add the element and incrment size.
	self->data[self->size++] = el;
}

//Print out the contents
void _priorityQueue_print(PriorityQueue *self)
{
	for(int i = 0; i < self->size; ++i) {
		printf("Key[%d] = [%f, %f] Loc[%d] = [%d, %d]\n",i,self->data[i].key[0],self->data[i].key[1],i,self->data[i].loc[0],self->data[i].loc[1]);
	}
	printf("\n");
}

//Free the vectors memory.
void _priorityQueue_free(PriorityQueue *self) { free(self->data); }