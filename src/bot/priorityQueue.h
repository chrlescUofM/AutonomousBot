/* priorityQueue.h
 * Author: Chris Lesch
 * Date: 4/3/15
 */

#ifndef _PRIORITY_QUEUE
#define _PRIORITY_QUEUE

#define PRIORITY_QUEUE_INITIAL_CAPACITY 100


typedef struct
{
	int loc[2];
	double key[2];
} Q;

typedef struct
{
	int size;
	int capacity;
	Q *data;
} PriorityQueue;

void _priorityQueue_init(PriorityQueue *self);
Q* _priorityQueue_top(PriorityQueue *self);
Q* _priorityQueue_get(PriorityQueue *self, int x, int y);
void _priorityQueue_remove(PriorityQueue *self, Q el);
void _priorityQueue_removeLoc(PriorityQueue *self, int x, int y);
void _priorityQueue_insert(PriorityQueue *self, Q el);
void _priorityQueue_free(PriorityQueue *self);
void _priorityQueue_print(PriorityQueue *self);

#endif