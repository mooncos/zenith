/*
 * fifo_queue.c
 *
 *  Created on: May 16, 2022
 *      Author: Marcos Gómez
 */



#include "fifo_queue.h"
#if USE_DYNAMIC_MALLOC == 1
__IO struct Queue* createQueue(uint8_t capacity) {
	__IO struct Queue *q = (__IO struct Queue*) malloc(sizeof(struct Queue));
	q->capacity = capacity;
	q->front = q->size = 0;

	// This is important, see the enqueue
	q->rear = capacity - 1;
	q->packetPointerArray = (uint8_t**) malloc(capacity * sizeof(uint8_t*));
	return q;
}
#else
void initAllocatedQueueWithArrayCapacity(__IO struct Queue *q, uint8_t **array, uint8_t capacity) {
	q->capacity = capacity;
	q->front = q->size = 0;
	q->rear = capacity - 1;
	q->packetPointerArray = array;
}
#endif

uint8_t isFull(__IO struct Queue *q) {
	return (q->size == q->capacity);
}

uint8_t isEmpty(__IO struct Queue *q) {
	return (q->size == 0);
}

void enq(__IO struct Queue *q, uint8_t *pointer) { // enqueue element into packet array
	if (isFull(q))
		return;
	q->rear = (q->rear + 1) % q->capacity;
	q->packetPointerArray[q->rear] = pointer;
	q->size = q->size + 1;
}

uint8_t* deq(__IO struct Queue *q) { // dequeue element from packet array
	if (isEmpty(q))
		return NULL;
	uint8_t *item = q->packetPointerArray[q->front];
	q->front = (q->front + 1) % q->capacity;
	q->size = q->size - 1;
	return item;
}

uint8_t* front(__IO struct Queue *q) {
	if (isEmpty(q))
		return NULL;
	return q->packetPointerArray[q->front];

}
uint8_t* rear(__IO struct Queue *q) {
	if (isEmpty(q))
		return NULL;
	return q->packetPointerArray[q->rear];

}

void move_front(__IO struct Queue *q) {
	q->front = (q->front + 1) % q->capacity;
	q->size = q->size - 1;
}

void move_rear(__IO struct Queue *q) {
	q->rear = (q->rear + 1) % q->capacity;
	q->size = q->size + 1;
}
