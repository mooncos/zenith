/*
 * fifo_queue.h
 *
 *  Created on: May 16, 2022
 *      Author: Marcos Gómez
 */

#ifndef FIFO_QUEUE_H_
#define FIFO_QUEUE_H_

#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include "main.h"

struct Queue {
	uint8_t front, rear, size; // front, rear indexes and size of queue
	uint8_t capacity; // maximum capacity for queue, this shall be set for the moment to 16
	uint8_t **packetPointerArray; // array holding the pointers to the blePackets to transmit
};
#if USE_DYNAMIC_MALLOC == 1
__IO struct Queue* createQueue(uint8_t capacity);
#else
void initAllocatedQueueWithArrayCapacity(__IO struct Queue *q, uint8_t **array,
		uint8_t capacity);
#endif
uint8_t isFull(__IO struct Queue *q);
uint8_t isEmpty(__IO struct Queue *q);
void enq(__IO struct Queue *q, uint8_t *pointer); // enqueue element into packet array
uint8_t* deq(__IO struct Queue *q); // dequeue element from packet array
uint8_t* front(__IO struct Queue *q);
uint8_t* rear(__IO struct Queue *q);
void move_front(__IO struct Queue *q);
void move_rear(__IO struct Queue *q);

#endif /* FIFO_QUEUE_H_ */
