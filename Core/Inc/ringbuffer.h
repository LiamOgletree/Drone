/*
 * ringbuffer.h
 *
 *  Created on: Jan 13, 2024
 *      Author: oglet
 */

#ifndef SRC_RINGBUFFER_H_
#define SRC_RINGBUFFER_H_

#include "main.h"
#include "BMP388.h"
#include "LIS2MDL.h"
#include "LSM6DSO32.h"

enum {
	UPDATE_BMP388,
	UPDATE_LIS2MDL,
	UPDATE_LSM6DSO32
};

typedef struct {
	int type;
	union {
		BMP388 bmp388;
		LIS2MDL lis2mdl;
		LSM6DSO32 lsm6dso32;
	};
} RingBuffer_t;

typedef struct RingBuffer {
	uint32_t capacity; // max size
	uint32_t size; // current size ("occupancy")
	uint32_t head;
	uint32_t tail;
	RingBuffer_t *buffer;
} RingBuffer;

typedef enum {
  RB_SUCCESS,
  RB_FAILURE
} RB_STATUS;

RB_STATUS RingBuffer_ctor(RingBuffer * const rb, uint32_t const capacity, RingBuffer_t * const buffer);
RB_STATUS RingBuffer_enqueue(RingBuffer * const rb, RingBuffer_t const item);
RB_STATUS RingBuffer_dequeue(RingBuffer * const rb, RingBuffer_t * const item);

#endif /* SRC_RINGBUFFER_H_ */
