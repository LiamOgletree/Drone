/*
 * ringbuffer.h
 *
 *  Created on: Jan 13, 2024
 *      Author: oglet
 */

#ifndef SRC_RINGBUFFERUART_H_
#define SRC_RINGBUFFERUART_H_

#include <sensorBMP388.h>
#include <sensorLIS2MDL.h>
#include <sensorLSM6DSO32.h>
#include <sensorPA1616S.h>
#include <StateMachine.h>
#include "main.h"

typedef enum {
    RB_PRESSURE,
    RB_TEMPERATURE,
    RB_MAGNETOMETER,
    RB_GYROSCOPE,
    RB_ACCELEROMETER,
    RB_GPS,
    RB_GPS_IDLE,
    RB_STATEMACHINE,
    RB_ERROR
} RB_TYPE;

typedef struct {
    int type;
    union {
        BMP388 bmp388;
        LIS2MDL lis2mdl;
        LSM6DSO32 lsm6dso32;
        StateMachine_t state;
        PA1616S pa1616s;
        PA1616S_IDLE pa1616s_idle;
        char * error_buf;
    };
} RingBuffer_t;

typedef struct {
    uint32_t capacity;
    uint32_t size;
    uint32_t head;
    uint32_t tail;
    RingBuffer_t *buffer;
} RingBuffer;

typedef enum {
    RB_SUCCESS,
    RB_FAILURE
} RB_STATUS;

RB_STATUS RingBuffer_ctor(RingBuffer * const rb,
                          uint32_t const capacity,
                          RingBuffer_t * const buffer);
RB_STATUS RingBuffer_enqueue(RingBuffer * const rb,
                             RingBuffer_t const item);
RB_STATUS RingBuffer_dequeue(RingBuffer * const rb,
                             RingBuffer_t * const item);

#endif /* SRC_RINGBUFFER_H_ */
