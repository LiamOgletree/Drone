/*
 * ringbufferStateMachine.h
 *
 *  Created on: Oct 3, 2024
 *      Author: liamt
 */

#ifndef INC_RINGBUFFERSTATEMACHINE_H_
#define INC_RINGBUFFERSTATEMACHINE_H_

#include "main.h"

typedef enum {
    SM_ACCELEROMETER,
    SM_PRESSURE,
    SM_GPS
} SM_UPDATE_TYPE;

typedef struct {
    int type;
    union {
        BMP388 bmp388;
        LIS2MDL lis2mdl;
        LSM6DSO32 lsm6dso32;
    };
} RingBufferSM_t;

typedef struct {
    uint32_t capacity; // max size
    uint32_t size;     // current size ("occupancy")
    uint32_t head;
    uint32_t tail;
    RingBufferSM_t *buffer;
} RingBufferSM;

typedef enum {
    RB_SM_SUCCESS,
    RB_SM_FAILURE
} RB_STATUS_SM;

RB_STATUS_SM RingBufferSM_ctor(RingBufferSM * const rb,
                               uint32_t const capacity,
                               RingBufferSM_t * const buffer);
RB_STATUS_SM RingBufferSM_enqueue(RingBufferSM * const rb,
                                  RingBufferSM_t const item);
RB_STATUS_SM RingBufferSM_dequeue(RingBufferSM * const rb,
                                  RingBufferSM_t * const item);

#endif /* INC_RINGBUFFERSTATEMACHINE_H_ */
