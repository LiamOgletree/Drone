/*
 * ringbuffer.h
 *
 *  Created on: Jan 13, 2024
 *      Author: oglet
 */

#ifndef SRC_RINGBUFFERUART_H_
#define SRC_RINGBUFFERUART_H_

#include "main.h"
#include "BMP388.h"
#include "LIS2MDL.h"
#include "LSM6DSO32.h"

typedef enum {
    UPDATE_BMP388,
    UPDATE_LIS2MDL,
    UPDATE_LSM6DSO32,
    UPDATE_ERROR
} UART_UPDATE_TYPE;

typedef struct {
    int type;
    union {
        BMP388 bmp388;
        LIS2MDL lis2mdl;
        LSM6DSO32 lsm6dso32;
        char * error_buf;
    };
} RingBufferUART_t;

typedef struct {
    uint32_t capacity; // max size
    uint32_t size;     // current size ("occupancy")
    uint32_t head;
    uint32_t tail;
    RingBufferUART_t *buffer;
} RingBufferUART;

typedef enum {
    RB_UART_SUCCESS,
    RB_UART_FAILURE
} RB_STATUS_UART;

RB_STATUS_UART RingBufferUART_ctor(RingBufferUART * const rb,
                                   uint32_t const capacity,
                                   RingBufferUART_t * const buffer);
RB_STATUS_UART RingBufferUART_enqueue(RingBufferUART * const rb,
                                      RingBufferUART_t const item);
RB_STATUS_UART RingBufferUART_dequeue(RingBufferUART * const rb,
                                      RingBufferUART_t * const item);

#endif /* SRC_RINGBUFFER_H_ */
