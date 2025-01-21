/*
 * sensorPA1616S.h
 *
 *  Created on: Oct 26, 2024
 *      Author: liamt
 */

#ifndef INC_SENSORPA1616S_H_
#define INC_SENSORPA1616S_H_

#include "main.h"

#define PA1616S_BUF_SIZE (1024)

typedef struct {
    uint16_t size;
    uint16_t  index;
    uint8_t * buf;
} PA1616S_IDLE;

typedef struct {
    float latitude;
    float longitude;
    float altitude;
    float speed;
    float heading;
} PA1616S;

typedef enum {
    PA1616S_SUCCESS,
    PA1616S_FAILURE
} PA1616S_STATUS;

PA1616S_STATUS PA1616S_Setup(UART_HandleTypeDef * const huart,
                             uint8_t * const buf);

#endif /* INC_SENSORPA1616S_H_ */
