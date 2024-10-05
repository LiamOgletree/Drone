/*
 * LIS2MDL.h
 *
 *  Created on: Sep 3, 2024
 *      Author: liamt
 */

#ifndef INC_LIS2MDL_H_
#define INC_LIS2MDL_H_

#include "main.h"

typedef struct LIS2MDL {
    int16_t X, Y, Z;
    uint32_t timestamp;
    float heading;
} LIS2MDL;

typedef enum {
    LIS2MDL_SUCCESS,
    LIS2MDL_FAILURE
} LIS2MDL_STATUS;

LIS2MDL_STATUS LIS2MDL_Setup(SPI_HandleTypeDef * const hspi);
LIS2MDL_STATUS LIS2MDL_Read(LIS2MDL * const lis2mdl,
                            SPI_HandleTypeDef * const hspi);

#endif /* INC_LIS2MDL_H_ */
