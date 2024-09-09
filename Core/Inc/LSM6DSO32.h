/*
 * LSM6DSO32.h
 *
 *  Created on: Sep 6, 2024
 *      Author: liamt
 */

#ifndef INC_LSM6DSO32_H_
#define INC_LSM6DSO32_H_

#include "main.h"

typedef struct LSM6DSO32 {
    float G_X, G_Y, G_Z;
    float A_X, A_Y, A_Z;
} LSM6DSO32;

typedef enum {
    LSM6DSO32_SUCCESS,
    LSM6DSO32_FAILURE
} LSM6DSO32_STATUS;

LSM6DSO32_STATUS LSM6DSO32_Setup(LSM6DSO32 * const lsm6dso32,
                                 SPI_HandleTypeDef * const hspi);
LSM6DSO32_STATUS LSM6DSO32_Read(LSM6DSO32 * const lsm6dso32,
                                SPI_HandleTypeDef * const hspi);

#endif /* INC_LSM6DSO32_H_ */
